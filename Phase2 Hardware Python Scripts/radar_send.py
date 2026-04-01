"""
pi_radar_streamer.py  —  runs on Raspberry Pi
Reads BGT60TR13C radar via SPI, processes frames,
streams JSON tracks to Jetson at 172.20.10.2:9576.

USAGE:  python pi_radar_streamer.py [/path/to/config_3rx_5m]
INSTALL: pip install numpy scipy spidev gpiozero
"""
import sys, os, re, json, struct, threading, time, queue, logging, socket
from dataclasses import dataclass, field
from collections import deque
from typing import List, Optional

import numpy as np
from scipy.signal import convolve2d
from scipy.ndimage import label as sp_label

logging.basicConfig(level=logging.WARNING)

try:
    import pyfftw.interfaces.numpy_fft as fft_mod
    import pyfftw; pyfftw.interfaces.cache.enable()
except ImportError:
    import numpy.fft as fft_mod

try:
    import numba
    @numba.vectorize([numba.float32(numba.complex64)])
    def _abs2(x): return x.real**2 + x.imag**2
except ImportError:
    def _abs2(x): return (x.real**2 + x.imag**2).astype(np.float32)

# ── Network ────────────────────────────────────────────────────────────────
JETSON_IP   = "172.20.10.2"  # FIXED: Jetson Orin Nano IP
JETSON_PORT = 9576

# ── Radar mounting orientation ─────────────────────────────────────────────
# Radar mounted sideways: antenna array faces forward, USB-C connector on top
RADAR_ROTATION_DEG = -90  # Try +90 if detections are still missing

# ── BGT60TR13C constants ───────────────────────────────────────────────────
_R = lambda n: n  # just a tag for readability
SPI_WR      = 0x01000000
SPI_RADR    = 0xFE000000; SPI_RADR_POS = 25
SPI_DATA    = 0x00FFFFFF
SPI_BURST   = 0xFF000000; SPI_BURST_SADR_POS = 17
REG_MAIN    = 0x00; REG_CHIP_ID = 0x02; REG_SFCTL = 0x06
REG_FSTAT   = 0x5f; REG_FIFO   = 0x60
FSTAT_FOF   = 0x800000; FSTAT_FUF = 0x080000
FSTAT_SBE   = 0x040000; FSTAT_CKE = 0x020000
FSTAT_CREF  = 0x200000; FSTAT_CREF_POS = 21
FSTAT_FOF_POS=23; FSTAT_FUF_POS=19; FSTAT_SBE_POS=18; FSTAT_CKE_POS=17
SFCTL_CREF  = 0x001fff; SFCTL_CREF_POS = 0
SFCTL_MISO  = 0x010000
CHIP_DIG    = 0xffff00; CHIP_DIG_POS = 8
CHIP_RF     = 0x0000ff; CHIP_RF_POS  = 0
MAIN_FSTART = 0x000001; MAIN_RST_POS = 1
RST_SW      = 0x1 << MAIN_RST_POS
FIFO_SIZE   = 8192
GSR_ERR     = 0x08|0x02|0x01
RET_OK=0; RET_ERR=-1


class BGT60TR13C:
    def __init__(self, spi_bus=0, spi_dev=0, spi_speed=50_000_000,
                 rst_pin=12, irq_pin=25, version=0):
        import spidev
        from gpiozero import DigitalInputDevice, DigitalOutputDevice, Device
        # Pi 5 needs lgpio; Pi 4 uses rpigpio. Try in order.
        try:
            from gpiozero.pins.lgpio import LGPIOFactory
            Device.pin_factory = LGPIOFactory()
        except Exception:
            try:
                from gpiozero.pins.pigpio import PiGPIOFactory
                Device.pin_factory = PiGPIOFactory()
            except Exception:
                from gpiozero.pins.rpigpio import RPiGPIOFactory
                Device.pin_factory = RPiGPIOFactory()
        self._spi = spidev.SpiDev(); self._spi.open(spi_bus,spi_dev)
        self._spi.max_speed_hz=spi_speed; self._spi.mode=0
        self._rst = DigitalOutputDevice(rst_pin)
        self._irq = DigitalInputDevice(irq_pin, pull_up=False)
        self.hard_reset()
        self.frame_buffer = queue.Queue(maxsize=256)
        self._sub = []; self._gsr = 0
        self._nbytes = 0; self._nburst = 0
        self._fp=None; self._fi=None; self._fb=None
        self._reg_file = None; self._stop = threading.Event()
        self._thread=None; self._ver=version; self._seq=0

    def _sr(self, addr, data):
        t=((addr<<SPI_RADR_POS)&SPI_RADR|SPI_WR|(data<<0)&SPI_DATA)
        tx=[t>>24&0xFF,t>>16&0xFF,t>>8&0xFF,t&0xFF]
        rx=self._spi.xfer2(tx); self._gsr=rx[0]

    def _gr(self, addr):
        t=(addr<<SPI_RADR_POS)&SPI_RADR
        tx=[t>>24&0xFF,t>>16&0xFF,t>>8&0xFF,t&0xFF]
        rx=self._spi.xfer2(tx); self._gsr=rx[0]
        return (rx[0]<<24)|(rx[1]<<16)|(rx[2]<<8)|rx[3]

    def _fifo(self, n):
        if 0<n>>1<=FIFO_SIZE and n%2==0:
            tx=([SPI_BURST>>24&0xFF,REG_FIFO<<SPI_BURST_SADR_POS>>16&0xFF,0,0]+[0]*((n>>1)*3))
            rx=self._spi.xfer2(tx); self._gsr=rx[0]
            if not(self._gsr&GSR_ERR): return rx[4:]
        return []

    def _chk_fifo(self):
        s=self._gr(REG_FSTAT)
        return (s&FSTAT_FOF)>>FSTAT_FOF_POS,(s&FSTAT_CREF)>>FSTAT_CREF_POS,(s&FSTAT_FUF)>>FSTAT_FUF_POS,(s&FSTAT_SBE)>>FSTAT_SBE_POS,(s&FSTAT_CKE)>>FSTAT_CKE_POS

    def check_chip_id(self):
        c=self._gr(REG_CHIP_ID)
        return RET_OK if ((c&CHIP_DIG)>>CHIP_DIG_POS==3 and (c&CHIP_RF)>>CHIP_RF_POS==3) else RET_ERR

    def set_register_config_file(self,fn): self._reg_file=fn

    def _load_regs(self):
        with open(self._reg_file) as f:
            for line in f:
                p=line.strip().split()
                if len(p)==3:
                    a=int(p[1],16); v=int(p[2],16)
                    if a==REG_SFCTL:
                        if self._spi.max_speed_hz>20_000_000: v|=SFCTL_MISO
                        else: v&=~SFCTL_MISO
                    self._sr(a,v)

    def set_fifo_parameters(self,nframe,nirq,nburst):
        self._fp=nframe; self._fi=nirq; self._fb=nburst

    def _apply_fifo(self):
        self._nbytes=(self._fp>>1)*3; self._nburst=self._fb
        t=self._gr(REG_SFCTL); t&=~SFCTL_CREF
        t|=(((self._fi>>1)-1)<<SFCTL_CREF_POS)&SFCTL_CREF
        self._sr(REG_SFCTL,t)

    def hard_reset(self):
        self._rst.on();time.sleep(0.01);self._rst.off();time.sleep(0.01);self._rst.on();time.sleep(0.01)

    def soft_reset(self,rt):
        self._sub=[]
        t=self._gr(REG_MAIN)|rt; self._sr(REG_MAIN,t)
        for _ in range(10):
            time.sleep(0.01)
            if not(self._gr(REG_MAIN)&rt): break
        time.sleep(0.01)

    def start(self):
        self._stop.clear()
        self._thread=threading.Thread(target=self._collect,daemon=True)
        self._thread.start()

    def stop(self):
        self.soft_reset(RST_SW); self._stop.set()
        if self._thread: self._thread.join()

    def _collect(self):
        while not self._stop.is_set():
            self.soft_reset(RST_SW); self._load_regs(); self._apply_fifo()
            self._seq=0
            t=self._gr(REG_MAIN)|MAIN_FSTART; self._sr(REG_MAIN,t)
            err=False
            while not self._stop.is_set():
                time.sleep(0.001)
                if err: break
                while True:
                    FOF,CRF,FUF,SBE,CKE=self._chk_fifo()
                    if FOF or FUF or SBE or CKE: err=True; break
                    if not CRF: break
                    data=self._fifo(self._nburst)
                    self._sub+=data
                    while len(self._sub)>=self._nbytes:
                        frame=bytes(self._sub[:self._nbytes])
                        self._sub=self._sub[self._nbytes:]
                        pkt=(self._ver.to_bytes(4,'little')+
                             self._seq.to_bytes(4,'little')+
                             len(frame).to_bytes(4,'little')+frame)
                        self._seq+=1
                        try: self.frame_buffer.put_nowait(pkt)
                        except queue.Full: pass

    def __del__(self):
        try: self.stop()
        except: pass


# ── Config ─────────────────────────────────────────────────────────────────
def find_file(d,pat):
    hits=sorted([f for f in os.listdir(d) if re.match(pat,f)],reverse=True)
    if not hits: raise FileNotFoundError(f"No {pat} in {d}")
    return os.path.join(d,hits[0])

def load_config(cfg_dir):
    reg=find_file(cfg_dir,r"BGT60TR13C_export_registers_\d{8}-\d{6}\.txt")
    sj=find_file(cfg_dir,r"BGT60TR13C_settings_\d{8}-\d{6}\.json")
    with open(sj) as f: s=json.load(f)
    total=0
    for frame in s["sequence"][0]["sequence"]:
        if frame["type"]=="loop":
            sp=0
            for ch in frame["sequence"]:
                if ch["type"]=="chirp": sp+=ch["num_samples"]*bin(ch["rx_mask"]).count("1")
            total+=sp*frame["num_repetitions"]
    return s,reg,total

def parse_cfg(s):
    loop=s["sequence"][0]["sequence"][0]; ch=loop["sequence"][0]
    return dict(frame_rate=1/s["sequence"][0]["repetition_time_s"],
                chirp_rate=1/loop["repetition_time_s"],
                num_chirps=loop["num_repetitions"],num_samples=ch["num_samples"],
                bandwidth=ch["end_frequency_Hz"]-ch["start_frequency_Hz"],
                start_freq=ch["start_frequency_Hz"],end_freq=ch["end_frequency_Hz"],
                rx_mask=ch["rx_mask"],num_antennas=bin(ch["rx_mask"]).count("1"))


# ── Radar processing ────────────────────────────────────────────────────────
C=3e8; PM={1:(1,0),2:(0,1),3:(0,0)}; NAZ=2; MINR=0.2
CFAR_DB=12.0; CGR=2;CTR=6;CGD=1;CTD=4   # LOWERED from 16.0 for more sensitivity
MMISS=3;  GR=0.12; GV=0.40; SVEL=0.20    # SVEL widened to 0.20 (was 0.15)
TIMMINENT=1.5; TCAUTION=3.0; RIMMINENT=0.50; RCAUTION=1.20
STATIC_CONFIRM_AGE  = 20
STATIC_MIN_PWR_DB   = 0.0
# Minimum consecutive frames a track must be APPROACHING before alarming.
# Prevents single-frame velocity spikes from triggering false IMMINENT.
APPROACH_CONFIRM    = 3

# Angular FOV filter — DISABLED FOR DEBUG
FOV_DEG = 90.0   # WIDENED from 45.0 to see all detections
# Number of angle FFT bins — zero-pad the 2×2 virtual aperture for finer resolution
NANGLE  = 64

class RadarCube:
    def __init__(self,p,mti=0.92):  # high MTI memory — aggressively suppresses static clutter
        self.p=p; self.mti=mti; self._prev=None
        nd=p["num_chirps"]; nr=p["num_samples"]
        cf=(p["start_freq"]+p["end_freq"])/2
        self.cf=cf
        ra=np.arange(nr>>1)*(C/(2*p["bandwidth"]))
        self.rskip=int(np.searchsorted(ra,MINR))
        self.range_axis=ra[self.rskip:]
        self.doppler_axis=-np.fft.fftshift(np.fft.fftfreq(nd,p["chirp_rate"]))/2*C/cf
        self.active=[a for a in PM if p["rx_mask"]&(1<<(a-1))]
        self.nd=nd; self.nr=nr
        # Angle axis: sin(θ) bins from the angle FFT, converted to degrees.
        # Antenna spacing = 0.5λ for BGT60TR13C → spatial freq bins map to sin(θ)
        # via: sin(θ) = bin / NANGLE  (normalised, spacing=0.5λ)
        spatial_freq = np.fft.fftshift(np.fft.fftfreq(NANGLE))
        # sin(θ) = spatial_freq / 0.5 = 2 * spatial_freq  (0.5λ spacing)
        sin_theta = np.clip(2.0 * spatial_freq, -1.0, 1.0)
        self.angle_axis = np.degrees(np.arcsin(sin_theta))  # degrees, -90..+90
        # Precompute which angle bins are within the forward FOV
        self.fov_mask = np.abs(self.angle_axis) <= FOV_DEG

    def process(self,payload):
        d=np.frombuffer(payload,dtype=np.uint8).reshape(-1,3).astype(np.uint16)
        fst=(d[:,0]<<4)|(d[:,1]>>4); snd=((d[:,1]&0xF)<<8)|d[:,2]
        adc=np.stack([fst,snd],1).reshape(-1).astype(np.float32)
        adc=adc.reshape(self.nd,self.nr,self.p["num_antennas"])
        # Fill 2×2 virtual aperture
        cube=np.zeros((self.nd,self.nr,NAZ,NAZ),dtype=np.float32)
        for i,a in enumerate(self.active):
            r,c=PM[a]
            if r<NAZ and c<NAZ: cube[:,:,r,c]=adc[:,:,i]
        # MTI clutter filter
        if self._prev is not None:
            mti=cube-self._prev
            self._prev=self.mti*cube+(1-self.mti)*self._prev
            cube=mti
        else:
            self._prev=cube.copy(); return None, None
        # Range + Doppler FFT (axes 0=Doppler, 1=range, 2-3=virtual aperture)
        cf=fft_mod.fftn(cube.astype(np.complex64),axes=(0,1,2,3))
        cf=np.fft.fftshift(cf,axes=(0,2,3))
        # Angle FFT — zero-pad virtual aperture to NANGLE bins along azimuth axis.
        # Extract the horizontal (row) axis of the 2×2 array as the angle dimension.
        # Shape after range/Doppler FFT: (nd, nr>>1, NAZ, NAZ)
        # We use axis 2 (virtual row) as the single angle dimension, averaged over col.
        rd_angle = cf[:,self.rskip:self.nr>>1,:,:]   # (nd, nr_valid, NAZ, NAZ)
        # Average over the second aperture dimension (cols) to get 1D virtual array
        rd_1d = rd_angle.mean(axis=3)                 # (nd, nr_valid, NAZ)
        # Zero-pad to NANGLE and FFT for angle resolution
        pad = NANGLE - NAZ
        rd_padded = np.pad(rd_1d, ((0,0),(0,0),(0,pad)))  # (nd, nr_valid, NANGLE)
        angle_fft = np.fft.fftshift(np.fft.fft(rd_padded, axis=2), axes=2)
        pw_full = _abs2(angle_fft.astype(np.complex64))  # (nd, nr_valid, NANGLE)
        # Range-Doppler map: max power across all angle bins (for CFAR)
        rd_map = pw_full.max(axis=2).T                    # (nr_valid, nd)
        # Angle map: for each (range, doppler) cell, which angle bin has peak power
        angle_map = pw_full.argmax(axis=2).T              # (nr_valid, nd)
        return rd_map, angle_map

@dataclass
class Track:
    tid:int; range_m:float; vel_mps:float; pwr_db:float
    age:int=0; missed:int=0; cls:str="UNKNOWN"; level:str="SAFE"
    ttc_s:float=float("inf"); angle_deg:float=0.0
    _approach_frames:int=0   # consecutive frames classified APPROACHING

    def update(self,r,v,pw,ang=0.0):
        a=0.35
        self.range_m=a*r+(1-a)*self.range_m
        self.vel_mps=a*v+(1-a)*self.vel_mps
        self.angle_deg=a*ang+(1-a)*self.angle_deg
        self.missed=0; self.age+=1; self._derive()

    def _derive(self):
        v=self.vel_mps
        if abs(v)<SVEL: self.cls="STATIC"
        elif v<0:       self.cls="APPROACHING"
        else:           self.cls="RECEDING"
        self.ttc_s=(self.range_m/abs(v) if self.cls=="APPROACHING" and self.range_m>0 else float("inf"))
        if self.cls=="APPROACHING":
            # Increment approach counter — must be consistently approaching
            # before alarming. Resets to zero if classification changes.
            self._approach_frames += 1
            if self._approach_frames >= APPROACH_CONFIRM:
                if self.range_m<=RIMMINENT or self.ttc_s<=TIMMINENT: self.level="IMMINENT"
                elif self.range_m<=RCAUTION or self.ttc_s<=TCAUTION: self.level="CAUTION"
                else: self.level="SAFE"
            else:
                self.level="SAFE"  # not yet confirmed approaching
        else:
            self._approach_frames = 0  # reset when not approaching

        if self.cls=="STATIC":
            # Static tracks are sent with level=SAFE by default.
            # The Jetson fusion layer promotes them to CAUTION/IMMINENT
            # only when LiDAR is confirmed stale (degraded mode).
            # This prevents background clutter false alarms during normal operation.
            self.level="SAFE"
        else:
            # RECEDING — moving away, not a collision threat
            self.level="SAFE"

    def to_dict(self):
        return {"id":self.tid,"range_m":round(self.range_m,3),"vel_mps":round(self.vel_mps,3),
                "ttc_s":round(self.ttc_s,2) if self.ttc_s!=float("inf") else None,
                "level":self.level,"cls":self.cls,"angle_deg":round(self.angle_deg,1)}

class Tracker:
    def __init__(self): self._t=[]; self._nid=1
    def update(self,dets):
        um=list(dets)
        for t in self._t:
            best=None;bc=1e9
            for d in um:
                r,v,pw,ang=d
                if abs(r-t.range_m)<GR and abs(v-t.vel_mps)<GV:
                    c=abs(r-t.range_m)/GR+abs(v-t.vel_mps)/GV
                    if c<bc: bc,best=c,d
            if best: t.update(best[0],best[1],best[2],ang=best[3]); um.remove(best)
            else: t.missed+=1
        self._t=[t for t in self._t if t.missed<MMISS]
        for d in um:
            r,v,pw,ang=d; t=Track(self._nid,r,v,pw,angle_deg=ang); t._derive(); self._t.append(t); self._nid+=1
        return [t for t in self._t if t.age>=3]  # 3 frames minimum before reporting

def cfar2d(rd):
    mag=10*np.log10(rd+1e-12)
    k=np.ones((2*(CGR+CTR)+1,2*(CGD+CTD)+1))
    k[CTR:CTR+2*CGR+1,CTD:CTD+2*CGD+1]=0
    ntr=k.sum(); k/=ntr
    noise=convolve2d(mag,k,mode="same",boundary="wrap")
    mask=mag>(noise+CFAR_DB)
    lbl,n=sp_label(mask,np.ones((3,3)))
    out=[]
    for i in range(1,n+1):
        idx=np.argwhere(lbl==i);pw=mag[idx[:,0],idx[:,1]];j=np.argmax(pw)
        out.append((int(idx[j,0]),int(idx[j,1]),float(pw[j])))
    return out

def parse_pkt(raw):
    if len(raw)<12: return None
    v=struct.unpack_from("<I",raw,0)[0]; dl=struct.unpack_from("<I",raw,8)[0]
    return raw[12:] if v==0 and len(raw)-12==dl else None


# ── Main ────────────────────────────────────────────────────────────────────
def find_cfg():
    here=os.path.dirname(os.path.abspath(__file__))
    for c in [os.path.join(here,"..","radar_config","config_3rx_5m"),
              os.path.join(here,"radar_config","config_3rx_5m"),
              os.path.join(here,"config_3rx_5m")]:
        if os.path.isdir(c): return c
    for root,_,files in os.walk(os.path.expanduser("~")):
        for f in files:
            if re.match(r"BGT60TR13C_settings_.*\.json",f): return root
    return None

def main():
    cfg_dir=sys.argv[1] if len(sys.argv)>1 else find_cfg()
    if not cfg_dir or not os.path.isdir(cfg_dir):
        print("[ERROR] Config dir not found. Pass it as argument.")
        sys.exit(1)
    setting,reg_file,frame_size=load_config(cfg_dir)
    p=parse_cfg(setting)
    print(f"[RADAR]  {p['num_chirps']}chirps x {p['num_samples']}samp x {p['num_antennas']}RX  BW={p['bandwidth']/1e9:.1f}GHz")
    print(f"[NET]    Streaming JSON to {JETSON_IP}:{JETSON_PORT}")
    print(f"[MOUNT]  Radar rotation: {RADAR_ROTATION_DEG}° (USB-C on top)")
    print(f"[DEBUG]  CFAR={CFAR_DB}dB, FOV={FOV_DEG}°, SVEL={SVEL}m/s")

    sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

    print("[HW]     Init BGT60TR13C...")
    try:
        radar=BGT60TR13C(spi_speed=50_000_000)
        radar.check_chip_id()
        radar.set_register_config_file(reg_file)
        radar.set_fifo_parameters(frame_size,4096,2048)
        radar.start()
        print("[HW]     Started. MTI settling for ~2 seconds...")
        time.sleep(2)  # Let MTI filter stabilize
    except Exception as e:
        print(f"[ERROR]  {e}"); sys.exit(1)

    cube=RadarCube(p); tracker=Tracker(); sent=0
    try:
        while True:
            try: raw=radar.frame_buffer.get(timeout=1)
            except queue.Empty: print("[WAIT]"); continue
            payload=parse_pkt(raw)
            if payload is None: continue
            rd, angle_map = cube.process(payload)
            if rd is None: continue

            dets=[]
            for ri,di,pw in cfar2d(rd):
                if ri>=len(cube.range_axis): continue
                r=float(cube.range_axis[ri])
                v=float(cube.doppler_axis[di]) if di<len(cube.doppler_axis) else 0.0
                # Get angle for this detection from the angle map
                ai = int(angle_map[ri, di]) if angle_map is not None else NANGLE//2
                ang = float(cube.angle_axis[ai]) if ai < len(cube.angle_axis) else 0.0
                
                # Apply mounting rotation: radar is sideways with USB-C on top
                ang = ang + RADAR_ROTATION_DEG
                # Normalize to -180..+180 range
                if ang > 180:
                    ang -= 360
                elif ang < -180:
                    ang += 360
                
                # FOV filter — WIDENED TO 90° FOR DEBUG, effectively disabled
                if abs(ang) > FOV_DEG:
                    continue
                dets.append((r, v, pw, ang))

            tracks=tracker.update(dets)
            worst="SAFE"
            for t in tracks:
                if t.level=="IMMINENT": worst="IMMINENT"; break
                if t.level=="CAUTION" and worst!="IMMINENT": worst="CAUTION"

            msg=json.dumps({"t":time.time(),"tracks":[t.to_dict() for t in tracks],"worst":worst})
            sock.sendto(msg.encode(),(JETSON_IP,JETSON_PORT))
            sent+=1
            if sent%50==0:
                print(f"[TX] {sent} frames  dets={len(dets)}  tracks={len(tracks)}  worst={worst}")
    except KeyboardInterrupt:
        print("\n[EXIT]")
    finally:
        radar.stop(); sock.close()

if __name__=="__main__":
    main()