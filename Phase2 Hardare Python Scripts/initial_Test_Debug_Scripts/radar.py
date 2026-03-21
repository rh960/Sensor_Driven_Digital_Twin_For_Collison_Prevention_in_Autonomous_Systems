"""
DreamHAT+ 60 GHz — Collision Detection System
==============================================
TRUE SINGLE FILE — talks directly to the radar hardware.
No second terminal. No UDP. Just run:

    python dreamhat_collision.py

INSTALL
-------
    pip install numpy scipy pyqtgraph PyQt5 spidev gpiozero
    # optional speedups:
    pip install pyfftw numba

WIRING (DreamHAT+ default GPIO)
--------------------------------
    RST  → GPIO 12
    IRQ  → GPIO 25
    SPI0 → MOSI/MISO/SCLK/CE0  (standard Pi SPI0)

COLLISION LEVELS
----------------
    SAFE      TTC > 3 s  or  not approaching
    CAUTION   TTC 1.5–3 s  OR  range < 1.2 m approaching
    IMMINENT  TTC < 1.5 s  OR  range < 0.5 m approaching
"""

import sys, os, re, json, struct, threading, time, queue, logging
from collections import deque
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np
from scipy.signal import convolve2d
from scipy.ndimage import label as sp_label

logging.basicConfig(level=logging.WARNING,
                    format='%(asctime)s %(levelname)s %(message)s')

# ── optional pyfftw ───────────────────────────────────────────────────────
try:
    import pyfftw.interfaces.numpy_fft as fft_mod
    import pyfftw; pyfftw.interfaces.cache.enable()
    FFT_BACKEND = "pyfftw"
except ImportError:
    import numpy.fft as fft_mod
    FFT_BACKEND = "numpy"

# ── optional numba ────────────────────────────────────────────────────────
try:
    import numba
    @numba.vectorize([numba.float32(numba.complex64)])
    def _abs2(x): return x.real**2 + x.imag**2
    NUMBA_OK = True
except ImportError:
    def _abs2(x): return (x.real**2 + x.imag**2).astype(np.float32)
    NUMBA_OK = False

# ── GUI ───────────────────────────────────────────────────────────────────
try:
    import pyqtgraph as pg
    from pyqtgraph.Qt import QtWidgets, QtCore, QtGui
    GUI_OK = True
except ImportError:
    GUI_OK = False
    print("[WARN] pyqtgraph/PyQt5 missing — headless mode")
    print("       pip install pyqtgraph PyQt5\n")


# ══════════════════════════════════════════════════════════════
#  BGT60TR13C CONSTANTS  (inline from BGT60TR13C_CONST.py)
# ══════════════════════════════════════════════════════════════
BGT60TRXX_SPI_WR_OP_MSK            = 0x01000000
BGT60TRXX_SPI_WR_OP_POS            = 24
BGT60TRXX_SPI_REGADR_MSK           = 0xFE000000
BGT60TRXX_SPI_REGADR_POS           = 25
BGT60TRXX_SPI_DATA_MSK             = 0x00FFFFFF
BGT60TRXX_SPI_DATA_POS             = 0
BGT60TRXX_SPI_BURST_MODE_CMD       = 0xFF000000
BGT60TRXX_SPI_BURST_MODE_SADR_MSK  = 0x00FE0000
BGT60TRXX_SPI_BURST_MODE_SADR_POS  = 17
BGT60TRXX_SPI_BURST_MODE_RWB_MSK   = 0x00010000
BGT60TRXX_SPI_BURST_MODE_RWB_POS   = 16
BGT60TRXX_SPI_BURST_MODE_LEN_MSK   = 0x0000FE00
BGT60TRXX_SPI_BURST_MODE_LEN_POS   = 9

BGT60TRXX_REG_MAIN                 = 0x00
BGT60TRXX_REG_ADC0                 = 0x01
BGT60TRXX_REG_CHIP_ID              = 0x02
BGT60TRXX_REG_SFCTL                = 0x06
BGT60TRXX_REG_FSTAT_TR13C          = 0x5f
BGT60TRXX_REG_FIFO_TR13C           = 0x60

BGT60TRXX_REG_MAIN_FRAME_START_MSK = 0x000001
BGT60TRXX_REG_MAIN_RESET_POS       = 1
BGT60TRXX_REG_CHIP_ID_RF_ID_MSK    = 0x0000ff
BGT60TRXX_REG_CHIP_ID_DIGITAL_ID_MSK = 0xffff00
BGT60TRXX_REG_CHIP_ID_DIGITAL_ID_POS = 8
BGT60TRXX_REG_CHIP_ID_RF_ID_POS    = 0

BGT60TRXX_REG_SFCTL_FIFO_CREF_MSK  = 0x001fff
BGT60TRXX_REG_SFCTL_FIFO_CREF_POS  = 0
BGT60TRXX_REG_SFCTL_MISO_HS_READ_MSK = 0x010000

BGT60TRXX_REG_FSTAT_FILL_STATUS_MSK = 0x003fff
BGT60TRXX_REG_FSTAT_FILL_STATUS_POS = 0
BGT60TRXX_REG_FSTAT_CLK_NUM_ERR_MSK = 0x020000
BGT60TRXX_REG_FSTAT_CLK_NUM_ERR_POS = 17
BGT60TRXX_REG_FSTAT_SPI_BURST_ERR_MSK = 0x040000
BGT60TRXX_REG_FSTAT_SPI_BURST_ERR_POS = 18
BGT60TRXX_REG_FSTAT_FUF_ERR_MSK    = 0x080000
BGT60TRXX_REG_FSTAT_FUF_ERR_POS    = 19
BGT60TRXX_REG_FSTAT_EMPTY_MSK      = 0x100000
BGT60TRXX_REG_FSTAT_EMPTY_POS      = 20
BGT60TRXX_REG_FSTAT_CREF_MSK       = 0x200000
BGT60TRXX_REG_FSTAT_CREF_POS       = 21
BGT60TRXX_REG_FSTAT_FULL_MSK       = 0x400000
BGT60TRXX_REG_FSTAT_FULL_POS       = 22
BGT60TRXX_REG_FSTAT_FOF_ERR_MSK    = 0x800000
BGT60TRXX_REG_FSTAT_FOF_ERR_POS    = 23
BGT60TRXX_REG_FSTAT_TR13C_FIFO_SIZE = 8192

BGT60TRXX_REG_GSR0_FOU_ERR_MSK     = 0x08
BGT60TRXX_REG_GSR0_MISO_HS_READ_MSK = 0x04
BGT60TRXX_REG_GSR0_SPI_BURST_ERR_MSK = 0x02
BGT60TRXX_REG_GSR0_CLK_NUM_ERR_MSK = 0x01

BGT60TRXX_RESET_SW   = 0x1 << BGT60TRXX_REG_MAIN_RESET_POS
BGT60TRXX_RESET_FSM  = 0x2 << BGT60TRXX_REG_MAIN_RESET_POS
BGT60TRXX_RESET_FIFO = 0x4 << BGT60TRXX_REG_MAIN_RESET_POS

RET_VAL_OK  = 0
RET_VAL_ERR = -1


# ══════════════════════════════════════════════════════════════
#  BGT60TR13C DRIVER  (inline from BGT60TR13C.py)
# ══════════════════════════════════════════════════════════════
class BGT60TR13C:
    def __init__(self, spi_bus=0, spi_dev=0, spi_speed=50_000_000,
                 rst_pin=12, irq_pin=25, version=0, save_to_file=None):
        import spidev
        from gpiozero import DigitalInputDevice, DigitalOutputDevice

        self.__fifo_num_samples_per_frame = None
        self.__fifo_num_samples_irq       = None
        self.__fifo_num_sampler_per_burst = None
        self.__register_config_file_name  = None

        self.__spi = spidev.SpiDev()
        self.__spi.open(spi_bus, spi_dev)
        self.__spi.max_speed_hz = spi_speed
        self.__spi.mode = 0

        self.__rst = DigitalOutputDevice(rst_pin)
        self.__irq = DigitalInputDevice(irq_pin, pull_up=False)
        self.hard_reset()

        self.frame_buffer    = queue.Queue(maxsize=256)
        self.__sub_frame_buf = []
        self.__num_samples_per_frame  = 0
        self.__num_bytes_per_frame    = 0
        self.__num_sampler_per_burst  = 0
        self.__last_gsr_reg           = 0
        self.__data_collection_thread = None
        self.__stop_event             = threading.Event()
        self.__version = version
        self.__seq     = 0
        self.__save_to_file = save_to_file
        self.__file_fd      = None

    # ── SPI primitives ────────────────────────────────────────
    def __set_reg(self, reg_addr, data):
        tmp = ((reg_addr << BGT60TRXX_SPI_REGADR_POS) & BGT60TRXX_SPI_REGADR_MSK |
               BGT60TRXX_SPI_WR_OP_MSK |
               (data << BGT60TRXX_SPI_DATA_POS) & BGT60TRXX_SPI_DATA_MSK)
        tx = [tmp>>24&0xFF, tmp>>16&0xFF, tmp>>8&0xFF, tmp&0xFF]
        rx = self.__spi.xfer2(tx)
        self.__last_gsr_reg = rx[0]
        return (rx[0]<<24)|(rx[1]<<16)|(rx[2]<<8)|rx[3]

    def __get_reg(self, reg_addr):
        tmp = (reg_addr << BGT60TRXX_SPI_REGADR_POS) & BGT60TRXX_SPI_REGADR_MSK
        tx  = [tmp>>24&0xFF, tmp>>16&0xFF, tmp>>8&0xFF, tmp&0xFF]
        rx  = self.__spi.xfer2(tx)
        self.__last_gsr_reg = rx[0]
        return (rx[0]<<24)|(rx[1]<<16)|(rx[2]<<8)|rx[3]

    def __get_fifo_data(self, num_samples):
        fifo_data = []
        if 0 < num_samples>>1 <= BGT60TRXX_REG_FSTAT_TR13C_FIFO_SIZE and num_samples%2==0:
            tx = ([BGT60TRXX_SPI_BURST_MODE_CMD>>24&0xFF,
                   BGT60TRXX_REG_FIFO_TR13C<<BGT60TRXX_SPI_BURST_MODE_SADR_POS>>16&0xFF,
                   0x00, 0x00] + [0x00]*((num_samples>>1)*3))
            rx = self.__spi.xfer2(tx)
            self.__last_gsr_reg = rx[0]
            if self.check_gsr_reg() == RET_VAL_OK:
                fifo_data = rx[4:]
            else:
                logging.error("GSR Error in FIFO read")
        return fifo_data

    # ── chip ──────────────────────────────────────────────────
    def check_chip_id(self):
        chip_id = self.__get_reg(BGT60TRXX_REG_CHIP_ID)
        digital = (chip_id & BGT60TRXX_REG_CHIP_ID_DIGITAL_ID_MSK) >> BGT60TRXX_REG_CHIP_ID_DIGITAL_ID_POS
        rf      = (chip_id & BGT60TRXX_REG_CHIP_ID_RF_ID_MSK) >> BGT60TRXX_REG_CHIP_ID_RF_ID_POS
        if digital == 3 and rf == 3:
            logging.info("BGT60TR13C detected.")
            return RET_VAL_OK
        logging.warning(f"Unexpected chip_id: digital={digital} rf={rf}")
        return RET_VAL_ERR

    def set_register_config_file(self, file_name):
        self.__register_config_file_name = file_name

    def __load_register_config(self):
        with open(self.__register_config_file_name) as f:
            for line in f:
                parts = line.strip().split()
                if len(parts) == 3:
                    addr = int(parts[1], 16)
                    val  = int(parts[2], 16)
                    if addr == BGT60TRXX_REG_SFCTL:
                        if self.__spi.max_speed_hz > 20_000_000:
                            val |= BGT60TRXX_REG_SFCTL_MISO_HS_READ_MSK
                        else:
                            val &= ~BGT60TRXX_REG_SFCTL_MISO_HS_READ_MSK
                    self.__set_reg(addr, val)

    def set_fifo_parameters(self, num_samples_per_frame,
                            num_samples_irq, num_sampler_per_burst):
        self.__fifo_num_samples_per_frame = num_samples_per_frame
        self.__fifo_num_samples_irq       = num_samples_irq
        self.__fifo_num_sampler_per_burst = num_sampler_per_burst

    def __apply_fifo_parameters(self):
        self.__num_samples_per_frame = self.__fifo_num_samples_per_frame
        self.__num_bytes_per_frame   = (self.__fifo_num_samples_per_frame>>1)*3
        self.__num_sampler_per_burst = self.__fifo_num_sampler_per_burst
        self.__set_fifo_limit(self.__fifo_num_samples_irq)

    def __set_fifo_limit(self, num_samples):
        if 0 < num_samples>>1 <= BGT60TRXX_REG_FSTAT_TR13C_FIFO_SIZE and num_samples%2==0:
            tmp = self.__get_reg(BGT60TRXX_REG_SFCTL)
            tmp &= ~BGT60TRXX_REG_SFCTL_FIFO_CREF_MSK
            tmp |= (((num_samples>>1)-1) << BGT60TRXX_REG_SFCTL_FIFO_CREF_POS) & BGT60TRXX_REG_SFCTL_FIFO_CREF_MSK
            self.__set_reg(BGT60TRXX_REG_SFCTL, tmp)

    def __check_fifo_status(self):
        s = self.__get_reg(BGT60TRXX_REG_FSTAT_TR13C)
        FOF = (s & BGT60TRXX_REG_FSTAT_FOF_ERR_MSK)    >> BGT60TRXX_REG_FSTAT_FOF_ERR_POS
        FUF = (s & BGT60TRXX_REG_FSTAT_FUF_ERR_MSK)    >> BGT60TRXX_REG_FSTAT_FUF_ERR_POS
        SBE = (s & BGT60TRXX_REG_FSTAT_SPI_BURST_ERR_MSK) >> BGT60TRXX_REG_FSTAT_SPI_BURST_ERR_POS
        CKE = (s & BGT60TRXX_REG_FSTAT_CLK_NUM_ERR_MSK) >> BGT60TRXX_REG_FSTAT_CLK_NUM_ERR_POS
        CRF = (s & BGT60TRXX_REG_FSTAT_CREF_MSK)        >> BGT60TRXX_REG_FSTAT_CREF_POS
        return FOF, CRF, FUF, SBE, CKE

    def check_gsr_reg(self):
        bad = (BGT60TRXX_REG_GSR0_FOU_ERR_MSK |
               BGT60TRXX_REG_GSR0_SPI_BURST_ERR_MSK |
               BGT60TRXX_REG_GSR0_CLK_NUM_ERR_MSK)
        return RET_VAL_ERR if self.__last_gsr_reg & bad else RET_VAL_OK

    # ── reset ─────────────────────────────────────────────────
    def hard_reset(self):
        self.__rst.on();  time.sleep(0.01)
        self.__rst.off(); time.sleep(0.01)
        self.__rst.on();  time.sleep(0.01)

    def soft_reset(self, reset_type):
        self.__sub_frame_buf = []
        tmp = self.__get_reg(BGT60TRXX_REG_MAIN) | reset_type
        self.__set_reg(BGT60TRXX_REG_MAIN, tmp)
        for _ in range(10):
            time.sleep(0.01)
            if not (self.__get_reg(BGT60TRXX_REG_MAIN) & reset_type):
                break
        time.sleep(0.01)

    # ── start / stop ──────────────────────────────────────────
    def start(self):
        if self.__data_collection_thread and self.__data_collection_thread.is_alive():
            self.stop()
        self.__stop_event.clear()
        self.__data_collection_thread = threading.Thread(
            target=self.__data_collection, daemon=True)
        self.__data_collection_thread.start()

    def stop(self):
        self.soft_reset(BGT60TRXX_RESET_SW)
        self.__stop_event.set()
        if self.__data_collection_thread:
            self.__data_collection_thread.join()
        if self.__file_fd:
            self.__file_fd.close()
            self.__file_fd = None

    # ── data collection thread ────────────────────────────────
    def __data_collection(self):
        while not self.__stop_event.is_set():
            self.soft_reset(BGT60TRXX_RESET_SW)
            self.__load_register_config()
            self.__apply_fifo_parameters()

            if self.__save_to_file:
                import datetime
                ts  = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
                fn  = f"{self.__save_to_file}_{ts}.bin"
                os.makedirs(os.path.dirname(fn) or ".", exist_ok=True)
                self.__file_fd = open(fn, "wb", buffering=0)

            self.__seq = 0
            tmp = self.__get_reg(BGT60TRXX_REG_MAIN) | BGT60TRXX_REG_MAIN_FRAME_START_MSK
            self.__set_reg(BGT60TRXX_REG_MAIN, tmp)
            err = False

            while not self.__stop_event.is_set():
                time.sleep(0.001)
                if err: break
                while True:
                    FOF, CRF, FUF, SBE, CKE = self.__check_fifo_status()
                    if FOF or FUF or SBE or CKE:
                        logging.error(f"FIFO error FOF={FOF} FUF={FUF} SBE={SBE} CKE={CKE}")
                        err = True; break
                    if not CRF: break
                    data = self.__get_fifo_data(self.__num_sampler_per_burst)
                    try:
                        self.__sub_frame_buf += data
                        while len(self.__sub_frame_buf) >= self.__num_bytes_per_frame:
                            frame = bytes(self.__sub_frame_buf[:self.__num_bytes_per_frame])
                            self.__sub_frame_buf = self.__sub_frame_buf[self.__num_bytes_per_frame:]
                            # Build framed packet: version(4) seq(4) len(4) data
                            pkt = (self.__version.to_bytes(4,'little') +
                                   self.__seq.to_bytes(4,'little') +
                                   len(frame).to_bytes(4,'little') +
                                   frame)
                            self.__seq += 1
                            if self.__file_fd:
                                self.__file_fd.write(pkt)
                            try:
                                self.frame_buffer.put_nowait(pkt)
                            except queue.Full:
                                logging.warning("Frame buffer full, dropping frame")
                    except Exception as e:
                        logging.error(f"Frame assembly error: {e}")

    def __del__(self):
        try: self.stop()
        except: pass


# ══════════════════════════════════════════════════════════════
#  CONFIG LOADER
# ══════════════════════════════════════════════════════════════
def find_file(directory: str, pattern: str) -> str:
    matches = [f for f in os.listdir(directory) if re.match(pattern, f)]
    if not matches:
        raise FileNotFoundError(f"No file matching '{pattern}' in {directory}")
    if len(matches) > 1:
        matches.sort(reverse=True)   # take most recent timestamp
    return os.path.join(directory, matches[0])


def load_config(cfg_dir: str):
    """Returns (setting_dict, register_file_path, frame_size)."""
    reg_file = find_file(cfg_dir, r"BGT60TR13C_export_registers_\d{8}-\d{6}\.txt")
    set_file = find_file(cfg_dir, r"BGT60TR13C_settings_\d{8}-\d{6}\.json")
    with open(set_file) as f:
        setting = json.load(f)
    frame_seq = setting["sequence"][0]["sequence"]
    total = 0
    for frame in frame_seq:
        if frame["type"] == "loop":
            reps = frame["num_repetitions"]
            samp = 0
            for chirp in frame["sequence"]:
                if chirp["type"] == "chirp":
                    samp += chirp["num_samples"] * bin(chirp["rx_mask"]).count("1")
            total += samp * reps
    return setting, reg_file, total


def parse_radar_cfg(setting: dict) -> dict:
    loop  = setting["sequence"][0]["sequence"][0]
    chirp = loop["sequence"][0]
    return dict(
        frame_rate   = 1.0 / setting["sequence"][0]["repetition_time_s"],
        chirp_rate   = 1.0 / loop["repetition_time_s"],
        num_chirps   = loop["num_repetitions"],
        num_samples  = chirp["num_samples"],
        bandwidth    = chirp["end_frequency_Hz"] - chirp["start_frequency_Hz"],
        start_freq   = chirp["start_frequency_Hz"],
        end_freq     = chirp["end_frequency_Hz"],
        rx_mask      = chirp["rx_mask"],
        num_antennas = bin(chirp["rx_mask"]).count("1"),
    )


# ══════════════════════════════════════════════════════════════
#  RADAR CUBE PROCESSOR
# ══════════════════════════════════════════════════════════════
# BGT60TR13C antenna positions (from mmw_cube_proc_v0.py):
#   RX2(0,1)           TX
#   RX3(0,0)    RX1(1,0)
POSITION_MAP = {1:(1,0), 2:(0,1), 3:(0,0)}
NUM_AZ_BINS  = 2
C_LIGHT      = 3e8


class RadarCube:
    def __init__(self, p: dict, min_range: float = 0.2, mti_alpha: float = 0.6):
        self.p         = p
        self.mti_alpha = mti_alpha
        self._prev     = None

        nd = p["num_chirps"]
        nr = p["num_samples"]
        bw = p["bandwidth"]
        cf = (p["start_freq"] + p["end_freq"]) / 2
        cr = p["chirp_rate"]

        self.lam        = C_LIGHT / cf
        self.range_res  = C_LIGHT / (2 * bw)
        _ra             = np.arange(nr>>1) * self.range_res
        self.rskip      = int(np.searchsorted(_ra, min_range))
        self.range_axis = _ra[self.rskip:]

        self.doppler_axis = (-np.fft.fftshift(np.fft.fftfreq(nd, 1.0/cr))
                             / 2 * C_LIGHT / cf)

        self.active_ant = [a for a in POSITION_MAP
                           if p["rx_mask"] & (1 << (a-1))]
        self.nd = nd; self.nr = nr

    def _read_uint12(self, data: bytes) -> np.ndarray:
        d   = np.frombuffer(data, dtype=np.uint8)
        d   = d.reshape(len(d)//3, 3).astype(np.uint16)
        fst = (d[:,0] << 4) | (d[:,1] >> 4)
        snd = ((d[:,1] & 0x0F) << 8) | d[:,2]
        return np.stack([fst,snd],axis=1).reshape(-1).astype(np.float32)

    def process(self, payload: bytes) -> Optional[np.ndarray]:
        adc = self._read_uint12(payload)
        adc = adc.reshape(self.nd, self.nr, self.p["num_antennas"])

        cube = np.zeros((self.nd, self.nr, NUM_AZ_BINS, NUM_AZ_BINS),
                        dtype=np.float32)
        for i, ant in enumerate(self.active_ant):
            r, c = POSITION_MAP[ant]
            if r < NUM_AZ_BINS and c < NUM_AZ_BINS:
                cube[:, :, r, c] = adc[:, :, i]

        # MTI clutter removal
        if self._prev is not None:
            mti = cube - self._prev
            self._prev = self.mti_alpha*cube + (1-self.mti_alpha)*self._prev
            cube = mti
        else:
            self._prev = cube.copy()
            return None   # first frame: MTI reference only

        cf  = fft_mod.fftn(cube.astype(np.complex64), axes=(0,1,2,3))
        cf  = np.fft.fftshift(cf, axes=(0,2,3))
        pw  = _abs2(cf.astype(np.complex64))
        pw  = pw[:, self.rskip:self.nr>>1, :, :]
        rd  = pw.mean(axis=(2,3))     # (Doppler, Range)
        return rd.T                   # → (Range, Doppler)


# ══════════════════════════════════════════════════════════════
#  DATA CLASSES
# ══════════════════════════════════════════════════════════════
TTC_IMMINENT_S   = 1.5
TTC_CAUTION_S    = 3.0
RANGE_IMMINENT_M = 0.50
RANGE_CAUTION_M  = 1.20
STATIC_VEL       = 0.06
TRAIL_LEN        = 50
CFAR_THRESH_DB   = 10.0
CFAR_GUARD_R=2; CFAR_TRAIN_R=6; CFAR_GUARD_D=1; CFAR_TRAIN_D=4
MAX_MISSED=6; GATE_R=0.12; GATE_V=0.40
GUI_FPS=10


@dataclass
class Detection:
    range_m: float; vel_mps: float; pwr_db: float; ri: int; di: int

@dataclass
class Track:
    tid:     int
    range_m: float; vel_mps: float; pwr_db: float
    age:     int   = 0
    missed:  int   = 0
    cls:     str   = "UNKNOWN"
    level:   str   = "SAFE"
    ttc_s:   float = float("inf")
    history: deque = field(default_factory=lambda: deque(maxlen=TRAIL_LEN))

    def update(self, d: Detection):
        a=0.35
        self.range_m = a*d.range_m + (1-a)*self.range_m
        self.vel_mps = a*d.vel_mps + (1-a)*self.vel_mps
        self.pwr_db  = a*d.pwr_db  + (1-a)*self.pwr_db
        self.missed=0; self.age+=1
        self.history.append((self.range_m, self.vel_mps))
        self._derive()

    def _derive(self):
        v=self.vel_mps
        if   abs(v)<STATIC_VEL: self.cls="STATIC"
        elif v<0:                self.cls="APPROACHING"
        else:                    self.cls="RECEDING"
        self.ttc_s = (self.range_m/abs(v)
                      if self.cls=="APPROACHING" and self.range_m>0
                      else float("inf"))
        if self.cls=="APPROACHING":
            if self.range_m<=RANGE_IMMINENT_M or self.ttc_s<=TTC_IMMINENT_S:
                self.level="IMMINENT"
            elif self.range_m<=RANGE_CAUTION_M or self.ttc_s<=TTC_CAUTION_S:
                self.level="CAUTION"
            else: self.level="SAFE"
        else: self.level="SAFE"

    def ttc_str(self):
        return "∞" if self.ttc_s==float("inf") else f"{self.ttc_s:.1f}s"


# ══════════════════════════════════════════════════════════════
#  CFAR + TRACKER
# ══════════════════════════════════════════════════════════════
def cfar2d(rd: np.ndarray) -> List[Tuple[int,int,float]]:
    mag=10*np.log10(rd+1e-12)
    gr,gd=CFAR_GUARD_R,CFAR_GUARD_D; tr,td=CFAR_TRAIN_R,CFAR_TRAIN_D
    ntr=(2*(gr+tr)+1)*(2*(gd+td)+1)-(2*gr+1)*(2*gd+1)
    k=np.ones((2*(gr+tr)+1,2*(gd+td)+1)); k[tr:tr+2*gr+1,td:td+2*gd+1]=0; k/=ntr
    noise=convolve2d(mag,k,mode="same",boundary="wrap")
    mask=mag>(noise+CFAR_THRESH_DB)
    lbl,n=sp_label(mask,np.ones((3,3)))
    out=[]
    for i in range(1,n+1):
        idx=np.argwhere(lbl==i); pw=mag[idx[:,0],idx[:,1]]; j=np.argmax(pw)
        out.append((int(idx[j,0]),int(idx[j,1]),float(pw[j])))
    return out

class Tracker:
    def __init__(self): self._t:List[Track]=[]; self._nid=1
    def update(self,dets:List[Detection])->List[Track]:
        um=list(dets)
        for t in self._t:
            best=None; bc=1e9
            for d in um:
                if abs(d.range_m-t.range_m)<GATE_R and abs(d.vel_mps-t.vel_mps)<GATE_V:
                    c=abs(d.range_m-t.range_m)/GATE_R+abs(d.vel_mps-t.vel_mps)/GATE_V
                    if c<bc: bc,best=c,d
            if best: t.update(best); um.remove(best)
            else: t.missed+=1
        self._t=[t for t in self._t if t.missed<MAX_MISSED]
        for d in um:
            t=Track(self._nid,d.range_m,d.vel_mps,d.pwr_db)
            t.history.append((d.range_m,d.vel_mps)); t._derive()
            self._t.append(t); self._nid+=1
        return [t for t in self._t if t.age>=2]


# ══════════════════════════════════════════════════════════════
#  PROCESSOR  (cube → tracks)
# ══════════════════════════════════════════════════════════════
def parse_pkt(raw: bytes) -> Optional[bytes]:
    """Strip 12-byte header, return payload or None."""
    if len(raw)<12: return None
    version=struct.unpack_from("<I",raw,0)[0]
    dlen   =struct.unpack_from("<I",raw,8)[0]
    if version!=0 or len(raw)-12!=dlen: return None
    return raw[12:]

class Processor:
    def __init__(self, cube: RadarCube):
        self.cube=cube; self.tracker=Tracker()
    def process(self, raw: bytes):
        payload=parse_pkt(raw)
        if payload is None: return None,[],"SAFE"
        rd=self.cube.process(payload)
        if rd is None: return None,[],"SAFE"
        nr,nd=rd.shape
        dets=[]
        for ri,di,pw in cfar2d(rd):
            if ri>=len(self.cube.range_axis): continue
            r=float(self.cube.range_axis[ri])
            v=float(self.cube.doppler_axis[di]) if di<len(self.cube.doppler_axis) else 0.0
            dets.append(Detection(r,v,pw,ri,di))
        tracks=self.tracker.update(dets)
        worst="SAFE"
        for t in tracks:
            if t.level=="IMMINENT": worst="IMMINENT"; break
            if t.level=="CAUTION" and worst!="IMMINENT": worst="CAUTION"
        return rd,tracks,worst


# ══════════════════════════════════════════════════════════════
#  CONSOLE ALERTS
# ══════════════════════════════════════════════════════════════
_alerted:dict={}
def con_alert(tracks):
    now=time.time()
    for t in tracks:
        if t.level!="SAFE" and now-_alerted.get(t.tid,0)>0.8:
            _alerted[t.tid]=now
            arr="←" if t.vel_mps<0 else "→"
            print(f"[{t.level:8s}] #{t.tid:02d}  r={t.range_m:.2f}m  "
                  f"v={t.vel_mps:+.2f}m/s{arr}  TTC={t.ttc_str()}  {t.cls}")


# ══════════════════════════════════════════════════════════════
#  GUI
# ══════════════════════════════════════════════════════════════
BG="   #050e05"; GRN="#00ff41"; DIMGRN="#004d14"
AMBER="#ffb300"; RED="#ff2020"; WHITE="#e8f5e9"
BG=BG.strip()

LEVEL_STYLE={
    "SAFE":    f"background:#003300;color:{GRN};border:2px solid {GRN};",
    "CAUTION": f"background:#332200;color:{AMBER};border:2px solid {AMBER};",
    "IMMINENT":f"background:#330000;color:{RED};border:2px solid {RED};",
}
LEVEL_PEN={"SAFE":(0,255,65),"CAUTION":(255,179,0),"IMMINENT":(255,32,32)}
TCOLORS=[(0,255,65),(0,229,255),(255,179,0),(255,80,80),
         (180,80,255),(0,200,150),(255,140,0),(120,220,255)]


class CollisionGUI:
    def __init__(self, radar: BGT60TR13C, proc: Processor):
        self.radar=radar; self.proc=proc
        self._fc=0; self._fps=0.0; self._ft=time.time()
        self._trails:dict={}; self._labels:dict={}

        pg.setConfigOption("background",BG); pg.setConfigOption("foreground",GRN)
        self.app=QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)

        self.win=QtWidgets.QMainWindow()
        self.win.setWindowTitle("DreamHAT+  ·  60 GHz Collision Detection")
        self.win.resize(1380,820)
        self.win.setStyleSheet(f"background:{BG};color:{GRN};")
        cw=QtWidgets.QWidget(); self.win.setCentralWidget(cw)
        vbox=QtWidgets.QVBoxLayout(cw)
        vbox.setContentsMargins(8,6,8,6); vbox.setSpacing(5)

        # Banner
        self.banner=QtWidgets.QLabel("◈  SAFE  ◈")
        self.banner.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.banner.setFixedHeight(60)
        self.banner.setFont(QtGui.QFont("Courier New",24,QtGui.QFont.Weight.Bold))
        self._set_banner("SAFE"); vbox.addWidget(self.banner)

        # Plot row
        pr=QtWidgets.QHBoxLayout(); pr.setSpacing(6); vbox.addLayout(pr,stretch=10)

        # Range-Doppler heatmap
        glw_rd=pg.GraphicsLayoutWidget(); glw_rd.setMinimumWidth(480)
        self.rdp=glw_rd.addPlot(title="RANGE–DOPPLER")
        self.rdp.titleLabel.setAttr("color",GRN)
        self.rdp.setLabel("left","Range (m)"); self.rdp.setLabel("bottom","Velocity m/s  [neg=approach]")
        self.rdi=pg.ImageItem(); self.rdp.addItem(self.rdi)
        pg.ColorBarItem(values=(0,60),colorMap=pg.colormap.get("inferno"),
                        label="dB",width=14).setImageItem(self.rdi,insert_in=self.rdp)
        self.rd_sc=pg.ScatterPlotItem(size=14,pen=pg.mkPen(GRN,width=1))
        self.rdp.addItem(self.rd_sc)
        # Physical ticks
        ra=proc.cube.range_axis; da=proc.cube.doppler_axis
        nr=len(ra); nd=len(da); sr=max(1,nr//6); sd=max(1,nd//6)
        self.rdp.getAxis("left").setTicks(
            [[(i,f"{ra[i]:.2f}") for i in range(0,nr,sr) if i<nr]])
        self.rdp.getAxis("bottom").setTicks(
            [[(i,f"{da[i]:.2f}") for i in range(0,nd,sd) if i<nd]])
        pr.addWidget(glw_rd,stretch=5)

        # Top-down view
        glw_pd=pg.GraphicsLayoutWidget(); glw_pd.setMinimumWidth(380)
        self.pp=glw_pd.addPlot(title="TOP-DOWN")
        self.pp.titleLabel.setAttr("color",GRN)
        max_r=float(ra[-1]) if len(ra) else 5.0
        self.pp.setXRange(-max_r,max_r); self.pp.setYRange(0,max_r)
        self.pp.setLabel("left","Range (m)"); self.pp.setLabel("bottom","Range (m)")
        for r in np.arange(0.5,max_r+0.5,0.5):
            e=QtWidgets.QGraphicsEllipseItem(-r,0,2*r,2*r)
            e.setPen(pg.mkPen(DIMGRN,width=0.7)); e.setBrush(pg.mkBrush(None))
            self.pp.addItem(e)
        for r,col in [(RANGE_CAUTION_M,AMBER),(RANGE_IMMINENT_M,RED)]:
            e=QtWidgets.QGraphicsEllipseItem(-r,0,2*r,2*r)
            e.setPen(pg.mkPen(col,width=1.5,style=QtCore.Qt.PenStyle.DashLine))
            e.setBrush(pg.mkBrush(None)); self.pp.addItem(e)
        self.pp.addItem(pg.ScatterPlotItem([{
            "pos":(0,0),"brush":pg.mkBrush(0,229,255,220),
            "pen":pg.mkPen(WHITE,width=1),"size":18,"symbol":"t"}]))
        self.tsc=pg.ScatterPlotItem(size=16); self.pp.addItem(self.tsc)
        pr.addWidget(glw_pd,stretch=4)

        # Right: TTC bars + table
        rp=QtWidgets.QVBoxLayout(); rp.setSpacing(4); pr.addLayout(rp,stretch=3)
        lbl=QtWidgets.QLabel("TIME-TO-COLLISION")
        lbl.setFont(QtGui.QFont("Courier New",9,QtGui.QFont.Weight.Bold))
        lbl.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter); rp.addWidget(lbl)
        glw_ttc=pg.GraphicsLayoutWidget(); glw_ttc.setMaximumWidth(340)
        self.ttcp=glw_ttc.addPlot()
        self.ttcp.hideAxis("left"); self.ttcp.setLabel("bottom","TTC (s)")
        self.ttcp.setXRange(0,TTC_CAUTION_S+0.5); self.ttcp.setYRange(0,10)
        for x,col in [(TTC_IMMINENT_S,RED),(TTC_CAUTION_S,AMBER)]:
            self.ttcp.addLine(x=x,pen=pg.mkPen(col,width=1.5,
                style=QtCore.Qt.PenStyle.DashLine))
        self.ttc_bars:dict={}; self.ttc_txts:dict={}
        rp.addWidget(glw_ttc,stretch=6)
        self.table=QtWidgets.QTableWidget(0,5)
        self.table.setHorizontalHeaderLabels(["#","Range","Vel","TTC","Status"])
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.verticalHeader().setVisible(False)
        self.table.setStyleSheet(
            f"background:{BG};color:{GRN};gridline-color:{DIMGRN};"
            f"QHeaderView::section{{background:#0a1f0a;color:{GRN};"
            f"font-family:Courier New;font-size:9pt;}}"
            f"font-family:Courier New;font-size:9pt;")
        self.table.setMinimumHeight(160); rp.addWidget(self.table,stretch=4)

        self.sbar=QtWidgets.QLabel("")
        self.sbar.setFont(QtGui.QFont("Courier New",8))
        self.sbar.setStyleSheet(f"color:{DIMGRN};"); vbox.addWidget(self.sbar)

        self.timer=QtCore.QTimer()
        self.timer.timeout.connect(self._update)
        self.timer.start(1000//GUI_FPS)
        self.win.show()

    def _set_banner(self,level):
        icons={"SAFE":"◈  SAFE  ◈","CAUTION":"⚠  CAUTION  ⚠","IMMINENT":"⛔  IMMINENT  ⛔"}
        self.banner.setText(icons.get(level,level))
        self.banner.setStyleSheet(f"padding:4px;border-radius:4px;{LEVEL_STYLE[level]}")

    def _update(self):
        try:
            raw=self.radar.frame_buffer.get_nowait()
        except queue.Empty:
            return
        # drain to latest
        while True:
            try: raw=self.radar.frame_buffer.get_nowait()
            except queue.Empty: break

        try: rd,tracks,worst=self.proc.process(raw)
        except Exception as e: print(f"[PROC] {e}"); return
        if rd is None: return

        con_alert(tracks)
        self._fc+=1; now=time.time()
        if now-self._ft>=1.0:
            self._fps=self._fc/(now-self._ft); self._fc=0; self._ft=now

        self._set_banner(worst)

        # RD heatmap
        self.rdi.setImage(10*np.log10(rd+1e-12),levels=(0,60))
        self.rd_sc.setData([{"pos":(t.di,t.ri),
            "brush":pg.mkBrush(*LEVEL_PEN[t.level],200),
            "pen":pg.mkPen(GRN,width=1)} for t in tracks])

        # Top-down
        active={t.tid for t in tracks}
        for tid in list(self._trails):
            if tid not in active: self.pp.removeItem(self._trails.pop(tid))
        for tid in list(self._labels):
            if tid not in active: self.pp.removeItem(self._labels.pop(tid))
        spots=[]
        max_v=abs(self.proc.cube.doppler_axis).max() or 1.0
        for t in tracks:
            col=TCOLORS[t.tid%len(TCOLORS)]; lp=LEVEL_PEN[t.level]
            sz=22 if t.level=="IMMINENT" else (16 if t.level=="CAUTION" else 12)
            x=np.clip(t.vel_mps*0.5,-max_v,max_v); y=t.range_m
            spots.append({"pos":(x,y),"brush":pg.mkBrush(*lp,210),
                           "pen":pg.mkPen(*col,width=2),"size":sz})
            if len(t.history)>=2:
                hx=[np.clip(h[1]*0.5,-max_v,max_v) for h in t.history]
                hy=[h[0] for h in t.history]
                if t.tid not in self._trails:
                    self._trails[t.tid]=self.pp.plot(hx,hy,pen=pg.mkPen(*col,width=1,alpha=90))
                else: self._trails[t.tid].setData(hx,hy)
            txt=f"#{t.tid}\n{t.range_m:.2f}m\n{t.vel_mps:+.2f}m/s\nTTC:{t.ttc_str()}"
            if t.tid not in self._labels:
                lb=pg.TextItem(txt,anchor=(0,1),color=col)
                lb.setFont(QtGui.QFont("Courier New",7))
                self.pp.addItem(lb); self._labels[t.tid]=lb
            self._labels[t.tid].setText(txt); self._labels[t.tid].setPos(x,y)
        self.tsc.setData(spots)

        # TTC bars
        for tid in list(self.ttc_bars):
            if tid not in active: self.ttcp.removeItem(self.ttc_bars.pop(tid))
        for tid in list(self.ttc_txts):
            if tid not in active: self.ttcp.removeItem(self.ttc_txts.pop(tid))
        self.ttcp.setYRange(0,max(len(tracks)*1.5+1,4))
        for i,t in enumerate(sorted(tracks,key=lambda x:x.range_m)):
            y0=i*1.4+0.2
            ttc=min(t.ttc_s,TTC_CAUTION_S+0.4) if t.ttc_s!=float("inf") else TTC_CAUTION_S+0.4
            col=RED if t.level=="IMMINENT" else (AMBER if t.level=="CAUTION" else GRN)
            bar=pg.BarGraphItem(x0=0,x1=ttc,y0=y0,y1=y0+1.0,brush=pg.mkBrush(col+"aa"))
            if t.tid in self.ttc_bars: self.ttcp.removeItem(self.ttc_bars[t.tid])
            self.ttcp.addItem(bar); self.ttc_bars[t.tid]=bar
            ls=f"#{t.tid} {t.ttc_str()}"
            if t.tid not in self.ttc_txts:
                tx=pg.TextItem(ls,anchor=(0,0.5),color=(220,220,220))
                tx.setFont(QtGui.QFont("Courier New",8))
                self.ttcp.addItem(tx); self.ttc_txts[t.tid]=tx
            self.ttc_txts[t.tid].setText(ls); self.ttc_txts[t.tid].setPos(0.05,y0+0.5)

        # Table
        self.table.setRowCount(len(tracks))
        for i,t in enumerate(sorted(tracks,key=lambda x:x.range_m)):
            lc=RED if t.level=="IMMINENT" else (AMBER if t.level=="CAUTION" else GRN)
            arr="←" if t.vel_mps<0 else ("→" if t.vel_mps>0 else "·")
            def cell(s,c=GRN):
                it=QtWidgets.QTableWidgetItem(s)
                it.setForeground(QtGui.QColor(c))
                it.setTextAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
                return it
            self.table.setItem(i,0,cell(f"#{t.tid:02d}"))
            self.table.setItem(i,1,cell(f"{t.range_m:.2f} m"))
            self.table.setItem(i,2,cell(f"{t.vel_mps:+.2f} m/s {arr}"))
            self.table.setItem(i,3,cell(t.ttc_str()))
            self.table.setItem(i,4,cell(t.level,lc))
        self.table.resizeColumnsToContents()

        p=self.proc.cube.p
        self.sbar.setText(
            f"  FPS:{self._fps:.1f}  Tracks:{len(tracks)}  "
            f"FFT:{FFT_BACKEND}  Numba:{NUMBA_OK}  "
            f"{p['num_chirps']}chirps×{p['num_samples']}samp×{p['num_antennas']}RX  "
            f"BW:{p['bandwidth']/1e9:.1f}GHz  "
            f"RRes:{C_LIGHT/(2*p['bandwidth'])*100:.0f}cm  "
            f"CAUTION<{TTC_CAUTION_S}s  IMMINENT<{TTC_IMMINENT_S}s")

    def run(self): self.app.exec()


# ══════════════════════════════════════════════════════════════
#  HEADLESS
# ══════════════════════════════════════════════════════════════
def headless(radar: BGT60TR13C, proc: Processor):
    print("\nDreamHAT+ Collision Detector  [headless]\n")
    while True:
        try: raw=radar.frame_buffer.get(timeout=2)
        except queue.Empty: print("[WAITING]"); continue
        try: rd,tracks,worst=proc.process(raw)
        except Exception as e: print(f"[ERR] {e}"); continue
        if rd is None: continue
        con_alert(tracks)
        row=(f"[{worst:8s}]  "+
             "  ".join(f"#{t.tid}:{t.range_m:.2f}m {t.vel_mps:+.2f}m/s TTC={t.ttc_str()}"
                       for t in tracks))
        print(row if tracks else f"[{worst}]  [NO OBJECTS]",flush=True)


# ══════════════════════════════════════════════════════════════
#  MAIN
# ══════════════════════════════════════════════════════════════
def main():
    cfg_dir = sys.argv[1] if len(sys.argv) > 1 else None

    # Auto-discover config dir if not given
    if cfg_dir is None:
        here = os.path.dirname(os.path.abspath(__file__))
        candidates = [
            os.path.join(here, "..", "radar_config", "config_3rx_5m"),
            os.path.join(here, "radar_config", "config_3rx_5m"),
            os.path.join(here, "config_3rx_5m"),
        ]
        for c in candidates:
            if os.path.isdir(c):
                cfg_dir = c; break
        if cfg_dir is None:
            # Last resort: find any dir containing a settings JSON
            for root,dirs,files in os.walk(os.path.expanduser("~")):
                for f in files:
                    if re.match(r"BGT60TR13C_settings_.*\.json", f):
                        cfg_dir = root; break
                if cfg_dir: break
        if cfg_dir is None:
            print("[ERROR] Cannot find radar config directory.")
            print("  Usage: python dreamhat_collision.py /path/to/config_3rx_5m")
            sys.exit(1)

    cfg_dir = os.path.abspath(cfg_dir)
    print("=" * 60)
    print("  DreamHAT+  60 GHz Collision Detection System")
    print(f"  Config : {cfg_dir}")
    print(f"  FFT    : {FFT_BACKEND}  |  Numba: {NUMBA_OK}")
    print("=" * 60)

    try:
        setting, reg_file, frame_size = load_config(cfg_dir)
    except Exception as e:
        print(f"[ERROR] Loading config: {e}"); sys.exit(1)

    p = parse_radar_cfg(setting)
    print(f"\n  Chirps/frame : {p['num_chirps']}")
    print(f"  Samples/chirp: {p['num_samples']}")
    print(f"  RX antennas  : {p['num_antennas']}  (mask={p['rx_mask']:#x})")
    print(f"  Bandwidth    : {p['bandwidth']/1e9:.2f} GHz")
    print(f"  Frame rate   : {p['frame_rate']:.1f} Hz")
    print(f"  Frame size   : {frame_size} samples  ({frame_size*3//2} bytes)\n")

    # Init radar
    print("  Initialising radar hardware...")
    try:
        radar = BGT60TR13C(spi_speed=50_000_000)
        radar.check_chip_id()
        radar.set_register_config_file(reg_file)
        radar.set_fifo_parameters(frame_size, 4096, 2048)
        radar.start()
        print("  Radar started.\n")
    except Exception as e:
        print(f"[ERROR] Radar init failed: {e}")
        print("  → Is the DreamHAT+ connected?")
        print("  → Is SPI enabled? (sudo raspi-config → Interfaces → SPI)")
        sys.exit(1)

    cube = RadarCube(p)
    proc = Processor(cube)

    try:
        if GUI_OK: CollisionGUI(radar, proc).run()
        else:      headless(radar, proc)
    except KeyboardInterrupt:
        print("\n[EXIT]")
    finally:
        print("Stopping radar...")
        radar.stop()

if __name__ == "__main__":
    main()