import carla
import random
import time
import queue
import numpy as np
import cv2

OUT_FILE = "/home/rh960/carla_env/carla_recording.mp4"
DURATION_SEC = 20
FPS = 10
WIDTH, HEIGHT = 640, 360

client = carla.Client("localhost", 2000)
client.set_timeout(20.0)
world = client.get_world()
bp_lib = world.get_blueprint_library()

actors = []
q = queue.Queue()

def cam_cb(image: carla.Image):
    q.put(image)

try:
    # Spawn vehicle
    vehicle_bp = bp_lib.filter("vehicle.tesla.model3")[0]
    spawn = random.choice(world.get_map().get_spawn_points())
    vehicle = world.spawn_actor(vehicle_bp, spawn)
    vehicle.set_autopilot(True)
    actors.append(vehicle)

    # Camera
    cam_bp = bp_lib.find("sensor.camera.rgb")
    cam_bp.set_attribute("image_size_x", str(WIDTH))
    cam_bp.set_attribute("image_size_y", str(HEIGHT))
    cam_bp.set_attribute("fov", "90")
    cam_bp.set_attribute("sensor_tick", str(1.0 / FPS))

    cam_tf = carla.Transform(carla.Location(x=1.5, z=2.4))
    cam = world.spawn_actor(cam_bp, cam_tf, attach_to=vehicle)
    actors.append(cam)
    cam.listen(cam_cb)

    # Video writer (mp4)
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(OUT_FILE, fourcc, FPS, (WIDTH, HEIGHT))

    print("Recording video to:", OUT_FILE)
    t_end = time.time() + DURATION_SEC
    frames_written = 0

    while time.time() < t_end:
        try:
            image = q.get(timeout=2.0)
        except queue.Empty:
            continue

        arr = np.frombuffer(image.raw_data, dtype=np.uint8).reshape((HEIGHT, WIDTH, 4))
        bgr = arr[:, :, :3]  # already BGR in CARLA raw_data order (BGRA)
        writer.write(bgr)
        frames_written += 1

    writer.release()
    print(f"Done. Frames written: {frames_written}")

finally:
    for a in actors:
        try:
            a.destroy()
        except Exception:
            pass
