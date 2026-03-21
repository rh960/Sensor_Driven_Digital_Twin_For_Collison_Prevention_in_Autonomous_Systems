import carla
import random
import pygame
import numpy as np
import time

# -----------------------------
# CONFIG (lower = less lag)
# -----------------------------
HOST = "localhost"   # Use "localhost" if using PuTTY tunnels (L2000, L2001)
PORT = 2000
TIMEOUT = 60.0

WIDTH, HEIGHT = 640, 360   # reduce resolution
FOV = 90
SENSOR_TICK = 0.05          # 10 FPS (0.05 = 20 FPS)

# -----------------------------
# CONNECT
# -----------------------------
client = carla.Client(HOST, PORT)
client.set_timeout(TIMEOUT)

print("Connecting to CARLA...")
world = client.get_world()
print("✅ Connected successfully!")
print("Current map:", world.get_map().name)

bp_lib = world.get_blueprint_library()
actors = []

# -----------------------------
# PYGAME SETUP
# -----------------------------
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("CARLA Live Camera (Smooth)")
clock = pygame.time.Clock()

# Store only the latest frame (drops older frames => less lag)
latest_rgb = {"arr": None, "ts": 0.0}

def camera_callback(image: carla.Image):
    # Convert CARLA BGRA bytes -> RGB numpy array
    arr = np.frombuffer(image.raw_data, dtype=np.uint8)
    arr = arr.reshape((image.height, image.width, 4))
    rgb = arr[:, :, :3][:, :, ::-1]  # BGRA -> RGB
    latest_rgb["arr"] = rgb
    latest_rgb["ts"] = time.time()

try:
    # -----------------------------
    # SPAWN VEHICLE
    # -----------------------------
    vehicle_bp = bp_lib.filter("vehicle.tesla.model3")[0]
    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        raise RuntimeError("No spawn points found on this map.")

    vehicle = world.spawn_actor(vehicle_bp, random.choice(spawn_points))
    vehicle.set_autopilot(True)
    actors.append(vehicle)
    print("🚗 Vehicle spawned")

    # -----------------------------
    # SPAWN CAMERA
    # -----------------------------
    cam_bp = bp_lib.find("sensor.camera.rgb")
    cam_bp.set_attribute("image_size_x", str(WIDTH))
    cam_bp.set_attribute("image_size_y", str(HEIGHT))
    cam_bp.set_attribute("fov", str(FOV))
    cam_bp.set_attribute("sensor_tick", str(SENSOR_TICK))  # lower FPS

    cam_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
    camera = world.spawn_actor(cam_bp, cam_transform, attach_to=vehicle)
    actors.append(camera)

    camera.listen(camera_callback)
    print("📷 Camera streaming (smooth mode). Close window to stop.")

    # -----------------------------
    # MAIN LOOP
    # -----------------------------
    running = True
    while running:
        clock.tick(60)  # viewer refresh rate (doesn't force camera FPS)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        rgb = latest_rgb["arr"]
        if rgb is not None:
            # pygame expects (width, height, 3)
            surf = pygame.surfarray.make_surface(rgb.swapaxes(0, 1))
            screen.blit(surf, (0, 0))

        # Small status overlay (optional)
        pygame.display.flip()

finally:
    print("🧹 Cleaning up actors...")
    for a in actors:
        try:
            a.destroy()
        except Exception:
            pass
    pygame.quit()
    print("Done.")
