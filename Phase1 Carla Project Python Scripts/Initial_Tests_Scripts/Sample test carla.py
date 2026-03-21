import carla
import random
import pygame
import numpy as np

# Connect to CARLA server
client = carla.Client("localhost", 2000)
client.set_timeout(10.0)

world = client.get_world()
blueprint_library = world.get_blueprint_library()

actor_list = []

pygame.init()
display = pygame.display.set_mode((800, 600))
pygame.display.set_caption("CARLA Camera Test")


def process_image(image):
    array = np.frombuffer(image.raw_data, dtype=np.uint8)
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]

    surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    display.blit(surface, (0, 0))
    pygame.display.flip()


try:
    # Spawn vehicle
    vehicle_bp = blueprint_library.filter("vehicle.tesla.model3")[0]
    spawn_point = random.choice(world.get_map().get_spawn_points())

    vehicle = world.spawn_actor(vehicle_bp, spawn_point)
    vehicle.set_autopilot(True)
    actor_list.append(vehicle)

    print("Vehicle spawned")

    # Spawn camera
    camera_bp = blueprint_library.find("sensor.camera.rgb")
    camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))

    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
    actor_list.append(camera)

    camera.listen(lambda image: process_image(image))

    print("Camera streaming... Close window to stop")

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

finally:
    print("Cleaning actors...")

    for actor in actor_list:
        actor.destroy()

    pygame.quit()
    print("Done")
