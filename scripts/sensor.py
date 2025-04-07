import carla

client = carla.Client('localhost', 2000)
client.set_timeout(5.0)
world = client.get_world()
actors = world.get_actors()

vehicles = actors.filter('vehicle.tesla.cybertruck') # assuming the ego vehicle is a cybertruck
for vehicle in vehicles:
    print(f"Vehicle ID: {vehicle.id}, Type: {vehicle.type_id}")

ego_id = vehicles[0].id
vehicle = world.get_actor(ego_id)

blueprint_library = world.get_blueprint_library()

camera_bp = blueprint_library.find('sensor.camera.rgb')
camera_bp.set_attribute('image_size_x', '1280')
camera_bp.set_attribute('image_size_y', '720')
camera_bp.set_attribute('fov', '150')

camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)

camera.listen(lambda image: image.save_to_disk('output/frame_%06d.png' % image.frame))
