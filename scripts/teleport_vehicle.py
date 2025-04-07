import carla

client = carla.Client('localhost', 2000)
client.set_timeout(5.0)
world = client.get_world()

bp_lib = world.get_blueprint_library()
vehicle_bp = bp_lib.filter("vehicle.audi.tt")[0]
print(vehicle_bp)

spawn_point = carla.Transform(
    carla.Location(x=-34.0, y=7.0, z=0.1),
    # carla.Rotation(yaw=90)
)

vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)
# vehicle.set_autopilot(False)  # Optional
print("Spawned at:", spawn_point)
