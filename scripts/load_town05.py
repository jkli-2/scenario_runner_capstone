import carla
client = carla.Client('localhost', 2000)
client.load_world('Town05')  # or Town03, etc.
