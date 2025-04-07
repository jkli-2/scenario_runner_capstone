import carla
import argparse

def main():
    parser = argparse.ArgumentParser(description='Load a specific town in Carla.')
    parser.add_argument('town_number', type=int, help='Town number to load (e.g., 5 for Town05)')
    args = parser.parse_args()

    # Format the town name with leading zero if needed
    town_id = f'Town{args.town_number:02d}'

    print(f'Connecting to CARLA and loading {town_id}...')
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    client.load_world(town_id)
    print(f'Successfully loaded {town_id}')

if __name__ == '__main__':
    main()
