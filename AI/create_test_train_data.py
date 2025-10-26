import sys, os
from colorama import Fore, init
import numpy as np

def create_train_data():
    init(autoreset=True)

    data_dir = os.path.expanduser(f"~/ALSAI/data/single_map_data")

    all_arrays = []

    for file_name in os.listdir(data_dir):
        if file_name not in ["map5_run1.npz", "map6_run1.npz"] and file_name.endswith(".npz"):
            full_path = os.path.join(data_dir,file_name)
            with np.load(full_path, allow_pickle=True) as data:
                print(Fore.GREEN + f"Getting data from {full_path}")
                array = list(data.values())[0]
                all_arrays.append(array)

    whole_data = np.concatenate(all_arrays, axis=0)

    np.random.shuffle(whole_data)

    output_file = os.path.expanduser(f"~/ALSAI/data/train_data")
    np.savez_compressed(output_file, whole_data, dtype=object)
    print(Fore.GREEN + f"Saved {len(whole_data)} pairs to {output_file}")
    init(autoreset=True)


def create_test_data():
    data_dir = os.path.expanduser(f"~/ALSAI/data/single_map_data")

    all_arrays = []

    for file_name in os.listdir(data_dir):
        if file_name in ["map5_run1.npz", "map6_run1.npz"] and file_name.endswith(".npz"):
            full_path = os.path.join(data_dir,file_name)
            with np.load(full_path, allow_pickle=True) as data:
                print(Fore.GREEN + f"Getting data from {full_path}")
                array = list(data.values())[0]
                all_arrays.append(array)

    whole_data = np.concatenate(all_arrays, axis=0)

    np.random.shuffle(whole_data)

    output_file = os.path.expanduser(f"~/ALSAI/data/test_data")
    np.savez_compressed(output_file, whole_data, dtype=object)
    print(Fore.GREEN + f"Saved {len(whole_data)} pairs to {output_file}")
    init(autoreset=True)



def main():
    create_train_data()
    create_test_data()


if __name__ == "__main__":
    main()