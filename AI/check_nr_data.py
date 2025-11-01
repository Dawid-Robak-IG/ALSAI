import numpy as np
from colorama import Fore, init
import os

def check_nr_test():
    data_file = os.path.expanduser(f"~/ALSAI/data/test_data.npz")
    data_npz = np.load(data_file, allow_pickle=True)
    data = data_npz['arr_0']
    print(Fore.GREEN + f"Found {len(data)} number of data for testing")

def check_nr_train():
    data_file = os.path.expanduser(f"~/ALSAI/data/train_data.npz")
    data_npz = np.load(data_file, allow_pickle=True)
    data = data_npz['arr_0']
    print(Fore.GREEN + f"Found {len(data)} number of data for trainig")

def main():
    init(autoreset=True)
    check_nr_test()
    check_nr_train()

if __name__ == "__main__":
    main()
    