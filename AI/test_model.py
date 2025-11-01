import numpy as np
import sys
from colorama import Fore, init
import os

import test_network

def test_model(model_name):
    init(autoreset=True)

    model_path = os.path.expanduser(f"~/ALSAI/AI/models/{model_name}.keras")

    if not os.path.exists(model_path):
        print(Fore.YELLOW + "There is no trained model for given name, exiting")
        sys.exit(2)
    else:
        print(Fore.GREEN + f"Got model: {model_path}")

    data_file = os.path.expanduser(f"~/ALSAI/data/test_data.npz")
    data_npz = np.load(data_file, allow_pickle=True)
    data = data_npz['arr_0']

    test_network.test_on_npz(data, model_path, model_name)

def main():
    if len(sys.argv) > 1:
        model_name = sys.argv[1]
    else:
        print(Fore.YELLOW + "Didn't get name for model, exiting")
        sys.exit(1)

    test_model(model_name)

if __name__ == "__main__":
    main()
    