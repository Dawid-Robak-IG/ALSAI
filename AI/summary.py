import os
os.environ["TF_CPP_MIN_LOG_LEVEL"] = "3"
os.environ["TF_ENABLE_ONEDNN_OPTS"] = "0" 


import sys
from colorama import Fore, init
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Conv1D, Flatten, Dense, Dropout, Input, GlobalAveragePooling1D, LeakyReLU, BatchNormalization
from tensorflow.keras import regularizers


import build_model

def model_sum(model_name):
    init(autoreset=True)

    print(Fore.GREEN + f"Summary for: {model_name}")
    model = build_model.build_model(model_name)

    model.build((None,360,2))
    model.summary()

def main():
    if len(sys.argv) > 1:
        model_name = sys.argv[1]
    else:
        print(Fore.YELLOW + "Didn't get name for model")
        sys.exit(1)
    model_sum(model_name)

if __name__ == "__main__":
    main()