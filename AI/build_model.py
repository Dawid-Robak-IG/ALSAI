import os
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Conv1D, MaxPooling1D, Flatten, Dense, Dropout, Input, GlobalAveragePooling1D, LeakyReLU, BatchNormalization
from tensorflow.keras import regularizers
from colorama import Fore, init
scan_length = 360

def build_model(model_name):
    init(autoreset=True)
    last_underscore_index = model_name.rfind('_')
    chosen_activation = model_name[last_underscore_index + 1:]
    print(Fore.GREEN + f"Got activation layer: {chosen_activation}")
    model_name = model_name[:last_underscore_index]

    if model_name == "model_conv1":
        model = Sequential([
            Input(shape=(scan_length, 2)), 
            Conv1D(64, kernel_size=5, activation=chosen_activation),
            MaxPooling1D(pool_size=4),
            Flatten(),
            Dense(3, activation='linear')
        ])
    elif model_name == "model_conv2":
        model = Sequential([
            Input(shape=(scan_length, 2)), 
            Conv1D(64, kernel_size=5, activation=chosen_activation),
            MaxPooling1D(pool_size=4),
            Conv1D(32, kernel_size=5, activation=chosen_activation),
            MaxPooling1D(pool_size=4),
            Flatten(),
            Dense(3, activation='linear')
        ])
    elif model_name == "model_conv3":
        model = Sequential([
            Input(shape=(scan_length, 2)), 
            Conv1D(64, kernel_size=5, activation=chosen_activation),
            MaxPooling1D(pool_size=4),
            Conv1D(32, kernel_size=5, activation=chosen_activation),
            MaxPooling1D(pool_size=4),
            Conv1D(16, kernel_size=5, activation=chosen_activation),
            MaxPooling1D(pool_size=4),
            Flatten(),
            Dense(3, activation='linear')
        ])
    elif model_name == "model_conv1_dropout":
        model = Sequential([
            Input(shape=(scan_length, 2)), 
            Conv1D(64, kernel_size=5, activation=chosen_activation),
            MaxPooling1D(pool_size=4),
            Flatten(),
            Dropout(0.2),
            Dense(3, activation='linear')
        ])
    elif model_name == "model_conv2_dropout":
        model = Sequential([
            Input(shape=(scan_length, 2)), 
            Conv1D(64, kernel_size=5, activation=chosen_activation),
            MaxPooling1D(pool_size=4),
            Conv1D(32, kernel_size=5, activation=chosen_activation),
            MaxPooling1D(pool_size=4),
            Flatten(),
            Dropout(0.2),
            Dense(3, activation='linear')
        ])
    elif model_name == "model_conv3_dropout":
        model = Sequential([
            Input(shape=(scan_length, 2)), 
            Conv1D(64, kernel_size=5, activation=chosen_activation),
            MaxPooling1D(pool_size=4),
            Conv1D(32, kernel_size=5, activation=chosen_activation),
            MaxPooling1D(pool_size=4),
            Conv1D(16, kernel_size=5, activation=chosen_activation),
            MaxPooling1D(pool_size=4),
            Flatten(),
            Dropout(0.2),
            Dense(3, activation='linear')
        ])
    elif model_name == "model_conv1_norm":
        model = Sequential([
            Input(shape=(scan_length, 2)), 
            Conv1D(64, kernel_size=5, activation=chosen_activation),
            MaxPooling1D(pool_size=4),
            BatchNormalization(),
            Flatten(),
            Dense(3, activation='linear')
        ])
    elif model_name == "model_conv2_norm":
        model = Sequential([
            Input(shape=(scan_length, 2)), 
            Conv1D(64, kernel_size=5, activation=chosen_activation),
            MaxPooling1D(pool_size=4),
            BatchNormalization(),
            Conv1D(32, kernel_size=5, activation=chosen_activation),
            MaxPooling1D(pool_size=4),
            BatchNormalization(),
            Flatten(),
            Dense(3, activation='linear')
        ])
    elif model_name == "model_conv3_norm":
        model = Sequential([
            Input(shape=(scan_length, 2)), 
            Conv1D(64, kernel_size=5, activation=chosen_activation),
            MaxPooling1D(pool_size=4),
            BatchNormalization(),
            Conv1D(32, kernel_size=5, activation=chosen_activation),
            MaxPooling1D(pool_size=4),
            BatchNormalization(),
            Conv1D(16, kernel_size=5, activation=chosen_activation),
            MaxPooling1D(pool_size=4),
            BatchNormalization(),
            Flatten(),
            Dense(3, activation='linear')
        ])
    return model