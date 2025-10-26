import os
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Conv1D, MaxPooling1D, Flatten, Dense, Dropout, Input, GlobalAveragePooling1D, LeakyReLU, BatchNormalization
from tensorflow.keras import regularizers

scan_length = 360

def build_model(model_name):
    if model_name == "model_conv1":
        model = Sequential([
            Input(shape=(scan_length, 2)), 
            Conv1D(64, kernel_size=5, activation='relu'),
            MaxPooling1D(pool_size=4),
            Flatten(),
            Dense(3, activation='linear')
        ])
    return model