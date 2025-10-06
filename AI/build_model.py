import os
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Conv1D, Flatten, Dense, Dropout, Input, GlobalAveragePooling1D

scan_length = 360

def build_model(model_name):
    if model_name == "model1":
        model = Sequential([
            Input(shape=(scan_length, 2)), 
            Conv1D(32, kernel_size=5, activation='relu'),
            Conv1D(64, kernel_size=5, activation='relu'),
            Flatten(),
            Dense(64, activation ='relu'),
            Dropout(0.2),
            Dense(3, activation='linear')
        ])
    elif model_name == "model_conv": # glebokosc na conv1D -> powinno wylapywac skomplikowane zaleznosci
        model = Sequential([
            Input(shape=(scan_length, 2)), 
            Conv1D(32, kernel_size=5, activation='relu'),
            Conv1D(64, kernel_size=5, activation='relu'),
            Conv1D(128, kernel_size=7, activation='relu'),
            Conv1D(256, kernel_size=10, activation='relu'),
            GlobalAveragePooling1D(),
            Dense(64, activation ='relu'),
            Dropout(0.2),
            Dense(3, activation='linear')
        ])
    elif model_name == "model_dense":
        model = Sequential([
            Input(shape=(scan_length, 2)), 
            Conv1D(32, kernel_size=5, activation='relu'),
            Conv1D(64, kernel_size=5, activation='relu'),
            Conv1D(128, kernel_size=7, activation='relu'),
            Conv1D(256, kernel_size=10, activation='relu'),
            Flatten(),
            Dense(256, activation ='relu'),
            Dense(128, activation ='relu'),
            Dense(64, activation ='relu'),
            Dropout(0.2),
            Dense(3, activation='linear')
        ])
    # elif model_name == "model4":
    # elif model_name == "model5":
    # elif model_name == "model6":
    return model