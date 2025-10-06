import os
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Conv1D, Flatten, Dense, Dropout, Input

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
    elif model_name == "model2":
    elif model_name == "model3":
    elif model_name == "model4":
    elif model_name == "model5":
    elif model_name == "model6":
    return model