import os
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Conv1D, Flatten, Dense, Dropout, Input, GlobalAveragePooling1D, LeakyReLU, BatchNormalization
from tensorflow.keras import regularizers

scan_length = 640

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
    elif model_name == "model_tanh":
        model = Sequential([
            Input(shape=(scan_length, 2)), 
            Conv1D(32, kernel_size=5, activation='tanh'),
            Conv1D(64, kernel_size=5, activation='tanh'),
            Flatten(),
            Dense(64, activation ='tanh'),
            Dropout(0.2),
            Dense(3, activation='linear')
        ])
    elif model_name == "model_leaky":
        model = Sequential([
            Input(shape=(scan_length, 2)), 
            Conv1D(32, kernel_size=5),
            LeakyReLU(alpha=0.1),
            Conv1D(64, kernel_size=5),
            LeakyReLU(alpha=0.1),
            Flatten(),
            Dense(64),
            LeakyReLU(alpha=0.1),
            Dropout(0.2),
            Dense(3, activation='linear')
        ])
    elif model_name == "model_elu":
        model = Sequential([
            Input(shape=(scan_length, 2)), 
            Conv1D(32, kernel_size=5, activation='elu'),
            Conv1D(64, kernel_size=5, activation='elu'),
            Flatten(),
            Dense(64, activation ='elu'),
            Dropout(0.2),
            Dense(3, activation='linear')
        ])
    elif model_name == "model_norm":
        model = Sequential([
            Input(shape=(scan_length, 2)), 
            Conv1D(32, kernel_size=5, activation='relu'),
            BatchNormalization(),
            Conv1D(64, kernel_size=5, activation='relu'),
            BatchNormalization(),
            Flatten(),
            Dense(64, activation ='relu'),
            BatchNormalization(),
            Dropout(0.2),
            Dense(3, activation='linear')
        ])
    elif model_name == "model_reg":
        model = Sequential([
            Input(shape=(scan_length, 2)), 
            Conv1D(32, kernel_size=5, activation='relu',kernel_regularizer=regularizers.l2(0.001)),
            Conv1D(64, kernel_size=5, activation='relu',kernel_regularizer=regularizers.l2(0.001)),
            Flatten(),
            Dense(64, activation ='relu',kernel_regularizer=regularizers.l2(0.001)),
            Dropout(0.2),
            Dense(3, activation='linear')
        ])
    return model