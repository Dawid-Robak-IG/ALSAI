import os
import sys
from colorama import Fore, Style, init
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Conv1D, Flatten, Dense, Dropout, Input, GlobalAveragePooling1D, LeakyReLU, BatchNormalization
from tensorflow.keras import regularizers


import visualkeras
import build_model

if len(sys.argv) > 1:
    model_name = sys.argv[1]
else:
    print(Fore.YELLOW + "Didn't get name for model")
    sys.exit(1)

model = build_model.build_model(model_name)

model.build((None,360,2))
model.summary()
visualkeras.layered_view(model, scale_xy=1, scale_z=1, max_z=1000, legend=True).show()