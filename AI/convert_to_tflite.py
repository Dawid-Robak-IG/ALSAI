import os
os.environ["TF_CPP_MIN_LOG_LEVEL"] = "3"
os.environ["TF_ENABLE_ONEDNN_OPTS"] = "0" 

import tensorflow as tf
import sys
from colorama import Fore, init 

def convert_to_tflite(model_name):
    init(autoreset=True)
    model_path = os.path.expanduser(f"~/ALSAI/AI/models/{model_name}.keras")
    tflite_path = os.path.expanduser(f"~/ALSAI/AI/models/{model_name}.tflite")

    model = tf.keras.models.load_model(model_path)

    converter = tf.lite.TFLiteConverter.from_keras_model(model)
    converter.target_spec.supported_types = [tf.float32]
    tflite_model = converter.convert()

    with open(tflite_path, "wb") as f:
        f.write(tflite_model)

    print(Fore.GREEN + f"Save tflite to {tflite_path}")

def main():
    if len(sys.argv) > 1:
        convert_to_tflite(sys.argv[1])
    else:
        print(Fore.YELLOW + "Didn't get name for model, exiting")
        sys.exit(1)

if __name__ == "__main__":
    main()

