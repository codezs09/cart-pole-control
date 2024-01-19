import os
import json
import imageio
from proto.proto_gen.data_pb2 import Data

def load_json(json_path):
    with open(json_path, 'r') as f:
        json_data = json.load(f)
    return json_data

def save_data(data_msg, file_path):
    serialized_data_msg = data_msg.SerializeToString()
    with open(file_path, 'wb') as f:
        f.write(serialized_data_msg)

def load_data(file_path):
    with open(file_path, 'rb') as f:
        serialized_data_msg = f.read()
    data_msg = Data()
    data_msg.ParseFromString(serialized_data_msg)
    return data_msg

def make_gif(frames_dir, export_path, dt, infinite_loop=False):
    image_files = [f for f in os.listdir(frames_dir) if f.endswith('.png')]
    image_files.sort(key=lambda x: int(x.split('.')[0]))

    images = []
    for image_file in image_files:
        image_path = os.path.join(frames_dir, image_file)
        images.append(imageio.imread(image_path))

    loop = 0 if infinite_loop else 1
    imageio.mimsave(export_path, images, duration=dt, loop=loop)