import json
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
