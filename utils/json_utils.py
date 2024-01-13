import json

def load_json(json_path):
    with open(json_path, 'r') as f:
        json_data = json.load(f)
    return json_data
