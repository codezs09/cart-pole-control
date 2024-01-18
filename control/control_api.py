
from proto.proto_gen.data_pb2 import Frame

class ControlAPI:
    def __init__(self, super_param_path, control_param_path):
        pass

    def control(self, state, target, last_control):
        pass

    def get_frame_msg(self):
        frame_msg = Frame()
        return frame_msg
