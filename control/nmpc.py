

import ctypes
import numpy as np

from proto.proto_gen.data_pb2 import Data, Frame


NMPC_CPP_LIB_PATH = "/path/to/libnmpc.so"
NMPC_CPP_LIB = ctypes.CDLL(NMPC_CPP_LIB_PATH)

class NMPC:
    def __init__(self, super_param_path, control_param_path):
        self._create_nmpc(super_param_path, control_param_path)

    def _create_nmpc(self, super_param_path, control_param_path):
        super_param_path_s = ctypes.create_string_buffer(super_param_path.encode('utf-8'))
        control_param_path_s = ctypes.create_string_buffer(control_param_path.encode('utf-8'))
        self._nmpc = NMPC_CPP_LIB.CreateNMPC(super_param_path_s, control_param_path_s)

    def control(self, state, target, last_control):
        if not isinstance(state, np.ndarray): 
            state = np.array(state, dtype=np.float64)
        if not isinstance(target, dtype=np.float64): 
            target = np.array(target, dtype=np.float64)

        NMPC_CPP_LIB.RunSolver.argtypes = [ctypes.c_void_p, \
                                           ctypes.POINTER(ctypes.c_double), ctypes.c_int, \
                                           ctypes.POINTER(ctypes.c_double), ctypes.c_int, \
                                           ctypes.c_double]
        NMPC_CPP_LIB.RunSolver(self._nmpc, state.ctypes.data_as(ctypes.POINTER(ctypes.c_double)), state.size, \
                                target.data_as(ctypes.POINTER(ctypes.c_double)), target.size, \
                                last_control)

    def get_frame_msg(self):
        NMPC_CPP_LIB.GetSerializedFrameMsg.restype = ctypes.c_char_p
        serialized_frame_msg = NMPC_CPP_LIB.GetSerializedFrameMsg(self._nmpc)
        frame_msg = Frame()
        frame_msg.ParseFromString(serialized_frame_msg)
        return frame_msg

    def __del__(self):
        NMPC_CPP_LIB.DestroyNMPC(self._nmpc)