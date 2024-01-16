
import os
import sys
import ctypes
import numpy as np

from proto.proto_gen.data_pb2 import Frame


SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
NMPC_CPP_LIB_PATH = os.path.join(os.path.dirname(SCRIPT_DIR), "build/control/libnmpc.so")
NMPC_CPP_LIB = ctypes.CDLL(NMPC_CPP_LIB_PATH)

sys.path.append('/home/deeproute/Projects/nmpc-cart-pole/build/control')

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
        if not isinstance(target, np.ndarray): 
            target = np.array(target, dtype=np.float64)

        NMPC_CPP_LIB.RunSolver.argtypes = [ctypes.c_void_p, \
                                           ctypes.POINTER(ctypes.c_double), ctypes.c_int, \
                                           ctypes.POINTER(ctypes.c_double), ctypes.c_int, \
                                           ctypes.c_double]
        NMPC_CPP_LIB.RunSolver(self._nmpc, state.ctypes.data_as(ctypes.POINTER(ctypes.c_double)), state.size, \
                                target.ctypes.data_as(ctypes.POINTER(ctypes.c_double)), target.size, \
                                last_control)

    def get_frame_msg(self):
        c_str = ctypes.c_char_p()
        c_str_size = ctypes.c_int()

        NMPC_CPP_LIB.GetSerializedFrameMsg.argtypes = [ctypes.c_void_p, \
                                                       ctypes.POINTER(ctypes.c_char_p), \
                                                       ctypes.POINTER(ctypes.c_int)]
        NMPC_CPP_LIB.GetSerializedFrameMsg.restype = None
        NMPC_CPP_LIB.GetSerializedFrameMsg(self._nmpc, \
                                                                   ctypes.byref(c_str), \
                                                                   ctypes.byref(c_str_size))
        serialized_frame_msg = ctypes.string_at(c_str, c_str_size.value)        
        # print("serialized_frame_msg: ", serialized_frame_msg)
        frame_msg = Frame()
        frame_msg.ParseFromString(serialized_frame_msg)

        # print(f"PYTHON frame: x={frame_msg.x:.6f}, dx={frame_msg.dx:.6f}, theta={frame_msg.theta:.6f}, dtheta={frame_msg.dtheta:.6f}, force={frame_msg.force:.6f}")
        # horizon = frame_msg.horizon
        # for i in range(len(horizon.t)):
        #     print(f"PYTHON horizon: t={horizon.t[i]:.6f}, x={horizon.x[i]:.6f}, dx={horizon.dx[i]:.6f}, \
        #           theta={horizon.theta[i]:.6f}, dtheta={horizon.dtheta[i]:.6f}, force={horizon.force[i]:.6f}")
            
        return frame_msg

    def __del__(self):
        NMPC_CPP_LIB.DestroyNMPC(self._nmpc)