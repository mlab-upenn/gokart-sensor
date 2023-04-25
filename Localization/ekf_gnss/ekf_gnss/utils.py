from scipy.spatial.transform import Rotation as Rot

def rot_mat_2d(angle):
    return Rot.from_euler('z', angle).as_matrix()[0:2, 0:2]