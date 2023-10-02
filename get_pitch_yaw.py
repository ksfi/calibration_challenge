import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

from scipy.linalg import inv
from scipy.spatial.transform import Rotation

from typing import Tuple, List, Any, IO, Dict

FILE_KF: str = "slam/run/build/keyframe_trajectory.txt"
FILE_F: str = "slam/run/build/frame_trajectory.txt"
kf: bool = False
try:
    DATAS: np.ndarray = np.loadtxt(FILE_KF) if kf else np.loadtxt(FILE_F)
except:
    ret_error_msg: str = f"no data file corresponding to {FILE_KF}" if kf else f"no data file corresponding to {FILE_F}"
    print(ret_error_msg)
    exit()

ts_idx: Dict = dict() # map the timestamp of the key frame to the index of the frame for extrapolation
for l in np.loadtxt(FILE_F):
    ts_idx[l[1]] = int(l[0]-221)

eon_f_focal_length: int = 910
eon_f_frame_size: Tuple[int, int] = (1164, 874)
eon_fcam_intrinsics: np.ndarray = np.array([
    [eon_f_focal_length,  0.0,  float(eon_f_frame_size[0])/2],
    [0.0,  eon_f_focal_length,  float(eon_f_frame_size[1])/2],
    [0.0,  0.0,                                          1.0]])
pp: Tuple[float, float] = (eon_fcam_intrinsics[0][2], eon_fcam_intrinsics[1][2])


def calib_challenge(coordvp: List[float], video_id: int, complete_from_begin: int, plot: bool=False) -> None:
    '''
    coordvp: array size 2 [x,y]
    complete_from_begin: int number of lines to add at the beginning of the returned file
    '''
    def pitch_yaw_via_vp(vp: List[float]) -> Tuple[float, float]:
        def normalize(img_pts_in: List[float], intrinsics: np.ndarray=eon_fcam_intrinsics) -> np.ndarray:
            # normalizes image coordinates
            # accepts single pt or array of pts
            intrinsics_inv: np.ndarray = np.linalg.inv(intrinsics)
            img_pts: np.ndarray = np.array(img_pts_in)
            input_shape: Tuple[int,...] = img_pts.shape
            img_pts = np.atleast_2d(img_pts)
            img_pts = np.hstack((img_pts, np.ones((img_pts.shape[0], 1))))
            img_pts_normalized: np.ndarray = img_pts.dot(intrinsics_inv.T)
            img_pts_normalized[(img_pts < 0).any(axis=1)] = np.nan
            return img_pts_normalized[:, :2].reshape(input_shape)
        def get_calib_from_vp(vp: List[float]) -> Tuple[float, float]:
            vp_norm = normalize(vp)
            yaw_calib = np.arctan(vp_norm[0])
            pitch_calib = -np.arctan(vp_norm[1]*np.cos(yaw_calib))
            roll_calib = 0
            return pitch_calib, yaw_calib
        return get_calib_from_vp(vp)

    offset_pitch, offset_yaw = pitch_yaw_via_vp(coordvp)
    def get_pose(offset_pitch: float=offset_pitch, offset_yaw: float=offset_yaw, video: int=video_id, plot: bool=False, complete_from_begin: int=complete_from_begin) -> None:
        ret: IO[str] = open(f"{video}.txt", "w")
        translations: List[List[float]] = []
        rotations: List[Any] = []
        rpy: List[List[float]] = []
        pitch: List[float] = []
        yaw: List[float] = []
        for l in DATAS:
            quat = l[-4:]
            t_wc = l[1:4] if kf else l[2:5]
            R = Rotation.from_quat(quat)
            rotations.append(R)
            translations.append(t_wc)

        for i in range(1, len(rotations)):
            R_prev: Any = rotations[i-1].inv()
            R_curr: Any = rotations[i]
            R_rel: Any = R_prev * R_curr
            rpy.append(R_rel.as_euler('XYZ', degrees=False))
            pitch.append(offset_pitch+rpy[-1][0])
            yaw.append(offset_yaw+rpy[-1][1])
            if i == 1 and complete_from_begin > 0:
                for i in range(complete_from_begin):
                    ret.write(f"{pitch[-1]} {yaw[-1]}\n")
            ret.write(f"{pitch[-1]} {yaw[-1]}\n")

        def plot_g() -> None:
            plt.plot(pitch, color='red')
            plt.plot(yaw, color='blue')
            plt.show()
        if plot:
            plot_g()

        def extrapolation() -> None:
            '''
            when using key frames
            '''
            rpy_rel: List[List[float]] = []
            for i in range(1, len(rpy)):
                temp: List[float] = [rpy[i][l]-rpy[i-1][l] for l in range(3)]
                if i > 2:
                    f = lambda x, k: ((temp[k]-rpy_rel[-1][k])/(ts_idx[rpy[i][0]]-ts_idx[rpy[i-1][0]]))*x + rpy_rel[-1][k]
                    for j in range(ts_idx[rpy[i-1][0]], ts_idx[rpy[i][0]]):
                        rpy_rel.append([f(j, 0), f(j, 1), f(j, 2)])
                rpy_rel.append(temp)
    get_pose()

def plt_img(path: str) -> None:
    img = mpimg.imread(path)
    imgplot = plt.imshow(img)
    plt.show()

if __name__ == "__main__":
    vp_coord = [638.7, 435.5]
    video_id = 0
    calib_challenge(vp_coord, video_id, 17, True)
