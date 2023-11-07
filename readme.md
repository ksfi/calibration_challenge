[Comma AI calibration challenge](https://github.com/commaai/calib_challenge)



I'm using [SLAM](https://github.com/stella-cv/stella_vslam) (slam/calbration_challenge_slam.cc) to get the accumulated [pose](https://en.wikipedia.org/wiki/Pose_(computer_vision)) over the video.
I'm adding [yolov8](https://github.com/ultralytics/ultralytics) to the SLAM pipeline to [infer and mask dynamic objects](https://www.sciencedirect.com/science/article/pii/S2214914720304402) as much as possible from features extraction.


After SLAM I'm taking the quaternions and turning them into relative pitches and yaws (see get_pitch_yaw.py).
SLAM gives the evolution of the pitch and yaw starting from an inital pose of 0,
to get the initial pose I'm computing the pitch and yaw offsets via the vanishing point of the first frame and add it to each pitch and yaw values.


I believe I could estimate the distortion coefficients of the camera for more accurate results, I [infered the depth](https://huggingface.co/docs/transformers/main/tasks/monocular_depth_estimation) of the frames for a video but with poor results.


It achieves below 10% on the labeled set and 33% on the unlabeled set.
