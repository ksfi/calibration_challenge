Lastly I did the [Comma AI calibration challenge](https://github.com/commaai/calib_challenge).



Here's the pipeline:


I'm using a [SLAM algorithm](https://github.com/stella-cv/stella_vslam) (see slam/run/src/calbration_challenge_slam.cc) to get the accumulated [pose](https://en.wikipedia.org/wiki/Pose_(computer_vision) over the video.
I'm adding [yolov8](https://github.com/ultralytics/ultralytics) to the SLAM pipeline to [infer and mask dynamic objects](https://www.sciencedirect.com/science/article/pii/S2214914720304402) as much as possible from features extraction.


After SLAM I'm taking the quaternions and turn them into relative pitches and yaws (see get_pitch_yaw.py).
SLAM gives the evolution of the pitch and yaw starting from an inital pose of 0,
to get the initial pose I'm computing the pitch and yaw offsets via the vanishing point of the first frame and add it to each pitch and yaw values.


I believe I could estimate the distortion coefficients of the camera for more accurate results, I infered the depth of the frames for a video but with poor results.


I saw you prefer using static typing hints so I added mypy coverage afterward (--check-untyped-defs).


I've achieved below 10% of error on the labeled set and a poor 33% on the unlabeled set, good enough for the Comma team to accept it though.