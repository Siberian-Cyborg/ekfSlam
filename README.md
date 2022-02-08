# ekfSLAM
Implementation of a simple landmark-based Extended Kalman Filter SLAM with known data association for a mobile robot plattform using a IP camera and APRIL tags. In order to run the code you will need MATLAB with the Computer Vision Toolbox and the Support Package for IP Cameras.

The robot plattform uses HEBI modules (https://www.hebirobotics.com/hardware) which allow for easy control of the electric motors. The code is written to work with those HEBI motors. Obviously you would have to change the code to suit your motors.
This is the HEBI API for matlab (https://docs.hebi.us/tools.html#matlab-api). You will need to download it here (https://files.hebi.us/download/matlab/hebi-matlab-1.7.5.zip)
The HEBI modules were controlled using the HEBI App for the phone.

First a camera calibration using APRIL tags is done (https://de.mathworks.com/help/vision/ug/camera-calibration-using-apriltag-markers.html).
Then the intrinsics of the camera are used to measure the distance of APRIL tags relative to the camera (https://de.mathworks.com/help/vision/ref/readapriltag.html).
The measurements are fed into the EKF SLAM.

The robot has four MECANUM wheels allowing for sideways movement and pinned rotations (rotating around the up axis on the spot). The plattform is equipped with an IP camera which detecs APRIL tags. The tags have unique ids, wherefore the data association is known a priori. The EKF SLAM is very basic and can be used as a starting point for more advanced implementations (computational efficient implementation, dynamic enviroment, unknown data association, 3 dimensional etc.).

To understand the theoretical background you can go thru this lecture: http://ais.informatik.uni-freiburg.de/teaching/ws12/mapping/pdf/slam04-ekf-slam.pdf

here a video of the robot:
TODO add video

here a screencast of the mapping:
TODO add screencast

# SLAM
SLAM stands for *S*imultaneous *L*ocalization *A*nd *M*apping. Its the process of driving around in an unknow enviroment while creating a map of your surrounding and localizing yourself relative to some landmarks in that enviroment. It is a crucial part of any autnomous robot that needs to navigate in an unknown or dynamically changing enviroment. The map created with SLAM can be used to plan optimal, collision free trjectories thru the space. There are plenty of different methods for solving the SLAM problem. The EKF SLAM is the oldest one. It is still widely used, especially for prototyping and testing. Other SLAM methods are: Particle-based methods like the FAST SLAM (1 & 2) by Montemerlo and Thrun (http://robots.stanford.edu/papers/montemerlo.fastslam-tr.pdf) or GRAPH SLAM which is state of the art. There are many different Graph-based SLAM approaches depending on the sensors used for observing the enviroment (RGBD, Stereo camers, Monocular vision, LIDAR). In general camera-based GRAPH SLAMs can be split into two groups, namely direct and indirect approaches. The former takes the raw camera data to perform the data association and SLAM whereas the latter first extracts features from the images like corners and then only uses thoses to perform the SLAM. If you want to dig deeper here is a excellent lecture series on SLAM: https://www.youtube.com/watch?v=U6vr3iNrwRA&list=PLgnQpQtFTOGQrZ4O5QzbIHgl3b1JHimN_ as well as https://www.youtube.com/watch?v=mQvKhmWagB4&list=PLgnQpQtFTOGQh_J16IMwDlji18SWQ2PZ6
