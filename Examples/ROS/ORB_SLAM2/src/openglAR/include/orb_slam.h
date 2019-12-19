#ifndef ORB_SLAM_H
#define ORB_SLAM_H

glm::mat4 getViewMatrix(bool slamMode);
Mat getCameraMatrix();
Mat getDistorsion();
Mat getM1l();
Mat getM2l();
Mat getM1r();
Mat getM2r();


void stereoRemap(Mat frame_left, Mat frame_right, Mat& frame_left_rectified, Mat& frame_right_rectified);
bool initTracking(const char * Extrincis_path);
bool initTracking1(const char * Extrincis_path);
bool trackStereo(Mat CameraPose);

#endif //BOB_AR_ORB_SLAM_H
