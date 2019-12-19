#ifndef PLANAR_TRACKING_H
#define PLANAR_TRACKING_H

using namespace std;
using namespace cv;

class Tracker
{
public:
    Tracker(Ptr<Feature2D> _detector, Ptr<DescriptorMatcher> _matcher, Mat _K) :
            detector(_detector),
            matcher(_matcher),
            K(_K)
            {}

    Tracker(Ptr<Feature2D> _detector, Ptr<DescriptorMatcher> _matcher, Mat _K, Mat _DistCoef) :
            detector(_detector),
            matcher(_matcher),
            K(_K),
            DistCoef(_DistCoef)
            {}
    //Tracker(){}

    void setFirstFrame(const char * first_frame_path);

    bool process(const Mat frame_left, bool slamMode);
    bool process1(cv::Mat &frame_left, bool slamMode);

    glm::mat4 getInitModelMatrix();
    glm::mat4 getInitModelMatrix1(cv::Mat Tpw);

protected:

    Ptr<Feature2D> detector;
    Ptr<DescriptorMatcher> matcher;
    Mat K, rvec, tvec, DistCoef;
    Mat first_frame, first_desc;
    vector<KeyPoint> first_kp, first_kp_1;//first_kp_1是debug用
    vector<Point2f> object_bb;
};

#endif //BOB_AR_PLANAR_TRACKING_H
