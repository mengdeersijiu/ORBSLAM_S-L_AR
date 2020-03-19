#ifndef PLANAR_TRACKING_H
#define PLANAR_TRACKING_H
#include <opencv2/aruco.hpp>
using namespace std;
using namespace cv;

class Tracker
{
public:
    Tracker(Ptr<cv::Feature2D> _detector, Ptr<cv::DescriptorMatcher> _matcher, Mat _K) :
            detector(_detector),
            matcher(_matcher),
            K(_K)
            {
                detectorParams = cv::aruco::DetectorParameters::create();
                dictionary = cv::aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME::DICT_ARUCO_ORIGINAL);
            }

    Tracker(Ptr<cv::Feature2D> _detector, Ptr<cv::DescriptorMatcher> _matcher, Mat _K, Mat _DistCoef) :
            detector(_detector),
            matcher(_matcher),
            K(_K),
            DistCoef(_DistCoef)
            {
                detectorParams = cv::aruco::DetectorParameters::create();
                dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_ARUCO_ORIGINAL);
            }
    //Tracker(){}

    void setFirstFrame(const char * first_frame_path);

    bool process(const cv::Mat frame_left, bool slamMode);
    bool process1(cv::Mat &frame_left, bool slamMode);
    bool processARUCO(cv::Mat &frame_left, bool slamMode);

    glm::mat4 getInitModelMatrix(bool slamMode);
    glm::mat4 getInitModelMatrix1(cv::Mat Tpw);

protected:

    Ptr<cv::Feature2D> detector;
    Ptr<cv::DescriptorMatcher> matcher;
    cv::Mat K, rvec, tvec, DistCoef;
    cv::Mat rvec_clone = cv::Mat::zeros(3,1,CV_64FC1);
    cv::Mat tvec_clone = cv::Mat::zeros(3,1,CV_64FC1);
    cv::Mat first_frame, first_desc;
    vector<cv::KeyPoint> first_kp, first_kp_1;//first_kp_1是debug用
    vector<cv::Point2f> object_bb;

    //ARUCO marker相关变量
    Ptr<cv::aruco::Dictionary> dictionary;
    vector<int> markerIds;
    vector<vector<cv::Point2f>> markerCorners, rejectedCandidates;
    float markerLength = 0.086;
    Ptr<cv::aruco::DetectorParameters> detectorParams;
};

#endif //BOB_AR_PLANAR_TRACKING_H
