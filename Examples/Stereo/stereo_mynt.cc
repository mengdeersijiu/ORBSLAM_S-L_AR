#include <iostream>
#include <fstream>
#include <algorithm>
#include <cassert>
#include <opencv2/opencv.hpp>
#include "../../include/System.h"

#include "mynteye/api/api.h"
#include "mynteye/logger.h"
#include "mynteye/device/types.h"

MYNTEYE_USE_NAMESPACE

bool flag = true;
void exit_while(int sig)
{
    flag = false;
}

class ImageGrabber {
public:
    explicit ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM) {}
    void GrabStereo(const cv::Mat& left_img, const cv::Mat& right_img, double timeStamp);
    ORB_SLAM2::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l, M2l, M1r, M2r;
};

void ImageGrabber::GrabStereo(const cv::Mat& left_img, const cv::Mat& right_img, double timeStamp)
{
    if (left_img.empty() || right_img.empty())
    {
        assert("left_img == empty || right_img == empty");
    }

    if (do_rectify)
    {
        cv::Mat _left_img, _right_img;
        //cv::remap(left_img, _left_img, M1l, M2l, cv::INTER_LINEAR);
        //cv::remap(right_img, _right_img, M1r, M2r, cv::INTER_LINEAR);
        //mpSLAM->TrackStereo(_left_img, _right_img, timeStamp);
        mpSLAM->TrackStereoOriginalIm(left_img, right_img, timeStamp);
    }
    else
    {
        mpSLAM->TrackStereo(left_img, right_img, timeStamp);
    }
}

int main(int argc, char** argv) {
    glog_init _(argc, argv);

    if (argc != 4) {
        std::cout << std::endl << "Usage: ./stereo_mynt path_to_vocabulary path_to_setting do_rectify " << std::endl;
        return 1;
    }

    std::cout << "--args: " << std::endl
              << "  path_to_vocabulary: " << argv[1] << std::endl
              << "  path_to_setting: " << argv[2] << std::endl
              << "  do_rectify(ture | false): " << argv[3] << std::endl;

    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::STEREO, true);
    ImageGrabber igb(&SLAM);

    std::stringstream ss(argv[3]);
    ss >> boolalpha >> igb.do_rectify;

    if (igb.do_rectify) {
        cv::FileStorage fsSetting(argv[2], cv::FileStorage::READ);
        if (!fsSetting.isOpened()) {
            std::cerr << "error: wrong path to setting" << std::endl;
            return -1;
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSetting["LEFT.K"] >> K_l;
        fsSetting["RIGHT.K"] >>  K_r;

        fsSetting["LEFT.P"] >> P_l;
        fsSetting["RIGHT.P"] >> P_r;

        fsSetting["LEFT.R"] >> R_l;
        fsSetting["RIGHT.R"] >> R_r;

        fsSetting["LEFT.D"] >> D_l;
        fsSetting["RIGHT.D"] >> D_r;
        cout<<"1"<<endl;

        int rows_l = fsSetting["LEFT.height"];
        int cols_l = fsSetting["LEFT.width"];
        int rows_r = fsSetting["RIGHT.height"];
        int cols_r = fsSetting["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
           rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)	{
            std::cerr << "error: calibration parameters to rectify stereo are missing!" << std::endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
        fsSetting.release();
    }

    auto &&api = API::Create(argc, argv);
    if (!api) return 1;

    auto request = api->GetStreamRequest();

    auto dev_name = api->GetInfo(Info::DEVICE_NAME);
    if (dev_name.substr(0, 11) == "MYNT-EYE-S1") {
        request.height = 480;
        request.width = 640;
        request.fps = 60;
    } else if (dev_name.substr(0, 11) == "MYNT-EYE-S2") {
        request.height = 1280;
        request.width = 400;
        request.fps = 60;
    } else {
        std::cout << "unknow device";
        return -1;
    }

    api->ConfigStreamRequest(request);

    api->EnableStreamData(Stream::LEFT_RECTIFIED);
    api->EnableStreamData(Stream::RIGHT_RECTIFIED);

    api->Start(Source::VIDEO_STREAMING);

    while (flag)
    {
        api->WaitForStreams();
        auto left_data = api->GetStreamData(Stream::LEFT);
        auto right_data = api->GetStreamData(Stream::RIGHT);
        if (left_data.frame.empty() || right_data.frame.empty())
        {
            continue;
        }
        cv::Mat left_img, right_img;
        if (left_data.frame.channels() == 1)
        {
            // s1
            left_img = left_data.frame;
            right_img = right_data.frame;
        }
        else if (left_data.frame.channels() >= 3)
        {
            // s2
            cv::cvtColor(left_data.frame, left_img, cv::COLOR_RGB2GRAY);
            cv::cvtColor(right_data.frame, right_img, cv::COLOR_RGB2GRAY);
        }

        igb.GrabStereo(left_img, right_img, left_data.img->timestamp*0.00001f);
    }

    SLAM.Shutdown();
    api->Stop(Source::VIDEO_STREAMING);
    std::cout << "save camera trajectory..." << std::endl;
    SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");
    std::cout << "save camera trajectory complete." << std::endl;

    return 0;
}
