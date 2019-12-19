/**
* Th file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include <fstream>
#include <algorithm>
#include <cassert>
#include <opencv2/opencv.hpp>

#include"../../../include/System.h"
#include"ViewerAR.h"

#include "mynteye/api/api.h"
#include "mynteye/logger.h"
#include "mynteye/device/types.h"

using namespace std;
MYNTEYE_USE_NAMESPACE

ORB_SLAM2::ViewerAR viewerAR;
bool bRGB = true;

cv::Mat K;
cv::Mat DistCoef;

bool flag = true;
void exit_while(int sig)
{
  flag = false;
}

class ImageGrabber
{
    public: 
    	ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}
        void GrabStereo(const cv::Mat& left_img, const cv::Mat& right_img, double timeStamp);
    ORB_SLAM2::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l, M2l, M1r, M2r;
};



int main(int argc, char** argv)
{

    glog_init _(argc, argv);


    if (argc != 4 )
    {
    	std::cout << std::endl << "Usage: ./stereo_mynt path_to_vocabulary path_to_setting do_rectify " << std::endl;
	return 1;
    }

    std::cout << "--args: " << std::endl
    	      << "	path_to_vocabulary: " << argv[1] << std::endl
	      << "	path_to_setting: " << argv[2] << std::endl
	      << "	do_rectify(ture | false): " << argv[3] << std::endl;

    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::STEREO, false);
    
    cout << endl << endl;
    cout << "-----------------------" << endl;
    cout << "Augmented Reality Demo" << endl;
    cout << "1) Translate the camera to initialize SLAM." << endl;
    cout << "2) Look at a planar region and translate the camera." << endl;
    cout << "3) Press Insert Cube to place a virtual cube in the plane. " << endl;
    cout << endl;
    cout << "You can place several cubes in different planes." << endl;
    cout << "-----------------------" << endl;
    cout << endl;
    
    viewerAR.SetSLAM(&SLAM);
    
    ImageGrabber igb(&SLAM);

    std::stringstream ss(argv[3]);
    	ss >> boolalpha >> igb.do_rectify;

    if (igb.do_rectify)
    {
    	cv::FileStorage fsSetting(argv[2], cv::FileStorage::READ);
        if (!fsSetting.isOpened())
        {
            std::cerr << "error: wrong path to setting"	<< std::endl;
            return -1;
        }

        bRGB = static_cast<bool>((int)fsSetting["Camera.RGB"]);

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;

        float fps = fsSetting["Camera.fps"];
        viewerAR.SetFPS(fps);

        float fx = fsSetting["Camera.fx"];
        float fy = fsSetting["Camera.fy"];
        float cx = fsSetting["Camera.cx"];
        float cy = fsSetting["Camera.cy"];

        viewerAR.SetCameraCalibration(fx,fy,cx,cy);

        K = cv::Mat::eye(3,3,CV_32F);
        K.at<float>(0,0) = fx;
        K.at<float>(1,1) = fy;
        K.at<float>(0,2) = cx;
        K.at<float>(1,2) = cy;

        DistCoef = cv::Mat::zeros(4,1,CV_32F);
        DistCoef.at<float>(0) = fsSetting["Camera.k1"];
        DistCoef.at<float>(1) = fsSetting["Camera.k2"];
        DistCoef.at<float>(2) = fsSetting["Camera.p1"];
        DistCoef.at<float>(3) = fsSetting["Camera.p2"];
        const float k3 = fsSetting["Camera.k3"];
        if(k3!=0)
        {
            DistCoef.resize(5);
            DistCoef.at<float>(4) = k3;
        }

        fsSetting["LEFT.K"] >> K_l;
        fsSetting["RIGHT.K"] >>  K_r;

        fsSetting["LEFT.P"] >> P_l;
        fsSetting["RIGHT.P"] >> P_r;

        fsSetting["LEFT.R"] >> R_l;
        fsSetting["RIGHT.R"] >> R_r;

        fsSetting["LEFT.D"] >> D_l;
        fsSetting["RIGHT.D"] >> D_r;

        int rows_l = fsSetting["LEFT.height"];
        int cols_l = fsSetting["LEFT.width"];
        int rows_r = fsSetting["RIGHT.height"];
        int cols_r = fsSetting["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)	
	    {
		    std::cerr << "error: calibration parameters to rectify stereo are missing!" << std::endl;
		    return -1;
	    }

	    cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);//CV_32F
	    cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
	
        string sTpwPath = static_cast<string>((string)fsSetting["Tpw.path"]);
        viewerAR.SetTpwPath(sTpwPath);
      
	    fsSetting.release();
    }

    auto &&api = API::Create(0, nullptr);
    if (!api) return 1;
//    auto request = api->GetStreamRequest();//不知道什么原因这么配置无效
//    request.height = 1280;
//    request.width = 400;
//    request.fps = 60;
    bool ok;
    auto &&request = api->SelectStreamRequest(&ok);//手动选择

    api->ConfigStreamRequest(request);
    auto in_left = api->GetIntrinsicsBase(Stream::LEFT);//内参基类
    auto in_right = api->GetIntrinsicsBase(Stream::RIGHT);
    if (in_left->calib_model() == CalibrationModel::PINHOLE)
    {//由基类指针转换为指定类型指针
        in_left = std::dynamic_pointer_cast<IntrinsicsPinhole>(in_left);
        in_right = std::dynamic_pointer_cast<IntrinsicsPinhole>(in_right);
    }
    else if (in_left->calib_model() == CalibrationModel::KANNALA_BRANDT)
    {
        in_left = std::dynamic_pointer_cast<IntrinsicsEquidistant>(in_left);
        in_right = std::dynamic_pointer_cast<IntrinsicsEquidistant>(in_right);
    }
    else
    {
        LOG(INFO) << "UNKNOW CALIB MODEL (未知校正模型) .";
        return 0;
    }
    api->EnableStreamData(Stream::LEFT);
    api->EnableStreamData(Stream::RIGHT);
    api->Start(Source::VIDEO_STREAMING);

    thread tViewer = thread(&ORB_SLAM2::ViewerAR::Run,&viewerAR);

    while (flag)
    {
        api->WaitForStreams();
        auto &&left_data = api->GetStreamData(Stream::LEFT);
        auto &&right_data = api->GetStreamData(Stream::RIGHT);

        igb.GrabStereo(left_data.frame, right_data.frame, left_data.img->timestamp*0.00001f);
    }
   
    SLAM.Shutdown();
    api->Stop(Source::ALL);

    return 0;
}

void ImageGrabber::GrabStereo(const cv::Mat& left_img, const cv::Mat& right_img, double timeStamp)
{
    if (left_img.empty() || right_img.empty())
    {
	    assert("left_img == empty || right_img == empty");
    }

    if (do_rectify)
    {
    	cv::Mat _left_img, _right_img;
        cv::remap(left_img, _left_img, M1l, M2l, cv::INTER_LINEAR);
        cv::remap(right_img, _right_img, M1r, M2r, cv::INTER_LINEAR);

        cv::Mat im = left_img.clone();
        cv::Mat imu;
        int a;
        cv::Mat Tcw = mpSLAM->TrackStereo(_left_img,_right_img,timeStamp);
        int state = mpSLAM->GetTrackingState();
        vector<ORB_SLAM2::MapPoint*> vMPs = mpSLAM->GetTrackedMapPoints();
        vector<cv::KeyPoint> vKeys = mpSLAM->GetTrackedKeyPointsUn();
        vector<size_t > vIndices = mpSLAM->GetTrackedIndices();
        cv::Mat Tmc = mpSLAM->GetMarkerPose();
        //对im去畸变 保存进imu中
        cv::undistort(im,imu,K,DistCoef);
        //调用viewerAR的SetImagePose函数将slam系统提供的数据传进AR中
        if(bRGB)
        {
            viewerAR.SetImagePose(imu,Tcw,state,vKeys,vMPs,Tmc,a,vIndices);
        }
        else
        {
            cv::cvtColor(imu,imu,CV_RGB2BGR);
            viewerAR.SetImagePose(imu,Tcw,state,vKeys,vMPs,Tmc,a,vIndices);
        }
    }
    else
    {
      mpSLAM->TrackStereo(left_img, right_img, timeStamp);
    }
}
