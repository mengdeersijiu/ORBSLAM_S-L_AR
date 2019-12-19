/**
* This file is part of ORB-SLAM2.
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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"
#include"ViewerAR.h"
#include"keyboard.h"

using namespace std;

ORB_SLAM2::ViewerAR viewerAR;
ORB_SLAM2::keyboard keyboard1;
bool bRGB = true;

cv::Mat K;
cv::Mat DistCoef;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);

    ORB_SLAM2::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Stereo path_to_vocabulary path_to_settings do_rectify" << endl;
        ros::shutdown();
        return 1;
    }
    cout<<argv[1]<<endl;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,false);
    

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

    stringstream ss(argv[3]);
	ss >> boolalpha >> igb.do_rectify;

    if(igb.do_rectify)
    {
        // Load settings related to stereo calibration
      cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
      if(!fsSettings.isOpened())
      {
	    cerr << "ERROR: Wrong path to settings" << endl;
	    return -1;
      }
      
      bRGB = static_cast<bool>((int)fsSettings["Camera.RGB"]);
      
      cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
      
      float fps = fsSettings["Camera.fps"];
      viewerAR.SetFPS(fps);

      float fx = fsSettings["Camera.fx"];
      float fy = fsSettings["Camera.fy"];
      float cx = fsSettings["Camera.cx"];
      float cy = fsSettings["Camera.cy"];

      viewerAR.SetCameraCalibration(fx,fy,cx,cy);

      K = cv::Mat::eye(3,3,CV_32F);
      K.at<float>(0,0) = fx;
      K.at<float>(1,1) = fy;
      K.at<float>(0,2) = cx;
      K.at<float>(1,2) = cy;

      DistCoef = cv::Mat::zeros(4,1,CV_32F);
      DistCoef.at<float>(0) = fsSettings["Camera.k1"];
      DistCoef.at<float>(1) = fsSettings["Camera.k2"];
      DistCoef.at<float>(2) = fsSettings["Camera.p1"];
      DistCoef.at<float>(3) = fsSettings["Camera.p2"];
      const float k3 = fsSettings["Camera.k3"];
      if(k3!=0)
      {
	    DistCoef.resize(5);
	    DistCoef.at<float>(4) = k3;
      }
     
      
      fsSettings["LEFT.K"] >> K_l;
      fsSettings["RIGHT.K"] >> K_r;

      fsSettings["LEFT.P"] >> P_l;
      fsSettings["RIGHT.P"] >> P_r;

      fsSettings["LEFT.R"] >> R_l;
      fsSettings["RIGHT.R"] >> R_r;

      fsSettings["LEFT.D"] >> D_l;
      fsSettings["RIGHT.D"] >> D_r;

      int rows_l = fsSettings["LEFT.height"];
      int cols_l = fsSettings["LEFT.width"];
      int rows_r = fsSettings["RIGHT.height"];
      int cols_r = fsSettings["RIGHT.width"];

      if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
	      rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
      {
	    cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
	    return -1;
      }

      cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
      cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
      
      string sTpwPath = static_cast<string>((string)fsSettings["Tpw.path"]);
      viewerAR.SetTpwPath(sTpwPath);

      fsSettings.release();
    }


    
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/camera/image/lefteye", 1);///mynteye/left/image_raw  /camera/image/lefteye   /usb_cam0/image_raw
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/camera/image/righteye", 1);///mynteye/right/image_raw  /camera/image/righteye   /usb_cam1/image_raw
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));
        
    thread tViewer = thread(&ORB_SLAM2::ViewerAR::Run,&viewerAR);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    //SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    //SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    //SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    //do_rectify为true时会对
    if(do_rectify)
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);

        //从slam系统中获取im Tcw state vMPs vKeys这些数据
        cv::Mat im = cv_ptrLeft->image.clone();
        cv::Mat imu;
    //	int a = keyboard1.mna;
        int a = 1;
        cv::Mat Tcw = mpSLAM->TrackStereoOriginalIm(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
        int state = mpSLAM->GetTrackingState();
        vector<ORB_SLAM2::MapPoint*> vMPs = mpSLAM->GetTrackedMapPoints();
        vector<cv::KeyPoint> vKeys = mpSLAM->GetTrackedKeyPointsUn();
        vector<size_t > vIndices = mpSLAM->GetTrackedIndices();
        cv::Mat Tmc = mpSLAM->GetMarkerPose();
        //cv::Mat Tpw = mpSLAM->GetPlaneMatrix();
        //对im去畸变 保存进imu中
        cv::undistort(im,imu,K,DistCoef);
            //调用viewerAR的SetImagePose函数将slam系统提供的数据传进AR中
        if(bRGB)
            viewerAR.SetImagePose(imu,Tcw,state,vKeys,vMPs,Tmc,a,vIndices);
        else
        {
            cv::cvtColor(imu,imu,CV_RGB2BGR);
            viewerAR.SetImagePose(imu,Tcw,state,vKeys,vMPs,Tmc,a,vIndices);
        }
      
    }
    else
    {
        mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
    }

	
}

