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


#ifndef VIEWER_H
#define VIEWER_H

#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Tracking.h"
#include "System.h"
#include <string>
#include <mutex>

namespace ORB_SLAM2
{

class Tracking;
class FrameDrawer;
class MapDrawer;
class System;

class Plane1
{
public:
    Plane1(const std::vector<MapPoint*> &vMPs, const cv::Mat &Tcw);
    Plane1(const float &nx, const float &ny, const float &nz, const float &ox, const float &oy, const float &oz);

    void Recompute();

    //normal
    cv::Mat n;
    //origin
    cv::Mat o;
    //arbitrary orientation along normal
    float rang;
    //transformation from world to the plane
    cv::Mat Tpw;
    //cv::Mat Twm;
    pangolin::OpenGlMatrix glTpw;
    
    //MapPoints that define the plane
    std::vector<MapPoint*> mvMPs;
    //camera pose when the plane was first observed (to compute normal direction)
    cv::Mat mTcw, XC;

};

class Viewer
{
public:

    Viewer();
    //增加的部分：构造函数增加地图加载相关参数**************************************************************************************************************
    Viewer(System* pSystem, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Tracking* pTracking, const string &strSettingPath, bool mbReuseMap);
    //***********************************************************************************************************************************************

    // Main thread function. Draw points, keyframes, the current camera pose and the last processed
    // frame. Drawing is refreshed according to the camera fps. We use Pangolin.
    void Run();

    void RequestFinish();

    void RequestStop();

    bool isFinished();

    bool isStopped();

    void Release();
    //Pause功能
    bool isPaused();
    
    void SetPause();
    
    void UnsetPause();
    
    void LoadCameraPose(const pangolin::OpenGlMatrix &Twc);
    
    //增加的部分：传递有矩阵信息的yaml文件路径***********************************************************************************

    void DrawImageTexture(pangolin::GlTexture &imageTexture, cv::Mat &im);
    
    void SetImagePose(const cv::Mat &im, const cv::Mat &Tcw, const int &status,
                      const std::vector<cv::KeyPoint> &vKeys, const std::vector<MapPoint*> &vMPs, const cv::Mat &Tcm);

    void GetImagePose(cv::Mat &im, cv::Mat &Tcw, int &status,
                      std::vector<cv::KeyPoint> &vKeys,  std::vector<MapPoint*> &vMPs, cv::Mat &Tcm);

    void DrawMapPointsInArea(const vector<MapPoint*> &vMPsInArea);
    //********************************************************************************************************************

private:

    Plane1* DetectPlane(const cv::Mat Tcw, const std::vector<MapPoint*> &vMPs, const int iterations=50);
    bool Stop();

    System* mpSystem;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;
    Tracking* mpTracker;

    // 1/fps in ms
    double mT;
    float mImageWidth, mImageHeight;

    float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;
    
    float fx,fy,cx,cy;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;
    //增加的部分：暂停功能相关变量*****************************************************************************
    bool mbPaused;
    std::mutex mMutexPause;
    //****************************************************************************************************
    bool mbStopped;
    bool mbStopRequested;
    
    bool mbReuseMap;

    std::mutex mMutexStop;
    
    //有矩阵信息的yaml文件路径
    string msTwmPath;
    string msTcwPath;
    //相机到绝对坐标系
    pangolin::OpenGlMatrix glTwc;

    std::mutex mMutexPoseImage;
    cv::Mat mTcw;
    cv::Mat mImage;
    int mStatus;
    std::vector<cv::KeyPoint> mvKeys;
    std::vector<MapPoint*> mvMPs;
    std::vector<MapPoint*> mvMPsInArea;
    cv::Mat mTcm;
    int mtest =1;
};



}


#endif // VIEWER_H
	

