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


#ifndef SYSTEM_H
#define SYSTEM_H

#include<string>
#include<thread>
#include<opencv2/core/core.hpp>
#include<condition_variable>

#include "Tracking.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
#include "Viewer.h"

#include "BoostArchiver.h"
// for map file io
#include <fstream>



namespace ORB_SLAM2
{

class Viewer;
class FrameDrawer;
class Map;
class Tracking;
class LocalMapping;
class LoopClosing;

const float EPS = 1e-4;

class System
{
public:
    // Input sensor
    enum eSensor{
        MONOCULAR=0,
        STEREO=1,
        RGBD=2
    };

public:

    // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.
    //增加的部分：构造函数增加了地图相关参数**********************************************************************************************************
    System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor, const bool bUseViewer = true, bool is_save_map_=false);
    //*****************************************************************************************************************************************
    
    // Proccess the given stereo frame. Images must be synchronized and rectified.
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    cv::Mat TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp);

    cv::Mat TrackStereoOriginalIm(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp);

    // Process the given rgbd frame. Depthmap must be registered to the RGB frame.
    // Input image: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Input depthmap: Float (CV_32F).
    // Returns the camera pose (empty if tracking fails).
    cv::Mat TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp);

    // Proccess the given monocular frame
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    cv::Mat TrackMonocular(const cv::Mat &im, const double &timestamp);

    // This stops local mapping thread (map building) and performs only camera tracking.
    void ActivateLocalizationMode();
    // This resumes local mapping thread and performs SLAM again.
    void DeactivateLocalizationMode();

    // Returns true if there have been a big map change (loop closure, global BA)
    // since last call to this function
    bool MapChanged();
    
    //Pause and resume mechanism
    //增加的部分：暂停功能*******************************************************************************
    void Pause();
    void Resume();
    //*********************************************************************************************
    //增加的部分：可以通过此函数启动aruco****************************************************************
    cv::Mat arucotrack();
    //**********************************************************************************************
    // Reset the system (clear map)
    void Reset();

    // All threads will be requested to finish.
    // It waits until all threads have finished.
    // This function must be called before saving the trajectory.
    void Shutdown();

    // Save camera trajectory in the TUM RGB-D dataset format.
    // Only for stereo and RGB-D. This method does not work for monocular.
    // Call first Shutdown()
    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    void SaveTrajectoryTUM(const string &filename);

    // Save keyframe poses in the TUM RGB-D dataset format.
    // This method works for all sensor input.
    // Call first Shutdown()
    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    void SaveKeyFrameTrajectoryTUM(const string &filename);

    // Save camera trajectory in the KITTI dataset format.
    // Only for stereo and RGB-D. This method does not work for monocular.
    // Call first Shutdown()
    // See format details at: http://www.cvlibs.net/datasets/kitti/eval_odometry.php
    void SaveTrajectoryKITTI(const string &filename);
    // Information from most recent processed frame
    // You can call this right after TrackMonocular (or stereo or RGBD)
    int GetTrackingState();
    std::vector<MapPoint*> GetTrackedMapPoints();
    std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();
    //std::vector<MapPoint*> GetTrackedMapPointsInArea();
    std::vector<size_t > GetTrackedIndices();
    cv::Mat GetPlaneMatrix();
    bool GetActivateLocalizationMode();

    //保存Map功能
    void SaveMap();

    // 此函数可以得到marker位姿
    cv::Mat GetMarkerPose();

    //调用此函数触发mbDetectPlane = true
    void PlaneSwitch();
    //更新mvMPsInArea和mvInlierMP并调用Recompute函数计算出Tpw
    cv::Mat TrackedPlaneCompute();
    //更新mvMPsInArea
    bool FillMpsTnArea(const std::vector<MapPoint*> &vMPsInArea);
    //更新mvInlierMP
    bool RANSACMPS(const std::vector<MapPoint*> &vMPs, const int iterations=50);

    //计算出Tpw
    cv::Mat Recompute();
    cv::Mat Recompute1();
    cv::Mat Recompute2();

    cv::Mat EXPSO3(const float &x, const float &y, const float &z);
    cv::Mat EXPSO3(const cv::Mat &v);

private:
// 增加的部分：地图保存与加载*************************************************************************
    void SaveMap(const string &filename);
    
    bool LoadMap(const string &filename);
//***************************************************************************************************
private:

    // Input sensor
    eSensor mSensor;

    // ORB vocabulary used for place recognition and feature matching.
    ORBVocabulary* mpVocabulary;

    // KeyFrame database for place recognition (relocalization and loop detection).
    KeyFrameDatabase* mpKeyFrameDatabase;

    // Map structure that stores the pointers to all KeyFrames and MapPoints.
    Map* mpMap;

    //增加的部分：地图保存与加载相关变量****************************************************************
    string mapfile;
    bool is_save_map;
    //*******************************************************************************************
    
    // Tracker. It receives a frame and computes the associated camera pose.
    // It also decides when to insert a new keyframe, create some new MapPoints and
    // performs relocalization if tracking fails.
    Tracking * mpTracker;

    // Local Mapper. It manages the local map and performs local bundle adjustment.
    LocalMapping* mpLocalMapper;

    // Loop Closer. It searches loops with every new keyframe. If there is a loop it performs
    // a pose graph optimization and full bundle adjustment (in a new thread) afterwards.
    LoopClosing* mpLoopCloser;

    // The viewer draws the map and the current camera pose. It uses Pangolin.
    Viewer* mpViewer;

    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;

    // System threads: Local Mapping, Loop Closing, Viewer.
    // The Tracking thread "lives" in the main execution thread that creates the System object.
    std::thread* mptLocalMapping;
    std::thread* mptLoopClosing;
    std::thread* mptViewer;
    
    //增加的部分：暂停功能flag
    std::mutex mMutexPause;
    condition_variable mCvResume;
    bool mbPause;
    bool mbaruco;

    // Reset flag
    std::mutex mMutexReset;
    bool mbReset;

    // Change mode flags
    std::mutex mMutexMode;
    bool mbActivateLocalizationMode;
    bool mbDeactivateLocalizationMode;

    // Tracking state
    int mTrackingState;
    std::vector<MapPoint*> mTrackedMapPoints;
    std::vector<cv::KeyPoint> mTrackedKeyPointsUn;
    //std::vector<MapPoint*> mTrackedMapPointsInArea;
    //std::vector<cv::KeyPoint> mTrackedKeyPointsUnInArea;
    std::vector<size_t> mvIndices;


    //State flag
    std::mutex mMutexState;

    //2019/11月新增加的
    std::mutex mMutexDetectPlane;//detectPlane flag
    bool mbDetectPlane = false;//DetectPlane flag
    string mstrSettingsFile;
    cv::Mat M1l, M2l, M1r, M2r;
    cv::Mat mTpw;
    std::vector<MapPoint*> mvInlierMPsInArea;//经RANSAC后当前帧中间区域对应的地图点
    std::vector<MapPoint*> mvMPsInArea;//当前帧中间区域对应的地图点
    std::vector<MapPoint*> mvInlierMPs;//经RANSAC后的当前帧对应的地图点

    //normal
    cv::Mat n;
    //origin
    cv::Mat o;
    //arbitrary orientation along normal
    float rang = 1.0f;
    //camera pose when the plane was first observed (to compute normal direction)
    cv::Mat mTcw, XC;

};

}// namespace ORB_SLAM

#endif // SYSTEM_H
