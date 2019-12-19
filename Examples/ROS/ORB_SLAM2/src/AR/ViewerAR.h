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


#ifndef VIEWERAR_H
#define VIEWERAR_H

#include <mutex>
#include <opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include <pangolin/pangolin.h>
//#include <pangolin/geometry/geometry_obj.h>
//#include <pangolin/geometry/geometry_obj.h>
#include <string>
#include"../../../include/System.h"
#include "keyboard.h"

#include <boost/serialization/vector.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/base_object.hpp>

namespace ORB_SLAM2
{



class Plane
{
public:
    Plane(const std::vector<MapPoint*> &vMPs, const cv::Mat &Tcw);
    Plane(const float &nx, const float &ny, const float &nz, const float &ox, const float &oy, const float &oz);
    Plane(const cv::Mat &T);

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

class ViewerAR
{
public:
    ViewerAR();

    void SetFPS(const float fps){
        mFPS = fps;
        mT=1e3/fps;
    }

    void SetSLAM(ORB_SLAM2::System* pSystem){
        mpSystem = pSystem;
    }

    // Main thread function. 
    void Run();

    void SetCameraCalibration(const float &fx_, const float &fy_, const float &cx_, const float &cy_){
        fx = fx_; fy = fy_; cx = cx_; cy = cy_;
    }

    void SetImagePose(const cv::Mat &im, const cv::Mat &Tcw, const int &status, const std::vector<cv::KeyPoint> &vKeys,
		      const std::vector<MapPoint*> &vMPs, const cv::Mat &Tcm, const int& a, const vector<size_t > &vIndices);

    void SetImagePose(const cv::Mat &im, const cv::Mat &Tcw, const int &status, const std::vector<cv::KeyPoint> &vKeys,
                      const std::vector<MapPoint*> &vMPs, const cv::Mat &Tcm, const int& a);

    void GetImagePose(cv::Mat &im, cv::Mat &Tcw, int &status, std::vector<cv::KeyPoint> &vKeys,
		      std::vector<MapPoint*> &vMPs, cv::Mat &Tcm, int& a, vector<size_t > &vIndices);

    void GetImagePose(cv::Mat &im, cv::Mat &Tcw, int &status, std::vector<cv::KeyPoint> &vKeys,
                      std::vector<MapPoint*> &vMPs, cv::Mat &Tcm, int& a);
    //增加的部分：传递有矩阵信息的yaml文件路径***********************************************************************************
    void SetTfixedFilePath(const string &sTfixedPath, const string &sTwc1Path);
    void GetTfixedFilePath(string &sTfixedPath, string &sTwc1Path);
    void SetTpwPath(const string &sTpwPath);
    void GetTpwPath(string &sTpwPath);
    //mat类型转换到openglmatrix类型
    pangolin::OpenGlMatrix TransformToGLMatrix(const cv::Mat &T);
    //判断是否为数字
    bool IsNumber(const string &s);

    bool SaveARPos(const string &sTpwPath);
    bool LoadARPos(const string &sTpwPath);

    //********************************************************************************************************************
private:
    //SLAM
    ORB_SLAM2::System* mpSystem;

    void PrintStatus(const int &status, const bool &bLocMode, cv::Mat &im);
    void AddTextToImage(const std::string &s, cv::Mat &im, const int r=0, const int g=0, const int b=0);
    void LoadCameraPose(const cv::Mat &Tcw);
    void DrawImageTexture(pangolin::GlTexture &imageTexture, cv::Mat &im);
    void DrawArea(cv::Mat &im);
    void DrawCube(const float &size, const float x=0.0, const float y=0.0, const float z=0.0);
    void DrawPlane(int ndivs, float ndivsize);
    void DrawPlane(Plane* pPlane, int ndivs, float ndivsize);
    void DrawTrackedPoints(const std::vector<cv::KeyPoint> &vKeys, const std::vector<MapPoint*> &vMPs, cv::Mat &im);

    Plane* DetectPlane(const cv::Mat Tcw, const std::vector<MapPoint*> &vMPs, const int iterations=50);

    // frame rate
    float mFPS, mT;
    float fx,fy,cx,cy;

    // Last processed image and computed pose by the SLAM
    std::mutex mMutexPoseImage;

    cv::Mat mTcw;
    cv::Mat mImage;
    int mStatus;
    std::vector<cv::KeyPoint> mvKeys;
    std::vector<MapPoint*> mvMPs;
    //增加的部分：*****************************************************************************************************
    std::vector<size_t > mvIndices;
    std::vector<MapPoint*> mvMPsInArea;
    std::vector<Plane*> mvpPlane = std::vector<Plane*>(100,static_cast<Plane*>(NULL));
    std::vector<bool> mvbDraw = std::vector<bool>(100,true);
    std::vector<cv::Mat> mvTpw = std::vector<cv::Mat>(100);
    cv::Mat mTpw;
    
    int mna;
    //marker到相机的变换
    cv::Mat mTcm;
//    //marker到绝对坐标系的变换
//    cv::Mat mTwm;
//    //marker到相机的变换
//    pangolin::OpenGlMatrix glTcm;
//    //marker到绝对坐标系的变换
//    pangolin::OpenGlMatrix glTwm;
//    //相机到绝对坐标系
//    pangolin::OpenGlMatrix glTwc1;
    
//    pangolin::OpenGlMatrix glT;
    
    //有矩阵信息的yaml文件路径
    string msTfixedPath;
    string msTcw1Path;
    string msTpwPath;
    
//    cv::Mat Tcm1 = cv::Mat::eye(4, 4, CV_32F);
//    cv::Mat Tcw1 = cv::Mat::eye(4, 4, CV_32F);
//    cv::Mat Twm1 = cv::Mat::eye(4, 4, CV_32F);

    int mnumber;

    //**************************************************************************************************************
};


}


#endif // VIEWERAR_H
	

