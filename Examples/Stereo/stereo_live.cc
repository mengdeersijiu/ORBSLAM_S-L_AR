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
#include<iomanip>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>
using namespace std::chrono;
using namespace std;

cv::Mat image_split(cv::Mat img,CvRect rect);
cv::Mat image_processL(cv::Mat img);
cv::Mat image_processR(cv::Mat img);

int main(int argc, char **argv)
{
    // Retrieve paths to images
   // vector<string> vstrImageLeft;
   // vector<string> vstrImageRight;
    vector<double> vTimeStamp;

    // Read rectification parameters
//    cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
//    if(!fsSettings.isOpened())
//    {
//        cerr << "ERROR: Wrong path to settings" << endl;
//        return -1;
//    }
//
//    cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
//    fsSettings["LEFT.K"] >> K_l;
//    fsSettings["RIGHT.K"] >> K_r;
//
//    fsSettings["LEFT.P"] >> P_l;
//    fsSettings["RIGHT.P"] >> P_r;
//
//    fsSettings["LEFT.R"] >> R_l;
//    fsSettings["RIGHT.R"] >> R_r;
//
//    fsSettings["LEFT.D"] >> D_l;
//    fsSettings["RIGHT.D"] >> D_r;
//
//    int rows_l = fsSettings["LEFT.height"];
//    int cols_l = fsSettings["LEFT.width"];
//    int rows_r = fsSettings["RIGHT.height"];
//    int cols_r = fsSettings["RIGHT.width"];
//
//    if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
//            rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
//    {
//        cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
//        return -1;
//    }
//
//    cv::Mat M1l,M2l,M1r,M2r;
//    cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
//    cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);
//

   // const int nImages = vstrImageLeft.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    cout << endl << "-------" << endl;
    cout << "Start processing camera ..." << endl;

    cv::Mat imageALL, imLeft, imRight, imLeftRect, imRightRect;

    cv::VideoCapture cap1(0);
    cap1.set(CV_CAP_PROP_FRAME_WIDTH,2560);
    cap1.set(CV_CAP_PROP_FRAME_HEIGHT,960);
    cap1.set(CV_CAP_PROP_FPS, 30);
    
//    cv::VideoCapture cap2(1);
//    cap2.set(CV_CAP_PROP_FRAME_WIDTH,1920);
//    cap2.set(CV_CAP_PROP_FRAME_HEIGHT,1080);
//    cap2.set(CV_CAP_PROP_FPS, 30);

    long int nImages = 0;
    int ni=0;
// Main loop
    while(ni>-1)
    {
        cap1 >> imageALL;

        if(imageALL.empty())
        {
            cerr << endl << "Check Camera!! "<< endl;
            return 1;
        }

        imLeft = image_processL(imageALL);
        imRight = image_processR(imageALL);

//        cv::remap(imLeft,imLeftRect,M1l,M2l,cv::INTER_LINEAR);
//        cv::remap(imRight,imRightRect,M1r,M2r,cv::INTER_LINEAR);

        time_point<system_clock> now = system_clock::now();
        
        double tframe = now.time_since_epoch().count();
        vTimeStamp.push_back(tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the images to the SLAM system
        //SLAM.TrackStereo(imLeftRect,imRightRect,tframe);
        SLAM.TrackStereoOriginalIm(imLeft,imRight,tframe);
        vector<size_t > vIndices = SLAM.GetTrackedIndices();



#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
       
        vTimesTrack.push_back(ttrack);

        // Wait to load the next frame
/*        
	double T=0;
        if(ni<nImages-1)
            T = vTimeStamp[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimeStamp[ni-1];

       if(ttrack<T)
            usleep((T-ttrack)*1e6);
*/
	nImages++;
	//std::cout << "stereoFrame : "<<nImages<< std::endl;
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");

    return 0;
}

    
//***********************************************************************8   

cv::Mat image_split(cv::Mat img,CvRect rect)
{    // 获取图像
    int crop_x1 = cv::max(0,rect.x);
    int crop_y1 = cv::max(0,rect.y);
    // 图像范围 0到cols-1, 0到rows-1
    int crop_x2 = cv::min(img.cols - 1, rect.x + rect.width - 1);
    int crop_y2 = cv::min(img.rows - 1, rect.y + rect.height - 1);
    // 左包含，右不包含
    return img(cv::Range(crop_y1,crop_y2 + 1),cv::Range(crop_x1,crop_x2 + 1));
}

cv::Mat image_processL(cv::Mat img)
{
    CvRect rect;
    rect.x = 0;
    rect.y = 0;
    rect.width = img.cols/2;
    rect.height = img.rows;
    //cv::Mat I = cv::Mat::zeros(rect.height, rect.width, 0);// 目标图像
    cv::Mat roi_imgL=image_split(img,rect);
    cv::Mat imgDstL;
    cv::resize(roi_imgL, imgDstL, cv::Size(640,480));

    //cv::waitKey(5);
    return imgDstL.clone();
}

cv::Mat image_processR(cv::Mat img)
{
    CvRect rect;
    rect.x = img.cols/2;
    rect.y = 0;
    rect.width = img.cols/2;
    rect.height = img.rows;

    cv::Mat roi_imgR=image_split(img,rect);
    cv::Mat imgDstR;
    cv::resize(roi_imgR, imgDstR, cv::Size(640,480));

//    cv::imshow(OUTPUT_RIGHT, roi_img);
    //cv::waitKey(5);
    return imgDstR.clone();
}