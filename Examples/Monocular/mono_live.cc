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

#include <iostream>
#include <fstream>
#include <algorithm>
#include <cassert>
#include <opencv2/opencv.hpp>
#include "../../include/System.h"
#include "../../include/Viewer.h"
using namespace std::chrono;
using namespace std;
//ORB_SLAM2::Viewer viewer;

//class ImageGrabber
//{
//public:
//    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}
//
//    void GrabImage(const cv::Mat& left_img, double timeStamp);
//
//    ORB_SLAM2::System* mpSLAM;
//};

int main(int argc, char **argv)
{
    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;
         return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);
    //ImageGrabber igb(&SLAM);

    cv::Mat imLeft;
    cv::VideoCapture cap1(0);
    cap1.set(CV_CAP_PROP_FRAME_WIDTH,1280);
    cap1.set(CV_CAP_PROP_FRAME_HEIGHT,720);
    cap1.set(CV_CAP_PROP_FPS, 30);

    int ni=0;
    while(ni>-1)
    {
        cap1 >> imLeft;
        if(imLeft.empty())
        {
            cerr << endl << "Check Left Camera!! "<< endl;
            return 1;
        }
        time_point<system_clock> now = system_clock::now();
        double tframe = now.time_since_epoch().count();
        // Pass the images to the SLAM system
        SLAM.TrackMonocular(imLeft, tframe);
    }
    // Stop all threads
    SLAM.Shutdown();
    return 0;
}

//void ImageGrabber::GrabImage(const cv::Mat& left_img, double timeStamp)
//{
//    cv::Mat Tcw = mpSLAM->TrackMonocular(left_img, timeStamp);
//    cv::Mat im = left_img.clone();
//    int state = mpSLAM->GetTrackingState();
//    vector<ORB_SLAM2::MapPoint*> vMPs = mpSLAM->GetTrackedMapPointsInArea();
//    vector<cv::KeyPoint> vKeys = mpSLAM->GetTrackedKeyPointsUn();
//    cv::Mat Tcm = mpSLAM->GetMarkerPose();
//    viewer.SetImagePose(im,Tcw,state,vKeys,vMPs,Tcm);
//}

