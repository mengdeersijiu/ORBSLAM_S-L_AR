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



#include "System.h"
#include "Converter.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>

static bool has_suffix(const std::string &str, const std::string &suffix)
{
    std::size_t index = str.find(suffix, str.size() - suffix.size());
    return (index != std::string::npos);
}

namespace ORB_SLAM2
{

  System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
                const bool bUseViewer, bool is_save_map_):mSensor(sensor), is_save_map(is_save_map_), mpViewer(static_cast<Viewer*>(NULL)), mbPause(false), mbaruco(false), mbReset(false),
         mbActivateLocalizationMode(false), mbDeactivateLocalizationMode(false)
{
    // Output welcome message
    cout << endl <<
    "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl <<
    "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
    "This is free software, and you are welcome to redistribute it" << endl <<
    "under certain conditions. See LICENSE.txt." << endl << endl;

    cout << "Input sensor was set to: ";

    if(mSensor==MONOCULAR)
        cout << "Monocular" << endl;
    else if(mSensor==STEREO)
        cout << "Stereo" << endl;
    else if(mSensor==RGBD)
        cout << "RGB-D" << endl;

    mstrSettingsFile = strSettingsFile;

    //Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       exit(-1);
    }

    cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;

    fsSettings["LEFT.K"] >> K_l;
    fsSettings["RIGHT.K"] >>  K_r;

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
       rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)	{
        std::cerr << "error: calibration parameters to rectify stereo are missing!" << std::endl;
    }

    cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
    cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);

    cv::FileNode mapfilen = fsSettings["Map.mapfile"];
    bool bReuseMap = false;
    if (!mapfilen.empty())
    {
        mapfile = (string)mapfilen;
    }

    fsSettings.release();
    //******************************************************************************************************
    
    //增加的部分：加载词袋文件可以用bin或txt格式加载***************************************************************
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;
    mpVocabulary = new ORBVocabulary();
    bool bVocLoad = false; // chose loading method based on file extension
    if (has_suffix(strVocFile, ".txt"))
        bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    else if(has_suffix(strVocFile, ".bin"))
        bVocLoad = mpVocabulary->loadFromBinaryFile(strVocFile);
    else
        bVocLoad = false;
    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Falied to open at: " << strVocFile << endl;
        exit(-1);
    }
    cout << "Vocabulary loaded!" << endl << endl;
    //******************************************************************************************************

    //Create KeyFrame Database
    //Create the Map
    if (!mapfile.empty() && LoadMap(mapfile))
    {
        bReuseMap = true;
    }
    else
    {
        mpKeyFrameDatabase = new KeyFrameDatabase(mpVocabulary);
        mpMap = new Map();
    }

    //Create Drawers. These are used by the Viewer
    mpFrameDrawer = new FrameDrawer(mpMap, bReuseMap);
    mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);

    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                             mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor, bReuseMap);

    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(mpMap, mSensor==MONOCULAR);
    mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run,mpLocalMapper);

    //Initialize the Loop Closing thread and launch
    mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor!=MONOCULAR);
    mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);

    //Initialize the Viewer thread and launch
    if(bUseViewer)
    {
        mpViewer = new Viewer(this, mpFrameDrawer,mpMapDrawer,mpTracker,strSettingsFile, bReuseMap);
        mptViewer = new thread(&Viewer::Run, mpViewer);
        mpTracker->SetViewer(mpViewer);
    }

    //Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);
}

cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp)
{
    if(mSensor!=STEREO)
    {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                std::this_thread::sleep_for(std::chrono::microseconds(1000));
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }
    
    
    // Check pause
    {
        unique_lock<mutex> lock(mMutexPause);
        if(mbPause)
        {
            mCvResume.wait(lock);
            mbPause = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
        }
    }

    cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft,imRight,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    mvIndices = mpTracker->mvIndices;

    //Check DetectPlane
    {
        unique_lock<mutex> lock(mMutexDetectPlane);
        if(mbDetectPlane)
        {
            //rang = -3.14f/2+((float)rand()/RAND_MAX)*3.14f;
            cv::Mat tmpTpw = TrackedPlaneCompute();
            if(!tmpTpw.empty())
            {
                mTpw = tmpTpw.clone();
            }
            else
            {
                mTpw = cv::Mat::eye(4,4,CV_32F);
            }
            mbDetectPlane = false;
        }
    }

    return Tcw;
}

cv::Mat System::TrackStereoOriginalIm(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp)
{
    if(mSensor!=STEREO)
    {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                std::this_thread::sleep_for(std::chrono::microseconds(1000));
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check pause
    {
        unique_lock<mutex> lock(mMutexPause);
        if(mbPause)
        {
            mCvResume.wait(lock);
            mbPause = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
        }
    }

    cv::Mat _left_img, _right_img;
    cv::remap(imLeft, _left_img, M1l, M2l, cv::INTER_LINEAR);
    cv::remap(imRight, _right_img, M1r, M2r, cv::INTER_LINEAR);

    cv::Mat Tcw = mpTracker->GrabImageStereo(_left_img,_right_img,timestamp);
    mTcw = Tcw.clone();

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    mvIndices = mpTracker->mvIndices;

    //Check DetectPlane
    {
        unique_lock<mutex> lock(mMutexDetectPlane);
        if(mbDetectPlane)
        {
            //rang = -3.14f/2+((float)rand()/RAND_MAX)*3.14f;
            cv::Mat tmpTpw = TrackedPlaneCompute();
            if(!tmpTpw.empty())
            {
                mTpw = tmpTpw.clone();
            }
            else
            {
                mTpw = cv::Mat::eye(4,4,CV_32F);
            }
            mbDetectPlane = false;
        }
    }

    return Tcw;
}

cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp)
{
    if(mSensor!=RGBD)
    {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                std::this_thread::sleep_for(std::chrono::microseconds(1000));
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    
    // Check pause
    {
    unique_lock<mutex> lock(mMutexPause);
    if(mbPause)
    {
        mCvResume.wait(lock);
        mbPause = false;
    }
    }

    
    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageRGBD(im,depthmap,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    mvIndices = mpTracker->mvIndices;
    return Tcw;
}

cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp)
{
    if(mSensor!=MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                std::this_thread::sleep_for(std::chrono::microseconds(1000));
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    
    // Check pause
    {
        unique_lock<mutex> lock(mMutexPause);
        if(mbPause)
        {
            mCvResume.wait(lock);
            mbPause = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
        }
    }

    cv::Mat Tcw = mpTracker->GrabImageMonocular(im,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    mvIndices = mpTracker->mvIndices;

    return Tcw;
}

void System::ActivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

bool System::MapChanged()
{
    static int n=0;
    int curn = mpMap->GetLastBigChangeIdx();
    if(n<curn)
    {
        n=curn;
        return true;
    }
    else
        return false;
}
//增加的部分：暂停函数***************************************************************************************************************************
void System::Pause()
{
    unique_lock<mutex> lock(mMutexPause);
    mbPause = true;
    cout << "Pause" << endl;
}

void System::Resume()
{
    unique_lock<mutex> lock(mMutexPause);
    mbPause = false;
    mCvResume.notify_one();
    cout << "Resume" << endl;
}

void System::PlaneSwitch()
{
    unique_lock<mutex> lock(mMutexDetectPlane);
    mbDetectPlane = true;
    cout<<"DetectPlane"<<endl;
}

//**************************************************************************************************************************************

//增加的部分：能通过此函数启动aruco*****************************************************************************************************
cv::Mat System::arucotrack()
{
    cv::Mat ima;
    mpTracker->mImGray.copyTo(ima);
    //ima = mpTracker->mImGray;
    cout << "in System::arucotrack" << endl;
    cv::Mat Tmc = mpTracker->GetposefromMark(ima);
    return Tmc;
}

//****************************************************************************************************************************
void System::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

void System::Shutdown()
{
    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();
    if(mpViewer)
    {
        mpViewer->RequestFinish();
        while(!mpViewer->isFinished())
        {
            std::this_thread::sleep_for(std::chrono::microseconds(5000));
        }
    }

    // Wait until all thread have effectively stopped
    while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
    {
        std::this_thread::sleep_for(std::chrono::microseconds(5000));
    }
    if(mpViewer)
        pangolin::BindToContext("ORB-SLAM2: Map Viewer");
}

void System::SaveTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        while(pKF->isBad())
        {
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        vector<float> q = Converter::toQuaternion(Rwc);

        f << setprecision(6) << *lT << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}


void System::SaveKeyFrameTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

    }

    f.close();
    cout << endl << "trajectory saved!" << endl;
}

void System::SaveTrajectoryKITTI(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORB_SLAM2::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
          //  cout << "bad parent" << endl;
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
             Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
             Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}

int System::GetTrackingState()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackingState;
}

vector<MapPoint*> System::GetTrackedMapPoints()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedMapPoints;
}

vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedKeyPointsUn;
}

//vector<MapPoint*> System::GetTrackedMapPointsInArea()
//{
//    unique_lock<mutex> lock(mMutexState);
//    return mTrackedMapPointsInArea;
//}

vector<size_t > System::GetTrackedIndices()
{
    unique_lock<mutex> lock(mMutexState);
    return mvIndices;
}

cv::Mat System::GetPlaneMatrix()
{
    unique_lock<mutex> lock(mMutexDetectPlane);
    return mTpw.clone();
}

bool System::GetActivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    return mbActivateLocalizationMode;
}

cv::Mat System::TrackedPlaneCompute()
{
    cv::Mat Tpw;

    //填充图像帧中间区域对应的地图点 mvMPsInArea
    int nn = mvIndices.size();
    vector<MapPoint*> vMPsInArea = vector<MapPoint*>(nn, static_cast<MapPoint*>(NULL));
    if(!mvIndices.empty())
    {
        int i2 = 0;
        for(vector<size_t>::const_iterator vit=mvIndices.begin(); vit!=mvIndices.end(); vit++)
        {
            const size_t i1 = *vit;
            MapPoint* pMP = mTrackedMapPoints[i1];
            vMPsInArea[i2] = pMP;
            i2++;
        }
    }

    //给mvMPsInArea赋值
    FillMpsTnArea(vMPsInArea);
    //给mvInlierMP赋值
    RANSACMPS(mTrackedMapPoints);
    Tpw = Recompute1();
    return Tpw.clone();
}

//给mvMPsInArea赋值
bool System::FillMpsTnArea(const vector<ORB_SLAM2::MapPoint *> &vMPsInArea)
{
    //筛选出当前帧中间区域的地图点
    vector<cv::Mat> vPointsInArea;
    vPointsInArea.reserve(vMPsInArea.size());
    vector<MapPoint*> vPointMPInArea;
    vPointMPInArea.reserve(vMPsInArea.size());

    for(size_t i=0; i<vMPsInArea.size(); i++)
    {
        MapPoint* pMP=vMPsInArea[i];
        if(pMP)
        {
            if(pMP->Observations()>2)//5
            {
                vPointsInArea.push_back(pMP->GetWorldPos());
                vPointMPInArea.push_back(pMP);
            }
        }
    }
    const int n = vPointMPInArea.size();
    //当前帧中间部分对应的地图点数量太少
    if(n<3)
    {
        cout<<"MapPoints<3"<<endl;
        return false;
    }
    mvMPsInArea = vPointMPInArea;
    return true;
}

//给mvInlierMP赋值 用RANSAC筛选出当前帧的inlier地图点
bool System::RANSACMPS(const vector<ORB_SLAM2::MapPoint *> &vMPs, const int iterations)
{
    // 筛选出当前帧的地图点
    vector<cv::Mat> vPoints;
    vPoints.reserve(vMPs.size());
    vector<MapPoint*> vPointMP;
    vPointMP.reserve(vMPs.size());

    for(size_t i=0; i<vMPs.size(); i++)
    {
        MapPoint* pMP=vMPs[i];
        if(pMP)
        {
            if(pMP->Observations()>5)//5
            {
                vPoints.push_back(pMP->GetWorldPos());
                vPointMP.push_back(pMP);
            }
        }
    }
    const int N = vPoints.size();
    //当前帧对应的地图点数量太少
    if(N<50)//50
    {
        return false;
    }

    // Indices for minimum set selection
    vector<size_t> vAllIndices;
    vAllIndices.reserve(N);
    vector<size_t> vAvailableIndices;

    for(int i=0; i<N; i++)
    {
        vAllIndices.push_back(i);
    }

    float bestDist = 1e10;
    vector<float> bestvDist;

    //RANSAC选出当前帧对应的地图点
    for(int n=0; n<iterations; n++)
    {
        vAvailableIndices = vAllIndices;

        cv::Mat A(3,4,CV_32F);
        A.col(3) = cv::Mat::ones(3,1,CV_32F);

        // Get min set of points
        for(short i = 0; i < 3; ++i)
        {
            int randi = DUtils::Random::RandomInt(0, vAvailableIndices.size()-1);

            int idx = vAvailableIndices[randi];

            A.row(i).colRange(0,3) = vPoints[idx].t();

            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }

        cv::Mat u,w,vt;
        cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

        const float a = vt.at<float>(3,0);
        const float b = vt.at<float>(3,1);
        const float c = vt.at<float>(3,2);
        const float d = vt.at<float>(3,3);

        vector<float> vDistances(N,0);

        const float f = 1.0f/sqrt(a*a+b*b+c*c+d*d);

        for(int i=0; i<N; i++)
        {
            vDistances[i] = fabs(vPoints[i].at<float>(0)*a+vPoints[i].at<float>(1)*b+vPoints[i].at<float>(2)*c+d)*f;
        }

        vector<float> vSorted = vDistances;
        sort(vSorted.begin(),vSorted.end());

        int nth = max((int)(0.2*N),10);//20
        const float medianDist = vSorted[nth];

        if(medianDist<bestDist)
        {
            bestDist = medianDist;
            bestvDist = vDistances;
        }
    }

    // Compute threshold inlier/outlier
    const float th = 1.4*bestDist;
    vector<bool> vbInliers(N,false);
    int nInliers = 0;
    for(int i=0; i<N; i++)
    {
        if(bestvDist[i]<th)
        {
            nInliers++;
            vbInliers[i]=true;
        }
    }


    //mvInlierMPs.reserve(nInliers);
    vector<MapPoint*> vInlierMPs(nInliers,NULL);

    int nin = 0;
    for(int i=0; i<N; i++)
    {
        if(vbInliers[i])
        {
            vInlierMPs[nin] = vPointMP[i];
            nin++;
        }
    }

    //int ntmp = vInlierMPs.size();
    //mvInlierMPs = vector<MapPoint*>(ntmp, static_cast<MapPoint*>(NULL));
    if(!vInlierMPs.empty())
    {
        mvInlierMPs = vInlierMPs;
    }

    return true;
}

//用当前帧中间区域对应的inlier地图点来计算o和n
cv::Mat System::Recompute()
{
    const int N = mvInlierMPsInArea.size();
    cout<<"plane N: "<<N<<endl;

    // Recompute plane with all points
    cv::Mat A = cv::Mat(N,4,CV_32F);
    A.col(3) = cv::Mat::ones(N,1,CV_32F);

    o = cv::Mat::zeros(3,1,CV_32F);

    int nPoints = 0;
    for(int i=0; i<N; i++)
    {
        MapPoint* pMP = mvInlierMPsInArea[i];
        if(!pMP->isBad())
        {
            cv::Mat Xw = pMP->GetWorldPos();
            o+=Xw;
            A.row(nPoints).colRange(0,3) = Xw.t();
            nPoints++;
        }
    }
    A.resize(nPoints);

    cv::Mat u,w,vt;
    cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    //SVD分解后4*4的V的转置
    float a = vt.at<float>(3,0);
    float b = vt.at<float>(3,1);
    float c = vt.at<float>(3,2);

    o = o*(1.0f/nPoints);

    const float f = 1.0f/sqrt(a*a+b*b+c*c);

    // Compute XC just the first time 相机到plane的向量？
    if(XC.empty())
    {
        cv::Mat Oc = -mTcw.colRange(0,3).rowRange(0,3).t()*mTcw.rowRange(0,3).col(3);
        XC = Oc-o;
    }

    if((XC.at<float>(0)*a+XC.at<float>(1)*b+XC.at<float>(2)*c)>0)
    {
        a=-a;
        b=-b;
        c=-c;
    }

    const float nx = a*f;
    const float ny = b*f;
    const float nz = c*f;

    n = (cv::Mat_<float>(3,1)<<nx,ny,nz);

    cv::Mat up = (cv::Mat_<float>(3,1) << 0.0f, 1.0f, 0.0f);

    cv::Mat v = up.cross(n);
    const float sa = cv::norm(v);
    const float ca = up.dot(n);
    const float ang = atan2(sa,ca);
    cv::Mat Tpw = cv::Mat::eye(4,4,CV_32F);

    Tpw.rowRange(0,3).colRange(0,3) = EXPSO3(v*ang/sa)*EXPSO3(up*rang);
    o.copyTo(Tpw.col(3).rowRange(0,3));
    //cout << "Tpw = " << endl << "" << Tpw << endl ;

    return Tpw.clone();
}

//用当前帧中间区域对应的地图点来计算o，用当前帧对应的inlier地图点来计算n
cv::Mat System::Recompute1()
{
    //更新mvInlierMPs
    RANSACMPS(mTrackedMapPoints);

    const int nn = mvMPsInArea.size();
    const int N = mvInlierMPs.size();
    cout<<"plane N: "<<N<<endl<<"plane nn: "<<nn<<endl;

    //用当前帧对应的inlier地图点数据构造A矩阵
    cv::Mat A = cv::Mat(N,4,CV_32F);
    A.col(3) = cv::Mat::ones(N,1,CV_32F);
    int NPoints = 0;
    for(int i=0; i<N; i++)
    {
        MapPoint* pMP = mvInlierMPs[i];
        if(!pMP->isBad())
        {
            cv::Mat Xw = pMP->GetWorldPos();
            //o+=Xw;
            A.row(NPoints).colRange(0,3) = Xw.t();
            NPoints++;
        }
    }
    A.resize(NPoints);

    o = cv::Mat::zeros(3,1,CV_32F);
    int nPoints=0;
    for(int i=0; i<nn; i++)
    {
        MapPoint* pMP = mvMPsInArea[i];
        if(!pMP->isBad())
        {
            cv::Mat Xw = pMP->GetWorldPos();
            o+=Xw;
            nPoints++;
        }
    }
    o = o*(1.0f/nPoints);

    //A矩阵SVD分解
    cv::Mat u,w,vt;
    cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    //SVD分解后4*4的V的转置
    float a = vt.at<float>(3,0);
    float b = vt.at<float>(3,1);
    float c = vt.at<float>(3,2);

    const float f = 1.0f/sqrt(a*a+b*b+c*c);

    // Compute XC just the first time 相机到plane的向量？
    if(XC.empty())
    {
        cv::Mat Oc = -mTcw.colRange(0,3).rowRange(0,3).t()*mTcw.rowRange(0,3).col(3);
        XC = Oc-o;
    }

    if((XC.at<float>(0)*a+XC.at<float>(1)*b+XC.at<float>(2)*c)>0)
    {
        a=-a;
        b=-b;
        c=-c;
    }

    const float nx = a*f;
    const float ny = b*f;
    const float nz = c*f;

    n = (cv::Mat_<float>(3,1)<<nx,ny,nz);

    cv::Mat up = (cv::Mat_<float>(3,1) << 0.0f, 1.0f, 0.0f);

    cv::Mat v = up.cross(n);
    const float sa = cv::norm(v);
    const float ca = up.dot(n);
    const float ang = atan2(sa,ca);
    cv::Mat Tpw = cv::Mat::eye(4,4,CV_32F);

    Tpw.rowRange(0,3).colRange(0,3) = EXPSO3(v*ang/sa)*EXPSO3(up*rang);
    o.copyTo(Tpw.col(3).rowRange(0,3));

    return Tpw.clone();
}

//用当前帧对应的inlier地图点来计算o和n
cv::Mat System::Recompute2()
{
    //更新mvInlierMPs
    RANSACMPS(mTrackedMapPoints);

    const int N = mvInlierMPs.size();
    cout<<"plane N: "<<N<<endl;

    // Recompute plane with all points
    cv::Mat A = cv::Mat(N,4,CV_32F);
    A.col(3) = cv::Mat::ones(N,1,CV_32F);

    o = cv::Mat::zeros(3,1,CV_32F);

    int nPoints = 0;
    for(int i=0; i<N; i++)
    {
        MapPoint* pMP = mvInlierMPs[i];
        if(!pMP->isBad())
        {
            cv::Mat Xw = pMP->GetWorldPos();
            o+=Xw;
            A.row(nPoints).colRange(0,3) = Xw.t();
            nPoints++;
        }
    }
    A.resize(nPoints);

    cv::Mat u,w,vt;
    cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    //SVD分解后4*4的V的转置
    float a = vt.at<float>(3,0);
    float b = vt.at<float>(3,1);
    float c = vt.at<float>(3,2);

    o = o*(1.0f/nPoints);

    const float f = 1.0f/sqrt(a*a+b*b+c*c);

    // Compute XC just the first time 相机到plane的向量？
    if(XC.empty())
    {
        cv::Mat Oc = -mTcw.colRange(0,3).rowRange(0,3).t()*mTcw.rowRange(0,3).col(3);
        XC = Oc-o;
    }

    if((XC.at<float>(0)*a+XC.at<float>(1)*b+XC.at<float>(2)*c)>0)
    {
        a=-a;
        b=-b;
        c=-c;
    }

    const float nx = a*f;
    const float ny = b*f;
    const float nz = c*f;

    n = (cv::Mat_<float>(3,1)<<nx,ny,nz);

    cv::Mat up = (cv::Mat_<float>(3,1) << 0.0f, 1.0f, 0.0f);

    cv::Mat v = up.cross(n);
    const float sa = cv::norm(v);
    const float ca = up.dot(n);
    const float ang = atan2(sa,ca);
    cv::Mat Tpw = cv::Mat::eye(4,4,CV_32F);

    Tpw.rowRange(0,3).colRange(0,3) = EXPSO3(v*ang/sa)*EXPSO3(up*rang);
    o.copyTo(Tpw.col(3).rowRange(0,3));

    return Tpw.clone();
}

//增加的部分*********************************************************************************************************
//*****************************************************************************************************************
void System::SaveMap(const string &filename)
{
    std::ofstream out(filename, std::ios_base::binary);
    if (!out)
    {
        cerr << "Cannot Write to Mapfile: " << mapfile << std::endl;
        exit(-1);
    }
    cout << "Saving Mapfile: " << mapfile << std::flush;
    boost::archive::binary_oarchive oa(out, boost::archive::no_header);
    oa << mpMap;
    oa << mpKeyFrameDatabase;
    cout << " ...done" << std::endl;
    out.close();
}


void System::SaveMap()
{
    if (mTrackingState == 3 || mTrackingState == 2)
    {
        cout << "Pause the local mapper to save a map" << endl;
        mpLocalMapper->RequestStop();
        while (!mpLocalMapper->isStopped())
        {
            usleep(5000);
        }
        
        string filename = to_string(time(0)) + ".bin";

        std::ofstream out(filename, std::ios_base::binary);
        if (!out)
        {
            cerr << "Cannot Write to Mapfile: " << filename << std::endl;
            exit(-1);
        }
        cout << "Saving Mapfile: " << filename << std::flush;
        boost::archive::binary_oarchive oa(out, boost::archive::no_header);
        oa << mpMap;
        oa << mpKeyFrameDatabase;
        cout << " ... done" << std::endl;
        out.close();
        mpLocalMapper->Release();
    }
    else
    {
        cout << "ORB-SLAM not initialised. Map not saved." << endl;
    }
}


bool System::LoadMap(const string &filename)
{
    std::ifstream in(filename, std::ios_base::binary);
    if (!in)
    {
        cerr << "Cannot Open Mapfile: " << mapfile << " , Create a new one" << std::endl;
        return false;
    }
    cout << "Loading Mapfile: " << mapfile << std::flush;
    boost::archive::binary_iarchive ia(in, boost::archive::no_header);
    ia >> mpMap;
    ia >> mpKeyFrameDatabase;
    mpKeyFrameDatabase->SetORBvocabulary(mpVocabulary);
    cout << " ...done" << std::endl;
    cout << "Map Reconstructing" << flush;
    vector<ORB_SLAM2::KeyFrame*> vpKFS = mpMap->GetAllKeyFrames();
    unsigned long mnFrameId = 0;
    for (auto it:vpKFS) {
        it->SetORBvocabulary(mpVocabulary);
        it->ComputeBoW();
        if (it->mnFrameId > mnFrameId)
            mnFrameId = it->mnFrameId;
    }
    Frame::nNextId = mnFrameId;
    cout << " ...done" << endl;
    in.close();
    return true;
}


cv::Mat System::GetMarkerPose()
{
    cv::Mat Tcm = cv::Mat::eye(4, 4, CV_32F);
    Tcm = mpTracker->mTcm.clone();
    return Tcm;
}


cv::Mat System::EXPSO3(const float &x, const float &y, const float &z)
{
    cv::Mat I = cv::Mat::eye(3,3,CV_32F);
    const float d2 = x*x+y*y+z*z;
    const float d = sqrt(d2);
    cv::Mat W = (cv::Mat_<float>(3,3) << 0, -z, y,
            z, 0, -x,
            -y,  x, 0);
    if(d<EPS)
        return (I + W + 0.5f*W*W);
    else
        return (I + W*sin(d)/d + W*W*(1.0f-cos(d))/d2);
}

cv::Mat System::EXPSO3(const cv::Mat &v)
{
    return EXPSO3(v.at<float>(0),v.at<float>(1),v.at<float>(2));
}


} //namespace ORB_SLAM


