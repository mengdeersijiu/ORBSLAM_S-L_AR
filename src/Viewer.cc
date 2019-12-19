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

#include "Viewer.h"
#include <pangolin/pangolin.h>

#include <mutex>

namespace ORB_SLAM2
{
//***********************************************************************************
const float eps = 1e-4;

cv::Mat ExpSO3(const float &x, const float &y, const float &z)
{
    cv::Mat I = cv::Mat::eye(3,3,CV_32F);
    const float d2 = x*x+y*y+z*z;
    const float d = sqrt(d2);
    cv::Mat W = (cv::Mat_<float>(3,3) << 0, -z, y,
                                         z, 0, -x,
                                        -y,  x, 0);
    if(d<eps)
        return (I + W + 0.5f*W*W);
    else
        return (I + W*sin(d)/d + W*W*(1.0f-cos(d))/d2);
}

cv::Mat ExpSO3(const cv::Mat &v)
{
    return ExpSO3(v.at<float>(0),v.at<float>(1),v.at<float>(2));
}
//***********************************************************************************  
Viewer::Viewer(){}

Viewer::Viewer(System* pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking, const string &strSettingPath, bool mbReuseMap_):
    mpSystem(pSystem), mpFrameDrawer(pFrameDrawer),mpMapDrawer(pMapDrawer), mpTracker(pTracking),
    mbFinishRequested(false), mbFinished(true), mbPaused(false), mbStopped(true), mbStopRequested(false), mbReuseMap(mbReuseMap_)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    float fps = fSettings["Camera.fps"];
    if(fps<1)
        fps=30;
    mT = 1e3/fps;

    mImageWidth = fSettings["Camera.width"];
    mImageHeight = fSettings["Camera.height"];
    if(mImageWidth<1 || mImageHeight<1)
    {
        mImageWidth = 640;
        mImageHeight = 480;
    }

    mViewpointX = fSettings["Viewer.ViewpointX"];
    mViewpointY = fSettings["Viewer.ViewpointY"];
    mViewpointZ = fSettings["Viewer.ViewpointZ"];
    mViewpointF = fSettings["Viewer.ViewpointF"];
    
    fx = fSettings["Camera.fx"];
    fy = fSettings["Camera.fy"];
    cx = fSettings["Camera.cx"];
    cy = fSettings["Camera.cy"];
    
    msTcwPath = static_cast<string>((string)fSettings["Tcw1.path"]);
}

void Viewer::Run()
{
    mbFinished = false;
    mbStopped = false;
    mbPaused = false;
    
    cv::Mat im, ima, Tcw, Tcm;
    //int status;
    vector<cv::KeyPoint> vKeys;
    //vector<MapPoint*> vMPsInArea;
    
    string sTwmPath;
    string sTcwPath = static_cast<string>((string)msTcwPath);

    pangolin::CreateWindowAndBind("ORB-SLAM2: Map Viewer",1024,768);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);
    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);
    pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
    pangolin::Var<bool> menuShowPointsInArea("menu.Show PointsInArea", false,true);
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
    pangolin::Var<bool> menuShowGraph("menu.Show Graph",true,true);    
    pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode",mbReuseMap,true);
    pangolin::Var<bool> menuSaveMap("menu.Save Map",false,false);
    pangolin::Var<bool> menuPauseResume("menu.Pause/Resume",false,false);
    pangolin::Var<bool> menuGetPoseFromMark("menu.GetPoseFromMark",false,false);
    pangolin::Var<bool> menuReset("menu.Reset",false,false);
    pangolin::Var<bool> menu_InsertCube1("menu.InsertCube1",false,true);
    pangolin::Var<bool> menu_drawcube("menu.Draw Cube", false,true);
    pangolin::Var<float> menu_cubesize("menu. Cube Size",0.05,0.01,0.3);
    pangolin::Var<bool> menu_drawim("menu.Draw Image",false,true);
    pangolin::Var<bool> menu_GetTpw("menu.GetTpw", false,false);
    
    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000),
                pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
                );
    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::Display("image1")
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

//     pangolin::OpenGlRenderState s_cam1(
// 	    pangolin::ProjectionMatrix(1024,768,500,500,512,389,0.1,1000),
// 	    pangolin::ModelViewLookAt(0.0,-0.7,-1.8, 0,0,0,0.0,-1.0, 0.0)
// 	    );
//     pangolin::View& d_cam1 = pangolin::CreateDisplay()
//             .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
//             .SetHandler(new pangolin::Handler3D(s_cam1));

    pangolin::OpenGlMatrix Twc;
    
    Twc.SetIdentity();
    //Tcw.SetIdentity();
//*************************************************************************
//     pangolin::View& d_image = pangolin::Display("image")
//             .SetBounds(0.0f,1.0f,pangolin::Attach::Pix(175),1.0f,(float)mImageWidth/mImageHeight)
//             .SetLock(pangolin::LockLeft, pangolin::LockTop);
// 
//     pangolin::GlTexture imageTexture(mImageWidth,mImageHeight,GL_RGB,false,0,GL_RGB,GL_UNSIGNED_BYTE);
//     pangolin::OpenGlMatrixSpec P = pangolin::ProjectionMatrixRDF_TopLeft(mImageWidth,mImageHeight,fx,fy,cx,cy,0.001,1000);
//*************************************************************************
    cv::namedWindow("ORB-SLAM2: Current Frame");

    bool bFollow = true;
    bool bLocalizationMode = false;

    while(1)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);
	    //cout<<"pangolin::OpenGlMatrixTwc"<< Twc <<endl;
        //mvMPsInArea=mpSystem->GetTrackedMapPointsInArea();

        if(menuFollowCamera && bFollow)
        {
            s_cam.Follow(Twc);
        }
        else if(menuFollowCamera && !bFollow)
        {
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
            s_cam.Follow(Twc);
            bFollow = true;
        }
        else if(!menuFollowCamera && bFollow)
        {
            bFollow = false;
        }

        if(menuLocalizationMode && !bLocalizationMode)
        {
            mpSystem->ActivateLocalizationMode();
            bLocalizationMode = true;
        }
        else if(!menuLocalizationMode && bLocalizationMode)
        {
            mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
        }
        
        //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
	    glClearColor(1.0f,1.0f,1.0f,1.0f);
// 	d_cam1.Activate(s_cam1);
	
	/////////////////////////////
// 	d_image.Activate();
// 	glColor3f(1.0,1.0,1.0);

	
	//GetImagePose(ima,Tcw,status,vKeys,vMPs,Tcm);

	//im = mpFrameDrawer->DrawFrame().clone();

	// Draw image
//         if(menu_drawim)
// 	{
// 	    DrawImageTexture(imageTexture,im);
// 	}
	
        // 	glClear(GL_DEPTH_BUFFER_BIT);
// 
//         // Load camera projection
//         glMatrixMode(GL_PROJECTION);
//         P.Load();
//         glMatrixMode(GL_MODELVIEW);
// 	
//         LoadCameraPose(Twc);
	
        mpMapDrawer->DrawCurrentCamera(Twc);
        if(menuShowKeyFrames || menuShowGraph)
            mpMapDrawer->DrawKeyFrames(menuShowKeyFrames,menuShowGraph);
        if(menuShowPoints)
            mpMapDrawer->DrawMapPoints();
//        if(menuShowPointsInArea)
//            DrawMapPointsInArea(mvMPsInArea);

        if(menuSaveMap)
        {
          mpSystem->SaveMap();
          menuSaveMap = false;
        }

        if(menu_GetTpw)
        {
            mpSystem->PlaneSwitch();
            cv::Mat tmp = mpSystem->GetPlaneMatrix();
            cout<<"Tpw = "<< endl << tmp << endl;
            menu_GetTpw = false;
        }

//	cv::Mat im = mpFrameDrawer->DrawFrame();
// 	if(im.empty())
// 	{
// 	    cout << "im is empty!" << endl;
// 	}
// 	else
// 	{
// 	    cv::Mat ima = im.clone();
//  	    imageTexture.Upload(ima.data,GL_RGB,GL_UNSIGNED_BYTE);
// 
// 	    imageTexture.RenderToViewportFlipY();
// 	}

        pangolin::FinishFrame();

        cv::Mat im = mpFrameDrawer->DrawFrame();
        cv::imshow("ORB-SLAM2: Current Frame",im);
        cv::waitKey(mT);

        if(menuReset)
        {
            menuShowGraph = true;
            menuShowKeyFrames = true;
            menuShowPoints = true;
            menuLocalizationMode = false;
            if(bLocalizationMode)
                mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
            bFollow = true;
            menuFollowCamera = true;
            mpSystem->Reset();
            menuReset = false;
	        menuPauseResume = false;
	        menuGetPoseFromMark = false;
        }
        
        
        if(menuPauseResume&&!isStopped())
        {
          mpSystem->Pause();
          SetPause();
        }
        else if(!menuPauseResume&&isPaused())
        {
          mpSystem->Resume();
          UnsetPause();
        }
	
        if(menuGetPoseFromMark)
        {
          cout << "menuGetPoseFromMark is pressed" << endl;
          mpTracker->GetposefromMark(im);
        }
	
        if(menu_InsertCube1)
        {
            cv::Mat tempTcw1;
            cv::FileStorage fTcwLoad(sTcwPath, cv::FileStorage::READ);
            //cout << "InsertCubeCamPos" << endl;
            string sTcw1;
            fTcwLoad["Tcw1.path"] >> sTcw1;
            cv::FileStorage fTcw1Load(sTcw1, cv::FileStorage::READ);
            fTcw1Load["Tcw1"] >> tempTcw1;
            //cout << "Tcw1: "<<tempTcw1 << endl;

            if(tempTcw1.empty())
            {
                cerr << "err: Tcw1 = []" << endl;
                menu_InsertCube1 = false;
            }
            else
            {
                cv::Mat sTwc1 = cv::Mat::eye(4, 4, CV_32F);

                cv::Mat Rcw = tempTcw1.rowRange(0,3).colRange(0,3);
                cv::Mat Rwc = Rcw.t();
                cv::Mat tcw = tempTcw1.rowRange(0,3).col(3);
                cv::Mat twc = -Rwc*tcw;

                Rwc.copyTo(sTwc1.rowRange(0,3).colRange(0,3));
                twc.copyTo(sTwc1.rowRange(0,3).col(3));

                glTwc.m[0] = sTwc1.at<float>(0,0);
                glTwc.m[1] = sTwc1.at<float>(1,0);
                glTwc.m[2] = sTwc1.at<float>(2,0);
                glTwc.m[3] = 0.0;

                glTwc.m[4] = sTwc1.at<float>(0,1);
                glTwc.m[5] = sTwc1.at<float>(1,1);
                glTwc.m[6] = sTwc1.at<float>(2,1);
                glTwc.m[7] = 0.0;

                glTwc.m[8] = sTwc1.at<float>(0,2);
                glTwc.m[9] = sTwc1.at<float>(1,2);
                glTwc.m[10] = sTwc1.at<float>(2,2);
                glTwc.m[11] = 0.0;

                glTwc.m[12] = sTwc1.at<float>(0,3);
                glTwc.m[13] = sTwc1.at<float>(1,3);
                glTwc.m[14] = sTwc1.at<float>(2,3);
                glTwc.m[15] = 1.0;
                cout<<"glTwc: "<< glTwc <<endl;
                glPushMatrix();
                glTwc.Multiply();
                // Draw cube
                if(menu_drawcube)
                {
                    pangolin::glDrawColouredCube(-menu_cubesize,menu_cubesize);
                }
                glPopMatrix();
            }
        }

	    if(Stop())
        {
            while(isStopped())
            {
                std::this_thread::sleep_for(std::chrono::microseconds(3000));
            }
        }

        if(CheckFinish())
            break;
    }
    SetFinish();
}

void Viewer::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool Viewer::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void Viewer::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool Viewer::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

bool Viewer::isPaused()
{
    unique_lock<mutex> lock(mMutexPause);
    return mbPaused;
}

void Viewer::SetPause()
{
    unique_lock<mutex> lock(mMutexPause);
    mbPaused = true;
}

void Viewer::UnsetPause()
{
    unique_lock<mutex> lock(mMutexPause);
    mbPaused = false;
}

void Viewer::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

bool Viewer::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool Viewer::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }
    return false;
}


void Viewer::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}

/*
bool Viewer::isarucoed()
{
    //unique_lock<mutex> lock(mMutexAruco);
    return mbArucoed; 
}

void Viewer::Setaruco()
{
    //unique_lock<mutex> lock(mMutexAruco);
    mbArucoed = true;
}

void Viewer::Unsetaruco()
{
    //unique_lock<mutex> lock(mMutexAruco);
    mbArucoed = false;
}
*/
void Viewer::LoadCameraPose(const pangolin::OpenGlMatrix &Twc)
{
        pangolin::OpenGlMatrix Tcw;
        Tcw.SetIdentity();
        Tcw = Twc.Inverse();
        Tcw.Load();
// 	cout<<"pangolin::OpenGlMatrix Tcw:"<<endl<< Tcw <<endl;
}

//增加的部分******************************************************************************************************

void Viewer::DrawImageTexture(pangolin::GlTexture &imageTexture, cv::Mat &im)
{
    if(!im.empty())
    {
        imageTexture.Upload(im.data,GL_RGB,GL_UNSIGNED_BYTE);
        imageTexture.RenderToViewportFlipY();
    }
}

void Viewer::SetImagePose(const cv::Mat &im, const cv::Mat &Tcw, const int &status, 
			  const vector<cv::KeyPoint> &vKeys, const vector<ORB_SLAM2::MapPoint*> &vMPs, const cv::Mat &Tcm)
{
    unique_lock<mutex> lock(mMutexPoseImage);
    mImage = im.clone();
    mTcw = Tcw.clone();
    mStatus = status;
    mvKeys = vKeys;
    mvMPs = vMPs;
    mTcm = Tcm;
//     if(!mTcw.empty())
//     {
//         cout<<"cv::Mat mTcw: "<<endl<<mTcw<<endl;
//     }

}

void Viewer::GetImagePose(cv::Mat &im, cv::Mat &Tcw, int &status, std::vector<cv::KeyPoint> &vKeys,  std::vector<MapPoint*> &vMPs, cv::Mat &Tcm)
{
    unique_lock<mutex> lock(mMutexPoseImage);
    im = mImage.clone();
    Tcw = mTcw.clone();
    status = mStatus;
    vKeys = mvKeys;
    vMPs = mvMPs;
    Tcm = mTcm;
//     if(im.empty())
//     {
//         cout<<"im empty"<<endl;
//     }
//     if(mImage.empty())
//     {
//         cout<<"mImage empty"<<endl;
//     }
//     else{
//       cout<<"mImage"<<endl;
//     }
//     cout<<"mtest: "<<mtest<<endl;
//     if(Tcw.empty())
//     {
//         cout<<"Tcw empty"<<endl;
//     }
//     else{
//       cout<<"Tcw: "<<Tcw<<endl;
//     }
}

//void Viewer::DrawMapPointsInArea(const vector<MapPoint*> &vMPsInArea)
//{
//    if(vMPsInArea.empty())
//        return;
//
//    glPointSize(2);
//    glBegin(GL_POINTS);
//    glColor3f(0.0,0.0,0.0);
//
//    for(size_t i=0, iend=vMPsInArea.size(); i<iend;i++)
//    {
//        cv::Mat pos = vMPsInArea[i]->GetWorldPos();
//        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
//    }
//    glEnd();
//}

Plane1* Viewer::DetectPlane(const cv::Mat Tcw, const std::vector<MapPoint*> &vMPs, const int iterations)
{
    // Retrieve 3D points
    vector<cv::Mat> vPoints;
    vPoints.reserve(vMPs.size());
    vector<MapPoint*> vPointMP;
    vPointMP.reserve(vMPs.size());

    for(size_t i=0; i<vMPs.size(); i++)
    {
        MapPoint* pMP=vMPs[i];
        if(pMP)
        {
            if(pMP->Observations()>5)
            {
                vPoints.push_back(pMP->GetWorldPos());
                vPointMP.push_back(pMP);
            }
        }
    }

    const int N = vPoints.size();

    if(N<50)
        return NULL;


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

    //RANSAC
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

        int nth = max((int)(0.2*N),20);
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

    return new Plane1(vInlierMPs,Tcw);
}


Plane1::Plane1(const std::vector<MapPoint *> &vMPs, const cv::Mat &Tcw):mvMPs(vMPs),mTcw(Tcw.clone())
{
    //-90到0度之间？
    rang = -3.14f/2+((float)rand()/RAND_MAX)*3.14f;
    Recompute();
}

void Plane1::Recompute()
{
    const int N = mvMPs.size();

    // Recompute plane with all points
    cv::Mat A = cv::Mat(N,4,CV_32F);
    A.col(3) = cv::Mat::ones(N,1,CV_32F);

    o = cv::Mat::zeros(3,1,CV_32F);

    int nPoints = 0;
    for(int i=0; i<N; i++)
    {
        MapPoint* pMP = mvMPs[i];
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
    Tpw = cv::Mat::eye(4,4,CV_32F);


    Tpw.rowRange(0,3).colRange(0,3) = ExpSO3(v*ang/sa)*ExpSO3(up*rang);
    o.copyTo(Tpw.col(3).rowRange(0,3));

    glTpw.m[0] = Tpw.at<float>(0,0);
    glTpw.m[1] = Tpw.at<float>(1,0);
    glTpw.m[2] = Tpw.at<float>(2,0);
    glTpw.m[3]  = 0.0;

    glTpw.m[4] = Tpw.at<float>(0,1);
    glTpw.m[5] = Tpw.at<float>(1,1);
    glTpw.m[6] = Tpw.at<float>(2,1);
    glTpw.m[7]  = 0.0;

    glTpw.m[8] = Tpw.at<float>(0,2);
    glTpw.m[9] = Tpw.at<float>(1,2);
    glTpw.m[10] = Tpw.at<float>(2,2);
    glTpw.m[11]  = 0.0;

    glTpw.m[12] = Tpw.at<float>(0,3);
    glTpw.m[13] = Tpw.at<float>(1,3);
    glTpw.m[14] = Tpw.at<float>(2,3);
    glTpw.m[15]  = 1.0;

    cout << "recompute Tpw = " << endl << "" << Tpw << endl ;

}

Plane1::Plane1(const float &nx, const float &ny, const float &nz, const float &ox, const float &oy, const float &oz)
{
    n = (cv::Mat_<float>(3,1)<<nx,ny,nz);
    o = (cv::Mat_<float>(3,1)<<ox,oy,oz);

    cv::Mat up = (cv::Mat_<float>(3,1) << 0.0f, 1.0f, 0.0f);

    cv::Mat v = up.cross(n);
    const float s = cv::norm(v);
    const float c = up.dot(n);
    const float a = atan2(s,c);
    Tpw = cv::Mat::eye(4,4,CV_32F);
    const float rang = -3.14f/2+((float)rand()/RAND_MAX)*3.14f;
    cout << rang;
    Tpw.rowRange(0,3).colRange(0,3) = ExpSO3(v*a/s)*ExpSO3(up*rang);
    o.copyTo(Tpw.col(3).rowRange(0,3));

    
    glTpw.m[0] = Tpw.at<float>(0,0);
    glTpw.m[1] = Tpw.at<float>(1,0);
    glTpw.m[2] = Tpw.at<float>(2,0);
    glTpw.m[3]  = 0.0;

    glTpw.m[4] = Tpw.at<float>(0,1);
    glTpw.m[5] = Tpw.at<float>(1,1);
    glTpw.m[6] = Tpw.at<float>(2,1);
    glTpw.m[7]  = 0.0;

    glTpw.m[8] = Tpw.at<float>(0,2);
    glTpw.m[9] = Tpw.at<float>(1,2);
    glTpw.m[10] = Tpw.at<float>(2,2);
    glTpw.m[11]  = 0.0;

    glTpw.m[12] = Tpw.at<float>(0,3);
    glTpw.m[13] = Tpw.at<float>(1,3);
    glTpw.m[14] = Tpw.at<float>(2,3);
    glTpw.m[15]  = 1.0;
    
    cout << "Tpw = " << endl << "" << Tpw << endl ;
}
//***************************************************************************************************************

}
