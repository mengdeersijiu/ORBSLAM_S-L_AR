#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>

#include <opencv2/opencv.hpp>
#include "planar_tracking.h"
#include "utils.h"

//ARUCO
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

using namespace cv;
using namespace std;

glm::mat4 Tracker::getInitModelMatrix()
{
    glm::mat4 initModelMatrix;
    Mat initR = cv::Mat::ones(3,3,CV_64FC1);
    Mat viewMatrix = cv::Mat::zeros(4, 4, CV_64FC1);
    if(!rvec.empty())
    {
        Rodrigues(rvec, initR);
    }

    for(unsigned int row=0; row<3; ++row)
    {
        for(unsigned int col=0; col<3; ++col)
        {
            viewMatrix.at<double>(row, col) = initR.at<double>(row, col);
        }
        if(!tvec.empty())
        {
            viewMatrix.at<double>(row, 3) = tvec.at<double>(row, 0);
        }
    }
    viewMatrix.at<double>(3, 3) = 1.0f;

    //viewMatrix = cvToGl * viewMatrix;

    viewMatrix.convertTo(viewMatrix, CV_32F);

    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            initModelMatrix[i][j] = viewMatrix.at<float>(j,i);
        }
    }
    return initModelMatrix;
}

glm::mat4 Tracker::getInitModelMatrix1(cv::Mat Tpw)
{
    glm::mat4 initModelMatrix1;

/*    Tpw.convertTo(Tpw,CV_64F);
    cv::Mat cvToGl = cv::Mat::zeros(4, 4, CV_64F);
    cvToGl.at<double>(0, 0) = 1.0f;
    cvToGl.at<double>(1, 1) = -1.0f;
    cvToGl.at<double>(2, 2) = -1.0f;
    cvToGl.at<double>(3, 3) = 1.0f;
    Tpw = cvToGl * Tpw;*/

    Tpw.convertTo(Tpw,CV_32F);

    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            initModelMatrix1[i][j] = Tpw.at<float>(j,i);
        }
    }
    return initModelMatrix1;
}

void Tracker::setFirstFrame(const char * first_frame_path)
{
    first_frame = imread(first_frame_path);
    vector<KeyPoint> kp;

    object_bb.push_back(Point2f(0,0));
    object_bb.push_back(Point2f(8.0,0));
    object_bb.push_back(Point2f(8.0,5.0));
    object_bb.push_back(Point2f(0,5.0));

    vector<Point2f> bb;
    bb.push_back(Point2f(0,0));
    bb.push_back(Point2f(640,0));
    bb.push_back(Point2f(640,400));
    bb.push_back(Point2f(0,400));

    Mat H;
    H = findHomography(bb, object_bb);
    
    detector->detectAndCompute(first_frame, noArray(), kp, first_desc);

    vector<Point2f> tmp_kp_orgn, tmp_kp_homo;

    first_kp = kp;
    //first_kp_1 = kp;

    for (size_t i = 0; i <= kp.size(); i++)
    {
        tmp_kp_orgn.push_back(Point2f(kp[i].pt));
    }

    perspectiveTransform(tmp_kp_orgn, tmp_kp_homo, H);
    for (size_t i = 0; i <= kp.size(); i++)
    {
        first_kp[i].pt = tmp_kp_homo[i];//x,y都缩小了80倍，不清楚为什么这么做
    }
}

bool Tracker::process(const Mat frame_left, bool slamMode)
{
    if (slamMode)//进入SLAM模式后就不再更新rvec, tvec
        return 1;

    vector<KeyPoint> kp;
    vector<Point3f> ObjectPoints;
    vector<Point2f> ImagePoints;
    Mat desc;
    
    detector->detectAndCompute(frame_left, noArray(), kp, desc);
    
    if(kp.size()<10)
    {
        return 0;
    }
    
    vector< vector<DMatch> > matches;
    vector<KeyPoint> matched1, matched2;
    matcher->knnMatch(first_desc, desc, matches, 2);
    for(unsigned i = 0; i < matches.size(); i++)
    {
        if(matches[i][0].distance < 0.8f * matches[i][1].distance)
        {
            matched1.push_back(first_kp[matches[i][0].queryIdx]);
            matched2.push_back(      kp[matches[i][0].trainIdx]);
        }
    }

    Mat inlier_mask, homography;

    size_t thd = (size_t)(0.08*first_desc.rows);

    if(matched1.size() >= thd)
    {
        homography = findHomography(Points(matched1), Points(matched2),
                                    RANSAC, 10.0f, inlier_mask);
    }

    if(matched1.size() < thd || homography.empty())
    {
        return 0;
    }


    for(unsigned i = 0; i < matched1.size(); i++)
    {
        if(inlier_mask.at<uchar>(i))
        {
            ObjectPoints.push_back(Point3f(matched1[i].pt.x, matched1[i].pt.y, 0));
            ImagePoints.push_back(Point2f(matched2[i].pt.x, matched2[i].pt.y));
        }
    }

    solvePnP(ObjectPoints, ImagePoints, K, noArray(), rvec, tvec);
    cout<<"tvec: "<<tvec<<endl;

    return 1;
}

bool Tracker::process1(cv::Mat &frame_left, bool slamMode)
{
    if (slamMode)
        return 1;

    aruco::CameraParameters camparam;
    camparam.CameraMatrix = K.clone();
    camparam.Distorsion = DistCoef.clone();
    camparam.CamSize.width = 640;
    camparam.CamSize.height = 400;
    //marker边长
    float MarkerSize = 0.086;
    int Marker_ID;
    aruco::MarkerDetector MDetector;
    aruco::CvDrawingUtils MDraw;
    //旋转向量
    cv::Mat Rvec;
    //平移向量
    cv::Mat Tvec;
    //识别marker
    vector<aruco::Marker> Markers = MDetector.detect(frame_left, camparam, MarkerSize);

    //识别出marker，并在100这个marker上绘制边
    for (unsigned int j=0;j<Markers.size();j++)
    {
        //marker ID test
        Marker_ID = Markers[j].id;
        //printf("Marker ID = %d \n",Marker_ID);

        if(Marker_ID == 699)
        {
            Markers[j].draw(frame_left,cv::Scalar(0,0,255),2);
            Markers[j].calculateExtrinsics(MarkerSize, camparam, false);
            //旋转向量
            rvec = Markers[j].Rvec;
            //平移向量
            tvec = Markers[j].Tvec;
            double T = cv::norm(Tvec);
            //cout<<"Tvec: "<<T<<endl;
            cout<<"Tvec: "<<Tvec<<endl;

            // 在图像上marker的位置绘制坐标
            if (camparam.isValid() && MarkerSize != -1)
            {
                MDraw.draw3dAxis(frame_left,camparam,rvec,tvec,MarkerSize);
            }
        }
    }
    cv::waitKey(16.667);//wait for key to be pressed
    cv::imshow("Frame",frame_left);


    if(rvec.empty()||tvec.empty())
    {
        return 0;
    }else{
        return 1;
    }
}