#include <iostream>

#include <opencv2/opencv.hpp>

#define GLM_FORCE_RADIANS
#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>

using namespace std;
using namespace cv;

Mat RCalib, TCalib, P1, P2, K, DistCoef;
Mat R, tvec, initRvec, initTvec;
cv::Mat cvToGl = cv::Mat::zeros(4, 4, CV_64F);
cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
cv::Mat M1l,M2l,M1r,M2r;

Mat getCameraMatrix()
{
    return K.clone();
}

Mat getDistorsion()
{
    return DistCoef.clone();
}

Mat getM1l()
{
    return M1l.clone();
}

Mat getM2l()
{
    return M2l.clone();
}

Mat getM1r()
{
    return M1r.clone();
}

Mat getM2r()
{
    return M2r.clone();
}

glm::mat4 getViewMatrix(bool slamMode)
{
    glm::mat4 V;
    Mat viewMatrix = cv::Mat::zeros(4, 4, CV_64FC1);

    if(slamMode)
    {
        R.convertTo(R, CV_64F);
        tvec.convertTo(tvec, CV_64F);
        //填充矩阵
        for(unsigned int row=0; row<3; ++row)
        {
            for(unsigned int col=0; col<3; ++col)
            {
                viewMatrix.at<double>(row, col) = R.at<double>(row, col);
            }
            viewMatrix.at<double>(row, 3) = tvec.at<double>(row, 0);
        }
        viewMatrix.at<double>(3, 3) = 1.0f;

        viewMatrix = cvToGl * viewMatrix;
    }
    else
    {
        viewMatrix = cvToGl;
    }

    viewMatrix.convertTo(viewMatrix, CV_32F);

    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            V[i][j] = viewMatrix.at<float>(j,i);
        }
    }

    return V;
}
//读取相机参数
bool initTracking(const char * Extrinsics_path)
{
    cvToGl.at<double>(0, 0) = 1.0f;
    // Invert the y axis
    cvToGl.at<double>(1, 1) = -1.0f;
    // invert the z axis
    cvToGl.at<double>(2, 2) = -1.0f;
    cvToGl.at<double>(3, 3) = 1.0f;

    cv::FileStorage f1;
    f1.open(Extrinsics_path, cv::FileStorage::READ);

    if (f1.isOpened())
    {
        f1["R"] >> RCalib;
        f1["T"] >> TCalib;
        f1["P1"] >> P1;
        f1["P2"] >> P2;
        f1["D1"] >> DistCoef;
        f1.release();
    }
    else
    {
        cout << "Couldn't open Extrinsics.xml" << endl;
        return 0;
    }

    P1.rowRange(0,3).colRange(0,3).copyTo(K);

    return 1;
}

bool initTracking1(const char * Extrinsics_path)
{
    cvToGl.at<double>(0, 0) = 1.0f;
    // Invert the y axis
    cvToGl.at<double>(1, 1) = -1.0f;
    // invert the z axis
    cvToGl.at<double>(2, 2) = -1.0f;
    cvToGl.at<double>(3, 3) = 1.0f;

    cv::FileStorage fsSettings;
    fsSettings.open(Extrinsics_path, cv::FileStorage::READ);

    if (fsSettings.isOpened())
    {

        float fx = fsSettings["Camera.fx"];
        float fy = fsSettings["Camera.fy"];
        float cx = fsSettings["Camera.cx"];
        float cy = fsSettings["Camera.cy"];


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

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);
        fsSettings.release();
    }
    else
    {
        cout << "Couldn't open Extrinsics.xml" << endl;
        return 0;
    }

    return 1;
}
//赋值R t
bool trackStereo(Mat CameraPose)
{
    if (CameraPose.empty())
    {
        return 0;
    }
    CameraPose.rowRange(0,3).colRange(0,3).copyTo(R);
    CameraPose.rowRange(0,3).col(3).copyTo(tvec);

    return 1;
}
