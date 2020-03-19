#define GLM_FORCE_RADIANS

#define MarkerID 699

#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>

#include <opencv2/opencv.hpp>
#include "planar_tracking.h"
#include "utils.h"

//ARUCO
//#include <aruco/aruco.h>
//#include <aruco/cvdrawingutils.h>

using namespace cv;
using namespace std;

glm::mat4 Tracker::getInitModelMatrix(bool slamMode)
{
    if(!slamMode)
    {
        //glm::mat4 rotation = glm::rotate(glm::mat4(1.0), glm::radians(180.0f), glm::vec3( -1, 0, 0));
        glm::mat4 initModelMatrix;
        cv::Mat initR = cv::Mat::ones(3,3,CV_64FC1);
        cv::Mat viewMatrix = cv::Mat::zeros(4, 4, CV_64FC1);

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

        viewMatrix.convertTo(viewMatrix, CV_32F);

        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                initModelMatrix[i][j] = viewMatrix.at<float>(j,i);
            }
        }

        //initModelMatrix = rotation * initModelMatrix;
        return initModelMatrix;
    }
}

glm::mat4 Tracker::getInitModelMatrix1(cv::Mat Tpw)
{
    glm::mat4 initModelMatrix1;
    Tpw.convertTo(Tpw,CV_32F);

#if 0
    cv::Mat initR = cv::Mat::ones(3,3,CV_64FC1);
    cv::Mat viewMatrix = cv::Mat::zeros(4, 4, CV_64FC1);

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
    viewMatrix.convertTo(viewMatrix, CV_32F);
#endif

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

#if 0
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
//    //旋转向量
//    cv::Mat Rvec;
//    //平移向量
//    cv::Mat Tvec;


    //识别marker
    vector<aruco::Marker> Markers = MDetector.detect(frame_left, camparam, MarkerSize);

    //识别出marker，并在699这个marker上绘制边
    for (unsigned int j=0;j<Markers.size();j++)
    {
        //marker ID test
        Marker_ID = Markers[j].id;

        if(Marker_ID == 699)
        {
            Markers[j].draw(frame_left,cv::Scalar(0,0,255),2);
            Markers[j].calculateExtrinsics(MarkerSize, camparam, false);
            //旋转向量
            rvec = Markers[j].Rvec;
            //平移向量
            tvec = Markers[j].Tvec;

            // 在图像上marker的位置绘制坐标
            if (camparam.isValid() && MarkerSize != -1)
            {
                MDraw.draw3dAxis(frame_left,camparam,rvec,tvec,MarkerSize);
            }
        }
    }
    //注意：orbslam的currentFrame窗口好像与下面这个窗口不能同时运行
    cv::waitKey(16);//wait for key to be pressed
    cv::imshow("Frame",frame_left);

#endif

    if(rvec.empty()||tvec.empty())
    {
        return 0;
    }else{
        return 1;
    }
}


bool Tracker::processARUCO(cv::Mat &frame_left, bool slamMode)
{
    if (slamMode)
        return 1;

//    Ptr<aruco::Dictionary> dictionary;
//    vector<int> markerIds;
//    vector<vector<cv::Point2f>> markerCorners, rejectedCandidates;
//    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
//    float markerLength = 0.086;

//    dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME::DICT_ARUCO_ORIGINAL);

    if(dictionary == NULL || detectorParams == NULL){
        cout<<"dictionary or detectorParams is null"<<endl;
        exit(11);
    }

    aruco::detectMarkers(
            frame_left,
            dictionary,
            markerCorners,
            markerIds,
            detectorParams,
            rejectedCandidates
            );

    if(markerIds.size() > 0) {
        int markerid = 0;
        for(unsigned int i=0;i<markerIds.size();i++){
            if(MarkerID == markerIds[i]){
                markerid = i;
                cout << "Marker found ID:"<< markerIds[markerid] <<"\n"<< endl;
            }
        }

        vector<cv::Vec3d> rvecs, tvecs;
        aruco::estimatePoseSingleMarkers(
                markerCorners,
                markerLength,
                K,
                DistCoef,//内参和畸变好像要用CV_32F的Mat（也就是float）
                rvecs,   //rvecs和tvecs好像要用Vec3d格式的vector
                tvecs
                );

        cv::Vec3d r = rvecs[markerid];
        cv::Vec3d t = tvecs[markerid];

        cv::Mat rtemp = cv::Mat::zeros(3,1,CV_32FC1);
        cv::Mat ttemp = cv::Mat::zeros(3,1,CV_32FC1);

        rtemp.at<float>(0,0) = r[0];
        rtemp.at<float>(0,1) = r[1];
        rtemp.at<float>(0,2) = r[2];
        ttemp.at<float>(0,0) = t[0];
        ttemp.at<float>(0,1) = t[1];
        ttemp.at<float>(0,2) = t[2];

        rtemp.convertTo(rtemp, CV_64FC1);
        ttemp.convertTo(ttemp, CV_64FC1);
        rvec = rtemp.clone();
        tvec = ttemp.clone();
//        cout<<"rvec:"<<rvec<<endl;
//        cout<<"tvec:"<<tvec<<endl;
//
//        if(rvec.empty()||tvec.empty()){
//            return 0;
//        }else{
//            //在标识上绘制(OpenCV的功能)
//            aruco::drawDetectedMarkers(frame_left, markerCorners, markerIds);
//            aruco::drawAxis(frame_left,       //图像
//                            K,                //相机内参
//                            DistCoef,        //畸变参数
//                            r,
//                            t,             //标识的位姿
//                            0.5*markerLength  //轴长
//                            );
//            return 1;
//        }

    }

    if(rvec.empty()||tvec.empty())
    {
        return 0;
    }else{
        return 1;
    }

}