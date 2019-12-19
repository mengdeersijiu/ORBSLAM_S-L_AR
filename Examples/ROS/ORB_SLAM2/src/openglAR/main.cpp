//standard includes
#include <stdio.h>
#include <string.h>
#include <ctime>
#include <chrono>
#include <thread>

#include <vector>

//opencv includes
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Include OpenCV
#include <opencv2/opencv.hpp>

// Include GLEW
#include <GL/glew.h>

// Include GLFW
#include <GLFW/glfw3.h>
GLFWwindow* window;
GLFWwindow* window2;
#define GLM_FORCE_RADIANS
// Include GLM
#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>

using namespace glm;
using namespace cv;

#include "shader.hpp"
#include "texture.hpp"
#include "controls.hpp"
#include "objloader.hpp"
#include "orb_slam.h"
#include "planar_tracking.h"

using namespace std::chrono;
using namespace std;

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iomanip>
#include"../../../include/System.h"

#include "mynteye/logger.h"
#include "mynteye/device/device.h"
#include "mynteye/device/utils.h"
#include "mynteye/util/times.h"
#include "mynteye/api/api.h"

MYNTEYE_USE_NAMESPACE

//#include<ros/ros.h>
//#include <cv_bridge/cv_bridge.h>
//#include <message_filters/subscriber.h>
//#include <message_filters/time_synchronizer.h>
//#include <message_filters/sync_policies/approximate_time.h>

cv::Mat image_split(cv::Mat img,CvRect rect);
cv::Mat image_processL(cv::Mat img);
cv::Mat image_processR(cv::Mat img);
void framebuffer_size_callback(GLFWwindow* window, int width, int height);// 当窗口大小改变时回调函数

int main( void )
{

    vector<double> vTimeStamp;

    // Initialise Tracking System
    bool success = initTracking1("/home/mxy/Downloads/ORB_SLAM2-map_save_load_and_bin_voc/config/mynteye_s2110_stereo_2.yaml");
    cv::Mat K = getCameraMatrix();
    cv::Mat DistCoef = getDistorsion();
    cv::Mat M1l,M2l,M1r,M2r;
    M1l = getM1l();
    M2l = getM2l();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM("/home/mxy/Downloads/ORB_SLAM2-map_save_load_and_bin_voc/Vocabulary/ORBvoc.bin",
            "/home/mxy/Downloads/ORB_SLAM2-map_save_load_and_bin_voc/config/mynteye_s2110_stereo_2.yaml",
            ORB_SLAM2::System::STEREO, true);

    if (!success)
        return 0;

    // Initialise GLFW
    if( !glfwInit() )
    {
        fprintf( stderr, "Failed to initialize GLFW\n" );
        getchar();
        return -1;
    }

    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    //glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy; should not be needed
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    //全屏显示
    bool isFullScreen = true;
    int monitorCount;
    GLFWmonitor** pMonitor = isFullScreen ? glfwGetMonitors(&monitorCount) : NULL;

    std::cout << "Screen number is " << monitorCount << std::endl;

    int holographic_screen = -1;
    for(int i=0; i<monitorCount; i++)
    {
        int screen_x, screen_y;
        const GLFWvidmode * mode = glfwGetVideoMode(pMonitor[i]);
        screen_x = mode->width;
        screen_y = mode->height;

        std::cout << "Screen size is X = " << screen_x << ", Y = " << screen_y << std::endl;
        if(screen_x==1920 && screen_y==1080)
        {
            holographic_screen = i;
        }
    }
    std::cout << holographic_screen << std::endl;
//////////创建窗口1
    window = glfwCreateWindow( 1280, 800, "SLAM-AR", NULL, NULL);
    if( window == NULL )
    {
        fprintf( stderr, "Failed to open GLFW window. If you have an Intel GPU, they are not 3.3 compatible. Try the 2.1 version of the tutorials.\n" );
        getchar();
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    printf("OpenGL version supported by this platform (%s): \n", glGetString(GL_VERSION));

    // Initialize GLEW
    glewExperimental = GL_TRUE; // Needed for core profile
    if (glewInit() != GLEW_OK)
    {
        fprintf(stderr, "Failed to initialize GLEW\n");
        getchar();
        glfwTerminate();
        return -1;
    }

    // Ensure we can capture the escape key being pressed below
    glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);

    // Dark blue background
    glClearColor(0.0f, 0.0f, 0.4f, 0.0f);

    // 开启深度测试
    glEnable(GL_DEPTH_TEST);
    // Accept fragment if it closer to the camera than the former one
    glDepthFunc(GL_LESS);

    // Cull triangles which normal is not towards the camera
    glEnable(GL_CULL_FACE);

    //VAO
    GLuint VertexArrayID;
    glGenVertexArrays(1, &VertexArrayID);
    glBindVertexArray(VertexArrayID);

    // Create and compile our GLSL program from the shaders
    GLuint programID = LoadShaders( "/home/mxy/Downloads/ORB_SLAM2-map_save_load_and_bin_voc/Examples/ROS/ORB_SLAM2/src/openglAR/Shaders/TransformVertexShader.vertexshader",
            "/home/mxy/Downloads/ORB_SLAM2-map_save_load_and_bin_voc/Examples/ROS/ORB_SLAM2/src/openglAR/Shaders/TextureFragmentShader.fragmentshader" );

    // Get a handle for our "MVP" uniform
    GLint MatrixID = glGetUniformLocation(programID, "MVP");

    // Load the texture
    int width, height;
    //GLuint Texture = png_texture_load("/home/mxy/Downloads/ORB_SLAM2-map_save_load_and_bin_voc/Examples/ROS/ORB_SLAM2/src/openglAR/SpongeBob/spongebob.png", &width, &height);
    GLuint Texture = png_texture_load("/home/mxy/Downloads/ORB_SLAM2-map_save_load_and_bin_voc/Examples/ROS/ORB_SLAM2/src/openglAR/SpongeBob/new_sand_table_2k.png", &width, &height);
    GLuint Texture1;
    glGenTextures(1, &Texture1);

    // Get a handle for our "myTextureSampler" uniform
    GLuint TextureID  = glGetUniformLocation(programID, "myTextureSampler");

    // Read our .obj file
    std::vector<glm::vec3> vertices;
    std::vector<glm::vec2> uvs;
    std::vector<glm::vec3> normals; // Won't be used at the moment.
    //loadOBJ("/home/mxy/Downloads/ORB_SLAM2-map_save_load_and_bin_voc/Examples/ROS/ORB_SLAM2/src/openglAR/SpongeBob/spongebob.obj", vertices, uvs, normals);
    loadOBJ("/home/mxy/Downloads/ORB_SLAM2-map_save_load_and_bin_voc/Examples/ROS/ORB_SLAM2/src/openglAR/SpongeBob/unity_ex_OBJ.obj", vertices, uvs, normals);

    // Load it into a VBO
    GLuint vertexbuffer;
    glGenBuffers(1, &vertexbuffer);//此处第一个参数为生成缓存个数，可以一次生成多个缓存
    //顶点缓冲对象的缓冲类型是GL_ARRAY_BUFFER 将VBO绑定到GL_ARRAY_BUFFER目标 说明缓冲类型
    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
    // 把顶点数据复制到缓冲的内存中 sizeof计算顶点数据大小 数据不会改变GL_STATIC_DRAW，size单位为Byte.
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(glm::vec3), &vertices[0], GL_STATIC_DRAW);

    GLuint uvbuffer;
    glGenBuffers(1, &uvbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
    glBufferData(GL_ARRAY_BUFFER, uvs.size() * sizeof(glm::vec2), &uvs[0], GL_STATIC_DRAW);


    static const GLfloat g_vertex_buffer_data[] =
    {
            1.0f, -1.0f, 0.0f,
            1.0f, 1.0f, 0.0f,
            -1.0f,  -1.0f, 0.0f,
            1.0f, 1.0f, 0.0f,
            -1.0f,  1.0f, 0.0f,
            -1.0f,  -1.0f, 0.0f,
    };

    static const GLfloat g_uv_buffer_data[] =
    {
            1.0f, 0.0f,
            1.0f, 1.0f,
            0.0f,  0.0f,
            1.0f, 1.0f,
            0.0f,  1.0f,
            0.0f,  0.0f,
    };

    GLuint colorbuffer;
    glGenBuffers(1, &colorbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(g_uv_buffer_data), g_uv_buffer_data, GL_STATIC_DRAW);

    GLuint cubebuffer;
    glGenBuffers(1, &cubebuffer);
    glBindBuffer(GL_ARRAY_BUFFER, cubebuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(g_vertex_buffer_data), g_vertex_buffer_data, GL_STATIC_DRAW);

//////////创建窗口2
    window2 = glfwCreateWindow(1280, 800, "ARUCO_MYNT_AR-2", NULL, window);
    if (window2 == NULL)
    {
        std::cout << "Failed to create the 2nd GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window2);
    glfwSetFramebufferSizeCallback(window2, framebuffer_size_callback);
    // Ensure we can capture the escape key being pressed below
    glfwSetInputMode(window2, GLFW_STICKY_KEYS, GL_TRUE);

    // Dark blue background
    //glClearColor(0.0f, 0.0f, 0.4f, 0.0f);

    // 开启深度测试
    glEnable(GL_DEPTH_TEST);
    // Accept fragment if it closer to the camera than the former one
    glDepthFunc(GL_LESS);

    // Cull triangles which normal is not towards the camera
    glEnable(GL_CULL_FACE);

    //VAO
    //GLuint VertexArrayID;
    glGenVertexArrays(1, &VertexArrayID);
    glBindVertexArray(VertexArrayID);

    // Create and compile our GLSL program from the shaders
    //GLuint programID = LoadShaders( "/home/mxy/Downloads/ORB_SLAM2-map_save_load_and_bin_voc/Examples/ROS/ORB_SLAM2/src/openglAR/Shaders/TransformVertexShader.vertexshader",
                                    //"/home/mxy/Downloads/ORB_SLAM2-map_save_load_and_bin_voc/Examples/ROS/ORB_SLAM2/src/openglAR/Shaders/TextureFragmentShader.fragmentshader" );

    // Get a handle for our "MVP" uniform
    //GLint MatrixID = glGetUniformLocation(programID, "MVP");

    // Load the texture
    //int width, height;
    //GLuint Texture = png_texture_load("/home/mxy/Downloads/ORB_SLAM2-map_save_load_and_bin_voc/Examples/ROS/ORB_SLAM2/src/openglAR/SpongeBob/spongebob.png", &width, &height);
    //GLuint Texture = png_texture_load("/home/mxy/Downloads/ORB_SLAM2-map_save_load_and_bin_voc/Examples/ROS/ORB_SLAM2/src/openglAR/SpongeBob/new_sand_table_2k.png", &width, &height);
    //GLuint Texture1;
    //glGenTextures(1, &Texture1);

    // Get a handle for our "myTextureSampler" uniform
    //GLuint TextureID  = glGetUniformLocation(programID, "myTextureSampler");

    // Read our .obj file
    //std::vector<glm::vec3> vertices;
    //std::vector<glm::vec2> uvs;
    //std::vector<glm::vec3> normals; // Won't be used at the moment.
    //loadOBJ("/home/mxy/Downloads/ORB_SLAM2-map_save_load_and_bin_voc/Examples/ROS/ORB_SLAM2/src/openglAR/SpongeBob/spongebob.obj", vertices, uvs, normals);
    loadOBJ("/home/mxy/Downloads/ORB_SLAM2-map_save_load_and_bin_voc/Examples/ROS/ORB_SLAM2/src/openglAR/SpongeBob/unity_ex_OBJ.obj", vertices, uvs, normals);

    // Load it into a VBO
    //GLuint vertexbuffer;
    glGenBuffers(1, &vertexbuffer);//此处第一个参数为生成缓存个数，可以一次生成多个缓存
    //顶点缓冲对象的缓冲类型是GL_ARRAY_BUFFER 将VBO绑定到GL_ARRAY_BUFFER目标 说明缓冲类型
    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
    // 把顶点数据复制到缓冲的内存中 sizeof计算顶点数据大小 数据不会改变GL_STATIC_DRAW，size单位为Byte.
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(glm::vec3), &vertices[0], GL_STATIC_DRAW);

    //GLuint uvbuffer;
    glGenBuffers(1, &uvbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
    glBufferData(GL_ARRAY_BUFFER, uvs.size() * sizeof(glm::vec2), &uvs[0], GL_STATIC_DRAW);


//    static const GLfloat g_vertex_buffer_data[] =
//            {
//                    1.0f, -1.0f, 0.0f,
//                    1.0f, 1.0f, 0.0f,
//                    -1.0f,  -1.0f, 0.0f,
//                    1.0f, 1.0f, 0.0f,
//                    -1.0f,  1.0f, 0.0f,
//                    -1.0f,  -1.0f, 0.0f,
//            };
//
//    static const GLfloat g_uv_buffer_data[] =
//            {
//                    1.0f, 0.0f,
//                    1.0f, 1.0f,
//                    0.0f,  0.0f,
//                    1.0f, 1.0f,
//                    0.0f,  1.0f,
//                    0.0f,  0.0f,
//            };

//    GLuint colorbuffer;
    glGenBuffers(1, &colorbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(g_uv_buffer_data), g_uv_buffer_data, GL_STATIC_DRAW);

//    GLuint cubebuffer;
    glGenBuffers(1, &cubebuffer);
    glBindBuffer(GL_ARRAY_BUFFER, cubebuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(g_vertex_buffer_data), g_vertex_buffer_data, GL_STATIC_DRAW);


    Ptr<ORB> orb = ORB::create();
    orb->setScoreType(cv::ORB::FAST_SCORE);
    orb->setMaxFeatures(1000);
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming(2)");
    Tracker orb_tracker(orb, matcher, K, DistCoef);
/*    orb_tracker.setFirstFrame("/home/mxy/Downloads/ORB_SLAM2-map_save_load_and_bin_voc/Examples/ROS/ORB_SLAM2/src/openglAR/slam14jiang.png");

    vector<float> vTimesTrack;
    cout << endl << "-------" << endl;
    cout << "Start processing camera ..." << endl;
    cv::Mat imageALL, imLeft, imRight, imLeftRect, imRightRect;
    cv::VideoCapture cap1(0);
    cap1.set(CV_CAP_PROP_FRAME_WIDTH,2560);
    cap1.set(CV_CAP_PROP_FRAME_HEIGHT,960);
    cap1.set(CV_CAP_PROP_FPS, 30);*/

    auto &&api = API::Create(0, nullptr);
    if (!api) return 1;
    auto request = api->GetStreamRequest();//不知道什么原因这么配置无效
    request.height = 400;
    request.width = 1280;
    request.fps = 60;

//    bool ok;
//    auto &&request = api->SelectStreamRequest(&ok);//手动选择

    api->ConfigStreamRequest(request);
    auto in_left = api->GetIntrinsicsBase(Stream::LEFT);//内参基类
    auto in_right = api->GetIntrinsicsBase(Stream::RIGHT);
    if (in_left->calib_model() == CalibrationModel::PINHOLE)
    {//由基类指针转换为指定类型指针
        in_left = std::dynamic_pointer_cast<IntrinsicsPinhole>(in_left);
        in_right = std::dynamic_pointer_cast<IntrinsicsPinhole>(in_right);
    }
    else if (in_left->calib_model() == CalibrationModel::KANNALA_BRANDT)
    {
        in_left = std::dynamic_pointer_cast<IntrinsicsEquidistant>(in_left);
        in_right = std::dynamic_pointer_cast<IntrinsicsEquidistant>(in_right);
    }
    else
    {
        LOG(INFO) << "UNKNOW CALIB MODEL (未知校正模型) .";
        return 0;
    }
    api->EnableStreamData(Stream::LEFT);
    api->EnableStreamData(Stream::RIGHT);
    api->Start(Source::VIDEO_STREAMING);

    bool slamMode = 1;
    bool bLocalizationMode = false;
    glm::mat4 initModelMatrix1 = glm::mat4(1.0);
    int ni = 0;
    cv::Mat CameraPose;
    cv::Mat Tpw;
    glm::mat4 ViewMatrix;
    glm::mat4 ProjectionMatrix;
    glm::mat4 ModelMatrix;
    cv::Mat LeftUndistort;

    do
    {
        ni++;
/*        cap1 >> imageALL;
        if(imageALL.empty())
        {
            cerr << endl << "Check Camera!! "<< endl;
            return 1;
        }
        imLeft = image_processL(imageALL);
        imRight = image_processR(imageALL);
        time_point<system_clock> now = system_clock::now();
        double tframe = now.time_since_epoch().count();
        vTimeStamp.push_back(tframe);*/

        api->WaitForStreams();
        auto &&left_data = api->GetStreamData(Stream::LEFT);
        auto &&right_data = api->GetStreamData(Stream::RIGHT);

/////////////////窗口1
        glfwMakeContextCurrent(window);
        // Clear the screen
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Use our shader
        glUseProgram(programID);

        // Bind our texture in Texture Unit 0
        glActiveTexture(GL_TEXTURE1);
        cv::remap(left_data.frame,LeftUndistort,M1l,M2l,cv::INTER_LINEAR);
        Texture1 = loadframe_opencv(LeftUndistort, Texture1);
        //Texture1 = loadframe_opencv(imLeft, Texture1);
        glBindTexture(GL_TEXTURE_2D, Texture1);

        // Set our "myTextureSampler" sampler to user Texture Unit 0
        glUniform1i(TextureID, 1);
        glm::mat4 MVP = glm::mat4(1.0);
        glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);

        glDisable(GL_DEPTH_TEST);

        // 1rst attribute buffer : vertices
        glEnableVertexAttribArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, cubebuffer);
        glVertexAttribPointer(
                0,                  // attribute
                3,                  // size
                GL_FLOAT,           // type
                GL_FALSE,           // normalized?
                0,                  // stride
                (void*)0            // array buffer offset
        );

        // 2nd attribute buffer : UVs
        glEnableVertexAttribArray(1);
        glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
        glVertexAttribPointer(
                1,                                // attribute
                2,                                // size
                GL_FLOAT,                         // type
                GL_FALSE,                         // normalized?
                0,                                // stride
                (void*)0                          // array buffer offset
        );
        glDrawArrays(GL_TRIANGLES, 0, 6 );

        glDisableVertexAttribArray(0);
        glDisableVertexAttribArray(1);

//        glfwPollEvents();
//        if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)
//        {
//            slamMode = true;
//        }

//        //success = orb_tracker.process(imLeft, slamMode);
//        success = orb_tracker.process(left_data.frame, slamMode);
//
//        //没得到相机到marker的变换时
//        if (!success)
//        {
//            glfwPollEvents();
//            if (glfwGetKey(window, GLFW_KEY_ESCAPE ) != GLFW_PRESS &&
//                glfwWindowShouldClose(window) == 0)
//            {
//                glfwSwapBuffers(window);
//                continue;
//            } else
//                break;
//        }

        //glm::mat4 ViewMatrix;
        // Compute the MVP matrix from keyboard and mouse input
        computeMatricesFromInputs();
        ProjectionMatrix = getProjectionMatrix();
        ModelMatrix = getModelMatrix();

        if(slamMode)
        {
            //cv::Mat CameraPose = SLAM.TrackStereoOriginalIm(imLeft, imRight, tframe);
            CameraPose = SLAM.TrackStereoOriginalIm(left_data.frame, right_data.frame, left_data.img->timestamp*0.00001f);
            //cv::Mat viewMatrix = cv::Mat::zeros(4, 4, CV_64FC1);

            //用Tcw构造ViewMatrix
            if(!CameraPose.empty())
            {
                trackStereo(CameraPose);
                ViewMatrix = getViewMatrix(slamMode);
                //cout<<"SLAM ON "<<ni<<endl;
            }
            else
            {
                ViewMatrix = glm::mat4(1.0);
            }

            //按enter更新Tpw
            glfwPollEvents();
            if(glfwGetKey(window, GLFW_KEY_ENTER) == GLFW_PRESS)
            {
                SLAM.PlaneSwitch();
                Tpw = SLAM.GetPlaneMatrix();
                cout<<"enter"<<endl;
                if(!Tpw.empty())
                {
                    initModelMatrix1 = orb_tracker.getInitModelMatrix1(Tpw);
                }
            }

            //发生回环地图更新时重新计算Tpw
            if(!Tpw.empty())
            {
                bLocalizationMode = SLAM.GetActivateLocalizationMode();
                if(!bLocalizationMode)
                {
                    if(SLAM.MapChanged())
                    {
                        cout << "Map changed. All virtual elements are recomputed!" << endl;
                        Tpw = SLAM.Recompute1();
                        initModelMatrix1 = orb_tracker.getInitModelMatrix1(Tpw);
                    }
                }
            }
            MVP = ProjectionMatrix * ViewMatrix * initModelMatrix1 * ModelMatrix;
        }
        else
        {
            ViewMatrix = getViewMatrix(slamMode);
            glm::mat4 initModelMatrix = orb_tracker.getInitModelMatrix();
            MVP = ProjectionMatrix * ViewMatrix * initModelMatrix * ModelMatrix;
        }

        glEnable(GL_DEPTH_TEST);

        // Send our transformation to the currently bound shader,
        // in the "MVP" uniform
        glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);

        // Bind our texture in Texture Unit 0
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, Texture);

        // Set our "myTextureSampler" sampler to user Texture Unit 0
        glUniform1i(TextureID, 0);

        // 1rst attribute buffer : vertices
        glEnableVertexAttribArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
        glVertexAttribPointer(
                0,                  // attribute
                3,                  // size
                GL_FLOAT,           // type
                GL_FALSE,           // normalized?
                0,                  // stride
                (void*)0            // array buffer offset
        );

        // 2nd attribute buffer : UVs
        glEnableVertexAttribArray(1);
        glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
        glVertexAttribPointer(
                1,                                // attribute
                2,                                // size
                GL_FLOAT,                         // type
                GL_FALSE,                         // normalized?
                0,                                // stride
                (void*)0                          // array buffer offset
        );

        // Draw the triangle !
        glDrawArrays(GL_TRIANGLES, 0, (GLsizei)vertices.size() );

        // Swap buffers
        glfwSwapBuffers(window);

        glfwPollEvents();
        if (glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS)
        {
            //slamMode = false;
            SLAM.Reset();
        }

        glfwPollEvents();


/////////////////窗口2
        glfwMakeContextCurrent(window2);

        // Clear the screen
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Use our shader
        glUseProgram(programID);

        // Bind our texture in Texture Unit 0
        //glActiveTexture(GL_TEXTURE1);

        //glBindTexture(GL_TEXTURE_2D, Texture1);

        // Set our "myTextureSampler" sampler to user Texture Unit 0
        glUniform1i(TextureID, 1);

        glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);

        glDisable(GL_DEPTH_TEST);

        // 1rst attribute buffer : vertices
        glEnableVertexAttribArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, cubebuffer);
        glVertexAttribPointer(
                0,                  // attribute
                3,                  // size
                GL_FLOAT,           // type
                GL_FALSE,           // normalized?
                0,                  // stride
                (void*)0            // array buffer offset
        );

        // 2nd attribute buffer : UVs
        glEnableVertexAttribArray(1);
        glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
        glVertexAttribPointer(
                1,                                // attribute
                2,                                // size
                GL_FLOAT,                         // type
                GL_FALSE,                         // normalized?
                0,                                // stride
                (void*)0                          // array buffer offset
        );
        glDrawArrays(GL_TRIANGLES, 0, 6 );

        glDisableVertexAttribArray(0);
        glDisableVertexAttribArray(1);

        glEnable(GL_DEPTH_TEST);

        // Send our transformation to the currently bound shader,
        // in the "MVP" uniform
        glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);

        // Bind our texture in Texture Unit 0
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, Texture);

        // Set our "myTextureSampler" sampler to user Texture Unit 0
        glUniform1i(TextureID, 0);

        // 1rst attribute buffer : vertices
        glEnableVertexAttribArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
        glVertexAttribPointer(
                0,                  // attribute
                3,                  // size
                GL_FLOAT,           // type
                GL_FALSE,           // normalized?
                0,                  // stride
                (void*)0            // array buffer offset
        );

        // 2nd attribute buffer : UVs
        glEnableVertexAttribArray(1);
        glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
        glVertexAttribPointer(
                1,                                // attribute
                2,                                // size
                GL_FLOAT,                         // type
                GL_FALSE,                         // normalized?
                0,                                // stride
                (void*)0                          // array buffer offset
        );

        // Draw the triangle !
        glDrawArrays(GL_TRIANGLES, 0, (GLsizei)vertices.size() );

        glfwSwapBuffers(window2);
        glfwPollEvents();

    } // Check if the ESC key was pressed or the window was closed
    while( glfwGetKey(window, GLFW_KEY_ESCAPE ) != GLFW_PRESS &&
           glfwWindowShouldClose(window) == 0 );


    // Cleanup VBO and shader
    glDeleteBuffers(1, &vertexbuffer);
    glDeleteBuffers(1, &uvbuffer);
    glDeleteProgram(programID);
    glDeleteTextures(1, &TextureID);
    glDeleteVertexArrays(1, &VertexArrayID);

    // Close OpenGL window and terminate GLFW
    glfwTerminate();

    return 0;
}

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

// 当窗口大小改变时回调函数
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    glViewport(0, 0, width, height);//确保视口大小与新窗口尺寸相匹配，注意视网膜屏幕的尺寸分辨率参数会更高
}