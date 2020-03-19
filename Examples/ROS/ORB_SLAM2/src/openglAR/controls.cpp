// Include GLFW
#include <GLFW/glfw3.h>
extern GLFWwindow* window;

#define GLM_FORCE_RADIANS
// Include GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
using namespace glm;

#include "controls.hpp"

#include <glm/gtc/type_ptr.hpp>
#include <iostream>

glm::mat4 ModelMatrix;
glm::mat4 TranslateMatrix;
glm::mat4 ScalingMatrix;
glm::mat4 ProjectionMatrix;
glm::mat4 ProjectionMatrix1;

glm::mat4 getModelMatrix()
{
	return ModelMatrix;
}
glm::mat4 getProjectionMatrix()
{
	return ProjectionMatrix;
}

glm::mat4 getProjectionMatrix1()
{
    return ProjectionMatrix1;
}

//float scale_factor = 1;
float scale_factor = 0.007;
float rotation_x = 0;
float rotation_y = 0;
float translate_x = 0;
float translate_y = 0;
float translate_z = 0;


float speed = 0.0005f; // 3 units / second
float rotate_speed = 30.0f;
float translate_speed = 0.2f;

void computeMatricesFromInputs(bool slamMode)
{
    //因为模型是面朝下倒着的，要做一个旋转让它站起来
    ModelMatrix = glm::rotate(glm::mat4(1.0), glm::radians(180.0f), glm::vec3( -1, 0, 0));

	// glfwGetTime is called only once, the first time this function is called
	static double lastTime = glfwGetTime();

	// Compute time difference between current and last frame
	double currentTime = glfwGetTime();
	float deltaTime = float(currentTime - lastTime);

	// 放大
	if (glfwGetKey( window, GLFW_KEY_UP ) == GLFW_PRESS)
	{
		scale_factor += deltaTime * speed;
	}
	// 缩小
	if (glfwGetKey( window, GLFW_KEY_DOWN ) == GLFW_PRESS)
	{
		scale_factor -= deltaTime * speed;
	}
	//翻转
	if (glfwGetKey( window, GLFW_KEY_KP_7 ) == GLFW_PRESS)
	{
        rotation_x += deltaTime * rotate_speed;
	}
	//翻转
	if (glfwGetKey( window, GLFW_KEY_KP_9 ) == GLFW_PRESS)
	{
        rotation_x -= deltaTime * rotate_speed;
	}
    //旋转
    if (glfwGetKey( window, GLFW_KEY_LEFT ) == GLFW_PRESS)
    {
        rotation_y += deltaTime * rotate_speed;
    }
    //旋转
    if (glfwGetKey( window, GLFW_KEY_RIGHT ) == GLFW_PRESS)
    {
        rotation_y -= deltaTime * rotate_speed;
    }
    //平移
    if (glfwGetKey( window, GLFW_KEY_KP_8 ) == GLFW_PRESS)
    {
        translate_x -= deltaTime * translate_speed;
    }
    if (glfwGetKey( window, GLFW_KEY_KP_2 ) == GLFW_PRESS)
    {
        translate_x += deltaTime * translate_speed;
    }
    if (glfwGetKey( window, GLFW_KEY_KP_6 ) == GLFW_PRESS)
    {
        translate_z -= deltaTime * translate_speed;
    }
    if (glfwGetKey( window, GLFW_KEY_KP_4 ) == GLFW_PRESS)
    {
        translate_z += deltaTime * translate_speed;
    }
    if (glfwGetKey( window, GLFW_KEY_KP_1 ) == GLFW_PRESS)
    {
        translate_y -= deltaTime * translate_speed;
    }
    if (glfwGetKey( window, GLFW_KEY_KP_3 ) == GLFW_PRESS)
    {
        translate_y += deltaTime * translate_speed;
    }

    //位置重置
    if(glfwGetKey( window, GLFW_KEY_KP_0 ) == GLFW_PRESS)
    {
        //scale_factor = 0.007;
        rotation_x = 0;
        rotation_y = 0;
        translate_x = 0;
        translate_y = 0;
    }

    float f_x = 403.575016;
    float f_y = 403.575016;
    float c_x = 310.014473;
    float c_y = 203.087488;

    float width = 640;
    float height = 400;

    float near_plane = 0.01;
    float far_plane = 100;
    
    float projection_matrix[16];
    projection_matrix[0] = 2*f_x/width;
    projection_matrix[1] = 0.0f;
    projection_matrix[2] = 0.0f;
    projection_matrix[3] = 0.0f;
    
    projection_matrix[4] = 0.0f;
    projection_matrix[5] = 2*f_y/height;
    projection_matrix[6] = 0.0f;
    projection_matrix[7] = 0.0f;
    
    projection_matrix[8] = 1.0f - 2*c_x/width;
    projection_matrix[9] = 2*c_y/height - 1.0f;
    projection_matrix[10] = -(far_plane + near_plane)/(far_plane - near_plane);
    projection_matrix[11] = -1.0f;
    
    projection_matrix[12] = 0.0f;
    projection_matrix[13] = 0.0f;
    projection_matrix[14] = -2.0f*far_plane*near_plane/(far_plane - near_plane);
    projection_matrix[15] = 0.0f;

    //////////////////////////////////////////
    float f_x1 = 403.575016;
    float f_y1 = 403.575016;
    float c_x1 = 310.014473;
    float c_y1 = 203.087488;
    float width1 = 426;
    float height1 = 266;
    float near_plane1 = 0.01;
    float far_plane1 = 100;

    float projection_matrix1[16];
    projection_matrix1[0] = 2*f_x1/width1;
    projection_matrix1[1] = 0.0f;
    projection_matrix1[2] = 0.0f;
    projection_matrix1[3] = 0.0f;

    projection_matrix1[4] = 0.0f;
    projection_matrix1[5] = 2*f_y1/height1;
    projection_matrix1[6] = 0.0f;
    projection_matrix1[7] = 0.0f;

    projection_matrix1[8] = 1.0f - 2*c_x1/width1;
    projection_matrix1[9] = 2*c_y1/height1 - 1.0f;
    projection_matrix1[10] = -(far_plane1 + near_plane1)/(far_plane1 - near_plane1);
    projection_matrix1[11] = -1.0f;

    projection_matrix1[12] = 0.0f;
    projection_matrix1[13] = 0.0f;
    projection_matrix1[14] = -2.0f*far_plane1*near_plane1/(far_plane1 - near_plane1);
    projection_matrix1[15] = 0.0f;
    
    ProjectionMatrix = glm::make_mat4(projection_matrix);
    ProjectionMatrix1 = glm::make_mat4(projection_matrix1);
    
    // Model matrix
    ModelMatrix = glm::rotate(ModelMatrix, glm::radians(rotation_x), glm::vec3( 1, 0, 0));
    ModelMatrix = glm::rotate(ModelMatrix, glm::radians(rotation_y), glm::vec3( 0, 1, 0));
    //下面模型做了一个平移。
    TranslateMatrix = glm::translate(glm::mat4(1.0f), glm::vec3(translate_x,translate_y,translate_z));
    ScalingMatrix = glm::scale(glm::mat4(1.0f), glm::vec3(scale_factor));
    ModelMatrix = TranslateMatrix * ModelMatrix * ScalingMatrix;

	// For the next frame, the "last time" will be "now"
	lastTime = currentTime;
}
