#include <iostream>
#include <string>
#include <cmath>

//#include "ConstVariate.h"

/* #include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv; */

extern float g_dpth_ppx, g_dpth_ppy , g_dpth_fx , g_dpth_fy;
extern float g_color_ppx, g_color_ppy , g_color_fx , g_color_fy;

/*
输入：传入点的位置 double a,b
返回：相对于中心的x、y偏转角度
提升：提高精度，加入考虑三角形模型
11-15修正：改为arctan，有现成函数
参数：27.68 19.91
*/

double deviation_angle_x(double a, double b);

double deviation_angle_y(double a, double b);