#include <iostream>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

/*
功能：检测多个球，f返回位置信息
利用全局变量：g_ball_color
!！加入：可信度（在检测圆时从不敏感参数开始，返回最终检测到球时候的参数可信程度）
*/

int CheckBall(int color);

