//搜寻目标颜色数值设定
#define PINK  1
#define WHITE 2
#define BLACK 3
#define BLUE  4
#define GREEN 5

//全局变量，应该加g_

//HSV的参数设定
int hmin_Max = 261;int hmax_Max = 360;
int smin_Max = 47 ;int smax_Max = 255;
int vmin_Max = 0  ;int vmax_Max = 255;

//霍夫参数设定
int hough_minDist = 75;          //圆心之间的最小距离= 70
int hough_canny = 20;            //canny边缘检测算子的高阈值，而低阈值为高阈值的一半。= 100
int hough_addthersold = 40;      //检测阶段圆心的累加器阈值 是否完美的圆形 = 100
int hough_minRadius = 0;         //有默认值0，表示圆半径的最小值= 0
int hough_maxRadius = 30;        //有默认值0，表示圆半径的最大值= 0

//分割时背景的颜色设定
int seg_bkg = BLACK;

//切割图像参数
int roi_region_x1 = 320;
int roi_region_x = 320;
int roi_region_y1 = 240;
int roi_region_y = 240;

//长度参数
float imgRows = 640, imgCols = 480;

//时间记录
double time_main =0;

//相机参数
float g_dpth_ppx = 326.615;
float g_dpth_ppy = 241.493;