# 

## Overview
This example 
```cpp
// Query frame size (width and height)
resault from : COLOR_BGR2RGB
terminate called after throwing an instance of 'cv::Exception'
  what():  OpenCV(4.2.0) /home/steven/Library/opencv-4.2.0/modules/imgproc/src/bilateral_filter.dispatch.cpp:166: error: (-215:Assertion failed) (src.type() == CV_8UC1 || src.type() == CV_8UC3) && src.data != dst.data in function 'bilateralFilter_8u'
[1]    23240 abort (core dumped)  ../bin/Test
``` 
调整好了参数   

程序修改在这儿

rs_option 中有调整realsense参数的方法   
'''cpp
rs2_set_option(const rs2_sensor* sensor, rs2_option option, float value, rs2_error** error);
'''
