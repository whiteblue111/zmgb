 #include "zf_common_headfile.hpp"
 #include "imgproc.hpp"
 #include <opencv2/opencv.hpp>  
 #include <cstdint>  
 #include <vector>  
 using namespace cv;
 Edge_line left_line={0};

/* 前进方向定义：
 *   0
 * 3   1
 *   2
 */
const int dir_front[4][2] = {{0,  -1},
                            {1,  0},
                            {0,  1},
                            {-1, 0}};
const int dir_frontleft[4][2] = {{-1, -1},
                                {1,  -1},
                                {1,  1},
                                {-1, 1}};
const int dir_frontright[4][2] = {{1,  -1},
                                {1,  1},
                                {-1, 1},
                                {-1, -1}};
 // 左手迷宫巡线
void findline_lefthand_adaptive(Mat img, int x, int y, int pts[][2], int *num) 
{
    uint8_t *ptr;
    int half = 1;
    int step = 0, dir = 0, turn = 0;
    while (step < *num && half < x && x < img.cols - half - 1 && half < y && y < img.rows - half - 1 && turn < 4) {
        int local_thres = 125;
        ptr = img.ptr(y);
        int current_value = ptr[x];

        ptr=img.ptr(y+dir_front[dir][1]);
        int front_value = ptr[x+dir_front[dir][0]];

        ptr = img.ptr(y + dir_frontleft[dir][1]);  
        int frontleft_value =ptr[x + dir_frontleft[dir][0]];

        if (front_value < local_thres) {
            dir = (dir + 1) % 4;
            turn++;
        } else if (frontleft_value < local_thres) {
            x += dir_front[dir][0];
            y += dir_front[dir][1];
            pts[step][0] = x;
            pts[step][1] = y;
            step++;
            turn = 0;
        } else {
            x += dir_frontleft[dir][0];
            y += dir_frontleft[dir][1];
            dir = (dir + 3) % 4;
            pts[step][0] = x;
            pts[step][1] = y;
            step++;
            turn = 0;
        }
    }
    *num = step;
}

// 右手迷宫巡线
void findline_righthand_adaptive(const cv::Mat& img, int x, int y, int pts[][2], int *num)  
{  
    CV_Assert(!img.empty() && img.type() == CV_8UC1);  
    CV_Assert(num && *num > 0);  
  
    uint8_t* ptr = nullptr;  
    int half = 1;  
    int step = 0, dir = 0, turn = 0;  
  
    while (step < *num &&  
           half < x && x < img.cols - half - 1 &&  
           half < y && y < img.rows - half - 1 &&  
           turn < 4)  
    {  
        int local_thres = 125;  
  
        ptr = (uint8_t*)img.ptr(y);  
        int current_value = ptr[x];  
        (void)current_value;  
  
        ptr = (uint8_t*)img.ptr(y + dir_front[dir][1]);  
        int front_value = ptr[x + dir_front[dir][0]];  
  
        ptr = (uint8_t*)img.ptr(y + dir_frontright[dir][1]);  
        int frontright_value = ptr[x + dir_frontright[dir][0]];  
  
        if (front_value < local_thres) {  
            dir = (dir + 3) % 4;  
            turn++;  
        } else if (frontright_value < local_thres) {  
            x += dir_front[dir][0];  
            y += dir_front[dir][1];  
            pts[step][0] = x;  
            pts[step][1] = y;  
            step++;  
            turn = 0;  
        } else {  
            x += dir_frontright[dir][0];  
            y += dir_frontright[dir][1];  
            dir = (dir + 1) % 4;  
            pts[step][0] = x;  
            pts[step][1] = y;  
            step++;  
            turn = 0;  
        }  
    }  
  
    *num = step;  
}  

//找巡左线的起始点
void find_left_base(Mat img,int *x, int *y)
{
    uint8_t *ptr = img.ptr(*y);
    for(int w = *x;w>1;w--)
    {
        if(ptr[w] == 255 && ptr[w-1] == 0)
        {
            *x = w;
            return;
        }
    }
    *x = 1;
}
//找巡右线的起始点
void find_right_base(Mat img,int *x, int *y)
{
    uint8_t *ptr = img.ptr(*y);
    for(int w = *x;w<UVC_WIDTH-1;w++)
    {
        if(ptr[w] == 255 && ptr[w+1] == 0)
        {
            *x = w;
            return;
        }
    }
    *x = UVC_WIDTH-2;
}



// 点集三角滤波
void blur_points(float pts_in[][2], int num, float pts_out[][2], int kernel) 
{
    assert(kernel % 2 == 1);
    int half = kernel / 2;
    for (int i = 0; i < num; i++) 
    {
        pts_out[i][0] = pts_out[i][1] = 0;
        for (int j = -half; j <= half; j++) {
            pts_out[i][0] += pts_in[limit_int(i + j, 0, num - 1)][0] * (half + 1 - abs(j));
            pts_out[i][1] += pts_in[limit_int(i + j, 0, num - 1)][1] * (half + 1 - abs(j));
        }
        pts_out[i][0] /= (2 * half + 2) * (half + 1) / 2;
        pts_out[i][1] /= (2 * half + 2) * (half + 1) / 2;
    }
}
// 点集等距采样  使走过的采样前折线段的距离为`dist`
void resample_points(float pts_in[][2], int num1, float pts_out[][2], int *num2, float dist)
{
    float remain = 0.f;
    int len = 0;
    for(int i=0; i<num1-1 && len < *num2; i++)
    {
        float x0 = pts_in[i][0];
        float y0 = pts_in[i][1];
        float dx = pts_in[i+1][0] - x0;
        float dy = pts_in[i+1][1] - y0;
        float dn = sqrt(dx*dx+dy*dy);
        dx /= dn;
        dy /= dn;

        while(remain < dn && len < *num2)
        {
            x0 += dx * remain;
            pts_out[len][0] = x0;
            y0 += dy * remain;
            pts_out[len][1] = y0;
            
            len++;
            dn -= remain;
            remain = dist;
        }
        remain -= dn;
    }
    *num2 = len;
}