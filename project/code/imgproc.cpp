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
void findline_lefthand_adaptive(Mat img, int x, int y, float pts[][2], int *num) 
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
void findline_righthand_adaptive(Mat img, int x, int y, float pts[][2], int *num)  
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
// 左边线推中线：中线在“左边线的右侧” dist  
void track_leftline(float pts_in[][2], int num, float pts_out[][2], int approx_num, float dist)  
{   
    if (num <= 0) return;  
    if (approx_num < 1) approx_num = 1;  
  
    for (int i = 0; i < num; i++) {  
        int i0 = limit_int(i - approx_num, 0, num - 1);  
        int i1 = limit_int(i + approx_num, 0, num - 1);  
  
        float dx = pts_in[i1][0] - pts_in[i0][0];  
        float dy = pts_in[i1][1] - pts_in[i0][1];  
        float dn = std::sqrt(dx * dx + dy * dy);  
  
        // 防止除0（局部重复点/静止点）  
        if (dn < 1e-6f) {  
            pts_out[i][0] = pts_in[i][0];  
            pts_out[i][1] = pts_in[i][1];  
            continue;  
        }  
  
        dx /= dn;  
        dy /= dn;  
  
        // 左线 -> 中线（右法向）  
        pts_out[i][0] = pts_in[i][0] - dy * dist;  
        pts_out[i][1] = pts_in[i][1] + dx * dist;  
    }  
}  
  
// 右边线推中线：中线在“右边线的左侧” dist  
void track_rightline(float pts_in[][2], int num, float pts_out[][2], int approx_num, float dist)  
{  
    assert(pts_in && pts_out);  
    if (num <= 0) return;  
    if (approx_num < 1) approx_num = 1;  
  
    for (int i = 0; i < num; i++) {  
        int i0 = limit_int(i - approx_num, 0, num - 1);  
        int i1 = limit_int(i + approx_num, 0, num - 1);  
  
        float dx = pts_in[i1][0] - pts_in[i0][0];  
        float dy = pts_in[i1][1] - pts_in[i0][1];  
        float dn = std::sqrt(dx * dx + dy * dy);  
  
        if (dn < 1e-6f) {  
            pts_out[i][0] = pts_in[i][0];  
            pts_out[i][1] = pts_in[i][1];  
            continue;  
        }  
  
        dx /= dn;  
        dy /= dn;  
  
        // 右线 -> 中线（左法向）  
        pts_out[i][0] = pts_in[i][0] + dy * dist;  
        pts_out[i][1] = pts_in[i][1] - dx * dist;  
    }  
} 
// 逆透视变换矩阵(原图->俯视)

float H[3][3] =  
{  
 {     2.1517,     1.9278,  -104.4764},
{    -0.3644,     4.9223,  -134.9300},
{    -0.0033,     0.0245,     1.0000}
};  

void map_points_to_ipm(float in_pts[][2], int in_num,  float out_pts[][2], int* out_num,  const float Hm[3][3]  )
{  
    int k = 0;  
    for (int i = 0; i < in_num; i++)  
    {  
        float x = (float)in_pts[i][0];  
        float y = (float)in_pts[i][1];  
  
        float w = Hm[2][0] * x + Hm[2][1] * y + Hm[2][2];  
        if (fabs(w) < 1e-6f) continue;  
  
        float u = (Hm[0][0] * x + Hm[0][1] * y + Hm[0][2]) / w;  
        float v = (Hm[1][0] * x + Hm[1][1] * y + Hm[1][2]) / w;  
  
        int ui = (int)(u + 0.5f);  
        int vi = (int)(v + 0.5f);  
  
        if (ui < 0 || ui >= UVC_WIDTH || vi < 0 || vi >= UVC_HEIGHT) continue;  
  
        out_pts[k][0] = ui;  
        out_pts[k][1] = vi;  
        k++;  
        if (k >= EDGELINE_MAX) break;  
    }  
    *out_num = k;  
}  


void add_black_border(cv::Mat &bin, int thickness)  
{  
    CV_Assert(!bin.empty() && bin.type() == CV_8UC1);  
    if (thickness <= 0) return;  
  
    int w = bin.cols, h = bin.rows;  
    thickness = std::min(thickness, std::min(w, h) / 2);  
  
    // 上下边  
    for (int y = 0; y < thickness; y++)  
        memset(bin.ptr(y), 0, w);  
    for (int y = h - thickness; y < h; y++)  
        memset(bin.ptr(y), 0, w);  
  
    // 左右边  
    for (int y = thickness; y < h - thickness; y++)  
    {  
        uint8_t *p = bin.ptr(y);  
        for (int x = 0; x < thickness; x++) p[x] = 0;  
        for (int x = w - thickness; x < w; x++) p[x] = 0;  
    }  
}  
// 作用：将迷宫巡线输出点集压缩为“每行仅一个点”  
// 策略：同一行取最靠内的点（左线取该行最大x，右线取该行最小x）  
// 输入  in_pts/in_num  : 原始迷宫点集  
// 输出  out_pts/out_num: 每行一个点后的点集（按 y 从大到小排序，便于后续控制）  
void compress_line_one_point_per_row(const float in_pts[][2], int in_num,  
                                     float out_pts[][2], int *out_num,  
                                     bool is_left_line)  
{  
    if (!out_num) return;  
    *out_num = 0;  
    if (!in_pts || !out_pts || in_num <= 0) return;  
  
    // 每行是否已有点  
    bool  used[UVC_HEIGHT] = {false};  
    float best_x[UVC_HEIGHT];  
    float best_y[UVC_HEIGHT];  
  
    // 初始化  
    for (int y = 0; y < UVC_HEIGHT; y++) {  
        best_x[y] = is_left_line ? -1e9f : 1e9f;  
        best_y[y] = (float)y;  
    }  
  
    // 扫描原点集，按“每行最优x”更新  
    for (int i = 0; i < in_num; i++)  
    {  
        int yi = (int)(in_pts[i][1] + 0.5f);  
        if (yi < 0 || yi >= UVC_HEIGHT) continue;  
  
        float xi = in_pts[i][0];  
        if (xi < 0 || xi >= UVC_WIDTH) continue;  
  
        if (!used[yi]) {  
            used[yi] = true;  
            best_x[yi] = xi;  
        } else {  
            if (is_left_line) {  
                // 左线取该行最大x（靠中间）  
                if (xi > best_x[yi]) best_x[yi] = xi;  
            } else {  
                // 右线取该行最小x（靠中间）  
                if (xi < best_x[yi]) best_x[yi] = xi;  
            }  
        }  
    }  
  
    // 按 y 从大到小输出（靠近车体优先）  
    int k = 0;  
    for (int y = UVC_HEIGHT - 1; y >= 0; y--)  
    {  
        if (!used[y]) continue;  
        out_pts[k][0] = best_x[y];  
        out_pts[k][1] = best_y[y];  
        k++;  
        if (k >= EDGELINE_MAX) break;  
    }  
  
    *out_num = k;  
}  
// 输入：已压缩后的左右边线（每行至多一个点）  
// 输出：中线点集（同样每行一个点）  
void build_midline_from_compressed_lr(const float left_pts[][2], int left_num,  
                                      const float right_pts[][2], int right_num,  
                                      float mid_pts[][2], int *mid_num)  
{  
    if (!mid_num) return;  
    *mid_num = 0;  
    if (!left_pts || !right_pts || !mid_pts) return;  
    if (left_num <= 0 || right_num <= 0) return;  
  
    int k = 0;  
  
    // 左右都按 y 从大到小，双指针按 y 对齐  
    int i = 0, j = 0;  
    while (i < left_num && j < right_num && k < EDGELINE_MAX)  
    {  
        int yl = (int)(left_pts[i][1] + 0.5f);  
        int yr = (int)(right_pts[j][1] + 0.5f);  
  
        if (yl == yr)  
        {  
            mid_pts[k][0] = 0.5f * (left_pts[i][0] + right_pts[j][0]);  
            mid_pts[k][1] = (float)yl;  
            k++; i++; j++;  
        }  
        else if (yl > yr)  
        {  
            i++; // 左点更靠下，向上追  
        }  
        else  
        {  
            j++; // 右点更靠下，向上追  
        }  
    }  
  
    *mid_num = k;  
}  


