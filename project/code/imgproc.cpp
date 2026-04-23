 #include "zf_common_headfile.hpp"
 #include "imgproc.hpp"
 #include <opencv2/opencv.hpp>  
 #include <cstdint>  
 #include <vector>  
 using namespace cv;
 Edge_line left_line={0};

 //角度
 float angles_l[POINTS_MAX_LEN]     = {0};  
 float angles_nms_l[POINTS_MAX_LEN] = {0};  
 float angles_r[POINTS_MAX_LEN]     = {0};  
 float angles_nms_r[POINTS_MAX_LEN] = {0};  
 float angle_l_max = 0;
 float angle_r_max = 0;
 int angle_l_max_id = 0;
 int angle_r_max_id = 0;



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
// 建议放在 imgproc.cpp 顶部或工具区  
static inline uint8_t safe_pixel(const cv::Mat& img, int x, int y)  
{  
    if (x < 0 || x >= img.cols || y < 0 || y >= img.rows) return 0; // 越界按黑  
    return img.ptr<uint8_t>(y)[x];  
}  
  
// 左线起始点（抗噪版）  
void find_left_base(cv::Mat img, int *x, int *y)  
{  
    CV_Assert(!img.empty() && img.type() == CV_8UC1);  
    if (!x || !y) return;  
  
    const int start_thre = 125;   // 可调  
    const int CHECK_DIS  = 3;     // 可调，连续性判据  
  
    int bx = *x;  
    int by = *y;  
  
    // 裁剪初值  
    bx = limit_int(bx, 0, img.cols - 1);  
    by = limit_int(by, 0, img.rows - 1);  
  
    bool found = false;  
  
    // 若起点偏黑：向右找连续白段（左线常见）  
    if (safe_pixel(img, bx, by) <= start_thre)  
    {  
        int right_limit = img.cols / 2;  
        for (int xx = bx; xx < right_limit; ++xx)  
        {  
            bool ok = true;  
            for (int k = 0; k <= CHECK_DIS; ++k)  
            {  
                if (safe_pixel(img, xx + k, by) <= start_thre) { ok = false; break; }  
            }  
            if (!ok) continue;  
  
            bx = xx;  
            found = true;  
            break;  
        }  
    }  
    else  
    {  
        // 起点偏白：向上找连续黑段（穿过边缘）  
        for (int yy = by; yy >= 1; --yy)  
        {  
            bool ok = true;  
            for (int k = 0; k <= CHECK_DIS; ++k)  
            {  
                if (safe_pixel(img, bx, yy - k) > start_thre) { ok = false; break; }  
            }  
            if (!ok) continue;  
  
            by = yy;  
            found = true;  
            break;  
        }  
    }  
  
    // 兜底（保留你原来的跳变法）  
    if (!found)  
    {  
        uint8_t *ptr = img.ptr(*y);  
        for (int w = *x; w > 1; w--)  
        {  
            if (ptr[w] == 255 && ptr[w - 1] == 0) { bx = w; found = true; break; }  
        }  
    }  
  
    *x = found ? bx : 1;  
    *y = by;  
}  
  
// 右线起始点（对称抗噪版）  
void find_right_base(cv::Mat img, int *x, int *y)  
{  
    CV_Assert(!img.empty() && img.type() == CV_8UC1);  
    if (!x || !y) return;  
  
    const int start_thre = 125;  
    const int CHECK_DIS  = 3;  
  
    int bx = *x;  
    int by = *y;  
  
    bx = limit_int(bx, 0, img.cols - 1);  
    by = limit_int(by, 0, img.rows - 1);  
  
    bool found = false;  
  
    // 若起点偏黑：向左找连续白段（右线对称）  
    if (safe_pixel(img, bx, by) <= start_thre)  
    {  
        int left_limit = img.cols / 2;  
        for (int xx = bx; xx >= left_limit; --xx)  
        {  
            bool ok = true;  
            for (int k = 0; k <= CHECK_DIS; ++k)  
            {  
                if (safe_pixel(img, xx - k, by) <= start_thre) { ok = false; break; }  
            }  
            if (!ok) continue;  
  
            bx = xx;  
            found = true;  
            break;  
        }  
    }  
    else  
    {  
        // 起点偏白：向上找连续黑段  
        for (int yy = by; yy >= 1; --yy)  
        {  
            bool ok = true;  
            for (int k = 0; k <= CHECK_DIS; ++k)  
            {  
                if (safe_pixel(img, bx, yy - k) > start_thre) { ok = false; break; }  
            }  
            if (!ok) continue;  
  
            by = yy;  
            found = true;  
            break;  
        }  
    }  
  
    // 兜底（保留你原来的跳变法）  
    if (!found)  
    {  
        uint8_t *ptr = img.ptr(*y);  
        for (int w = *x; w < img.cols - 2; w++)  
        {  
            if (ptr[w] == 255 && ptr[w + 1] == 0) { bx = w; found = true; break; }  
        }  
    }  
  
    *x = found ? bx : (img.cols - 2);  
    *y = by;  
}  

// // 找巡左线起始点：  
// // 1) 先在当前行从 x 向左找 (255,0) 跳变  
// // 2) 若没找到，则从左边界附近开始，逐行向上找跳变  
// void find_left_base(Mat img,int *x, int *y)  
// {  
//     uint8_t *ptr = img.ptr(*y);  
  
//     // 1) 先在当前行从中间往左找  
//     for (int w = *x; w > 1; w--)  
//     {  
//         if (ptr[w] == 255 && ptr[w - 1] == 0)  
//         {  
//             *x = w;  
//             return;  
//         }  
//     }  
  
//     // 2) 没找到：从左边界开始，逐行向上找  
//     for (int yy = *y - 1; yy >= 1; yy--)  
//     {  
//         ptr = img.ptr(yy);  
//         for (int w = 2; w < img.cols / 2; w++)   // 左半区搜索  
//         {  
//             if (ptr[w] == 255 && ptr[w - 1] == 0)  
//             {  
//                 *x = w;  
//                 *y = yy;   // 记得更新y  
//                 return;  
//             }  
//         }  
//     }  
  
//     // 3) 还没找到就兜底  
//     *x = 1;  
// }  
 

// // 找巡右线起始点：  
// // 1) 先在当前行从 x 向右找 (255,0) 跳变（即 ptr[w]==255 && ptr[w+1]==0）  
// // 2) 若没找到，则从右边界附近开始，逐行向上找跳变  
// void find_right_base(Mat img, int *x, int *y)  
// {  
//     uint8_t *ptr = img.ptr(*y);  
  
//     // 1) 先在当前行从中间往右找  
//     for (int w = *x; w < img.cols - 2; w++)  
//     {  
//         if (ptr[w] == 255 && ptr[w + 1] == 0)  
//         {  
//             *x = w;  
//             return;  
//         }  
//     }  
  
//     // 2) 没找到：从右边界开始，逐行向上找  
//     for (int yy = *y - 1; yy >= 1; yy--)  
//     {  
//         ptr = img.ptr(yy);  
//         for (int w = img.cols - 3; w >= img.cols / 2; w--) // 右半区搜索  
//         {  
//             if (ptr[w] == 255 && ptr[w + 1] == 0)  
//             {  
//                 *x = w;  
//                 *y = yy;   // 记得更新y  
//                 return;  
//             }  
//         }  
//     }  
  
//     // 3) 还没找到就兜底  
//     *x = img.cols - 2;  
// }  




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
    for(int i=0; i < num1-1 && len < *num2; i++)
    {
        float x0 = pts_in[i][0];
        float y0 = pts_in[i][1];
        float dx = pts_in[i+1][0] - x0;
        float dy = pts_in[i+1][1] - y0;
        float dn = sqrtf(dx*dx + dy*dy);  
        if (dn < 1e-6f) continue;  
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
void track_leftline(float pts_in[][2], int num_in, float pts_out[][2],int& num_out, int approx_num, float dist)  
{   
    num_out = 0;
    if (num_in <= 0) return;  
    if (approx_num < 1) approx_num = 1;  
  
    for (int i = 0; i < num_in; i++) {  
        int i0 = limit_int(i - approx_num, 0, num_in - 1);  
        int i1 = limit_int(i + approx_num, 0, num_in - 1);  
  
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
    num_out = num_in; 
}  
  
// 右边线推中线：中线在“右边线的左侧” dist  
void track_rightline(float pts_in[][2], int num_in, float pts_out[][2], int& num_out, int approx_num, float dist)  
{  
    num_out = 0;  
    if (num_in <= 0) return;  
    if (approx_num < 1) approx_num = 1;  
  
    for (int i = 0; i < num_in; i++) {  
        int i0 = limit_int(i - approx_num, 0, num_in - 1);  
        int i1 = limit_int(i + approx_num, 0, num_in - 1);  
  
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
  
        // 右线 -> 中线（左法向）  
        pts_out[i][0] = pts_in[i][0] + dy * dist;  
        pts_out[i][1] = pts_in[i][1] - dx * dist;  
    }  
    num_out = num_in;  
}  

// 作用：  中线归一化
// 作用：  
// 1) 用图像下方锚点统一中线起点；  
// 2) 若中线已到达锚点附近 -> 从最近点截断；  
// 3) 若中线未到锚点 -> 用直线把锚点连接到原中线首点；  
// 4) 最后重新等距采样，得到稳定可控的中线。  
//  
// 依赖：resample_points(), limit_int()  
// 常量：EDGELINE_MAX, UVC_WIDTH, UVC_HEIGHT  
  
void normalize_midline_with_anchor(float pts_in[][2], int in_num, float pts_out[][2], int *out_num)  
{  
    if (!out_num) return;  
    *out_num = 0;  
    if (!pts_in || !pts_out || in_num <= 0) return;  
  
    int outn = 0;  
  
    // 1) 找最靠近锚点y的点  
    int best_i = -1;  
    int best_dy = 1e9;  
    for (int i = 0; i < in_num; i++)  
    {  
        int yi = (int)(pts_in[i][1] + 0.5f);  
        int dy = std::abs(yi - CENTER_BEGIN_Y);  
        if (dy < best_dy) { best_dy = dy; best_i = i; }  
    }  
    if (best_i < 0) return;  
  
    // 2) 已到锚点附近：从该点截断  
    if (best_dy <= 3)  
    {  
        for (int i = best_i; i < in_num && outn < EDGELINE_MAX; i++)  
        {  
            pts_out[outn][0] = pts_in[i][0];  
            pts_out[outn][1] = pts_in[i][1];  
            outn++;  
        }  
    }  
    // 3) 未到锚点：首点强制锚点，后面接原线  
    else  
    {  
        pts_out[outn][0] = (float)CENTER_BEGIN_X;  
        pts_out[outn][1] = (float)CENTER_BEGIN_Y;  
        if (cross_type == CROSS_IN)
         pts_out[outn][1] = (float)(CENTER_BEGIN_Y - 20);  // 十字时锚点稍微往上，避免干扰
        outn++;  
  
        int start_i = 0;  
        if (in_num > 0)  
        {  
            float ddx = pts_in[0][0] - pts_out[outn - 1][0];  
            float ddy = pts_in[0][1] - pts_out[outn - 1][1];  
            if (std::sqrt(ddx * ddx + ddy * ddy) < 0.5f) start_i = 1;  
        }  
  
        for (int i = start_i; i < in_num && outn < EDGELINE_MAX; i++)  
        {  
            pts_out[outn][0] = pts_in[i][0];  
            pts_out[outn][1] = pts_in[i][1];  
            outn++;  
        }  
    }  
  
    *out_num = outn;  
}  
 
  


// 逆透视变换矩阵(原图->俯视)

// float H[3][3] =  
// {  
//  {     2.1517,     1.9278,  -104.4764},
// {    -0.3644,     4.9223,  -134.9300},
// {    -0.0033,     0.0245,     1.0000}
// };  
// //逆透视变换原图到俯视图
// void map_points_to_ipm(float pts_in[][2], int in_num,  float pts_out[][2], int* out_num,  const float Hm[3][3]  )
// {  
//     int k = 0;  
//     for (int i = 0; i < in_num; i++)  
//     {  
//         float x = (float)pts_in[i][0];  
//         float y = (float)pts_in[i][1];  
  
//         float w = Hm[2][0] * x + Hm[2][1] * y + Hm[2][2];  
//         if (fabs(w) < 1e-6f) continue;  
  
//         float u = (Hm[0][0] * x + Hm[0][1] * y + Hm[0][2]) / w;  
//         float v = (Hm[1][0] * x + Hm[1][1] * y + Hm[1][2]) / w;  
  
//         int ui = (int)(u + 0.5f);  
//         int vi = (int)(v + 0.5f);  
  
//         if (ui < 0 || ui >= UVC_WIDTH || vi < 0 || vi >= UVC_HEIGHT) continue;  
  
//         pts_out[k][0] = ui;  
//         pts_out[k][1] = vi;  
//         k++;  
//         if (k >= EDGELINE_MAX) break;  
//     }  
//     *out_num = k;  
// }  

//加黑框便于迷宫巡线
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
//加图像下半部分黑框
 void add_black_border_half(cv::Mat &bin, int thickness)
 {
    CV_Assert(!bin.empty() && bin.type() == CV_8UC1);  
    if (thickness <= 0) return;  
  
    int w = bin.cols, h = bin.rows;  
    thickness = std::min(thickness, std::min(w, h) / 2);  
  
    // 下边   
    for (int y = h - thickness; y < h; y++)  
        memset(bin.ptr(y), 0, w);  
  
    // 左右半边  
    for (int y = h / 2; y < h - thickness; y++)  
    {  
        uint8_t *p = bin.ptr(y);  
        for (int x = 0; x < thickness; x++) p[x] = 0;  
        for (int x = w - thickness; x < w; x++) p[x] = 0;  
    }  
 }
// 作用：将迷宫巡线输出点集压缩为“每行仅一个点”  
// 策略：同一行取最靠内的点（左线取该行最大x，右线取该行最小x）  
// 输入  pts_in/in_num  : 原始迷宫点集  
// 输出  pts_out/out_num: 每行一个点后的点集（按 y 从大到小排序，便于后续控制）  
void compress_line_one_point_per_row(const float pts_in[][2], int in_num,  
                                     float pts_out[][2], int *out_num,  
                                     bool is_left_line)  
{  
    if (!out_num) return;  
    *out_num = 0;  
    if (!pts_in || !pts_out || in_num <= 0) return;  
  
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
        int yi = (int)(pts_in[i][1] + 0.5f);  
        if (yi < 0 || yi >= UVC_HEIGHT) continue;  
  
        float xi = pts_in[i][0];  
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
        pts_out[k][0] = best_x[y];  
        pts_out[k][1] = best_y[y];  
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
// 点集局部角度变化率
void local_angle_points(float pts_in[][2], int num, float angle_out[], int dist)
 {
    for (int i = 0; i < num; i++) {
        if (i <= 0 || i >= num - 1) {
            angle_out[i] = 0;
            continue;
        }
        float dx1 = pts_in[i][0] - pts_in[limit_int(i - dist, 0, num - 1)][0];
        float dy1 = pts_in[i][1] - pts_in[limit_int(i - dist, 0, num - 1)][1];
        float dn1 = sqrtf(dx1 * dx1 + dy1 * dy1);
        float dx2 = pts_in[limit_int(i + dist, 0, num - 1)][0] - pts_in[i][0];
        float dy2 = pts_in[limit_int(i + dist, 0, num - 1)][1] - pts_in[i][1];
        float dn2 = sqrtf(dx2 * dx2 + dy2 * dy2);
        float c1 = dx1 / dn1;
        float s1 = dy1 / dn1;
        float c2 = dx2 / dn2;
        float s2 = dy2 / dn2;
        angle_out[i] = atan2f(c1 * s2 - c2 * s1, c2 * c1 + s2 * s1);
    }
}
// 角度变化率非极大抑制
void nms_angle(float angle_in[], int num, float angle_out[], int kernel) 
{
    int half = kernel / 2;
    for (int i = 0; i < num; i++) {
        angle_out[i] = angle_in[i];
        for (int j = -half; j <= half; j++) 
        {
            if (fabs(angle_in[limit_int(i + j, 0, num - 1)]) > fabs(angle_out[i])) 
            {
                angle_out[i] = 0;
                break;
            }
        }
    }
}
//返回角度最大值
void max_angle(float angle_in[], int num, float *angle_max, int *idx)  
{  
    *angle_max = 0.0f;  
    *idx = 0;  
    if (num < 3) return;  
  
    for (int i = 1; i < num - 1; i++)  
    {  
        if (fabs(angle_in[i]) > fabs(*angle_max))  
        {  
            *angle_max = fabs(angle_in[i]); // 保留符号  
            *idx = i;  
        }  
    }  
}  
// 对中线做加权，输出加权中心点（近->中->远可配权重）  
// 输入:  Mline 中线点集（建议传 rpts_c_resample）  
// 输出:  out_x/out_y 加权中心点坐标  
// 返回:  true=有效，false=无效（会回退上一次结果）  
bool calc_weighted_center_point(float Mline[][2], int num, float *out_x, float *out_y)  
{  
    if (!out_x || !out_y) return false;  
  
    const int target_samples = 15;  
    const int start_idx = 3;  
  
    // 近中远权重（可按车感微调）  
    const float weights[target_samples] = {  
        0.20f, 0.32f, 0.70f, 1.00f, 1.10f,  
        1.20f, 1.20f, 1.30f, 1.30f, 1.30f,  
        1.20f, 1.20f, 1.00f, 1.00f, 1.00f  
    };  
  
    // 记忆回退（丢线时保持）  
    static float last_x = UVC_WIDTH * 0.5f;  
    static float last_y = UVC_HEIGHT - 1.0f;  
  
    if (num <= start_idx) {  
        *out_x = last_x;  
        *out_y = last_y;  
        return false;  
    }  
  
    int available = num - start_idx;  
    float step = (available > target_samples) ? (float)available / (float)target_samples : 1.0f;  
  
    float sum_wx = 0.0f, sum_wy = 0.0f, sum_w = 0.0f;  
  
    for (int k = 0; k < target_samples; k++)  
    {  
        int i = start_idx + (int)(k * step);  
        if (i >= num) break;  
  
        float x = Mline[i][0];  
        float y = Mline[i][1];  
  
        // 有效性过滤  
        if (x < 0 || x >= UVC_WIDTH || y < 0 || y >= UVC_HEIGHT) continue;  
  
        float w = weights[k];  
        sum_wx += x * w;  
        sum_wy += y * w;  
        sum_w  += w;  
  
        if (step == 1.0f && i == num - 1) break;  
    }  
  
    if (sum_w < 1e-5f) {  
        *out_x = last_x;  
        *out_y = last_y;  
        return false;  
    }  
  
    *out_x = sum_wx / sum_w;  
    *out_y = sum_wy / sum_w;  
  
    last_x = *out_x;  
    last_y = *out_y;  
    return true;  
}  

/**
 * @brief 计算图像偏差提供差速
 */
void img_err_get()
{
    if(rpts_c_resample_num > 5)
    {
        aim_id = limit_int(aim_id,0,rpts_c_resample_num - 1);
        img_err = UVC_WIDTH/2 - rpts_c_resample[aim_id][0];
    }
    else
    {
        img_err = 0.0f;
    }
}

// void find_corners() {
//     // 识别L拐点
//     Lpt_l_found = Lpt_r_found = false;
//     is_straight_l = rpts_l_resample_num > 190;
//     is_straight_r = rpts_r_resample_num > 190;
//     for (int i = 0; i < rpts_l_resample_num; i++) 
//     {
//         // if (angles_nms_l[i] < 1e-4f) continue;
//         // int im1 = limit_int(i - 1, 0, rpts_l_resample_num - 1);
//         // int ip1 = limit_int(i + 1, 0, rpts_l_resample_num - 1);
//         // float conf = fabs(angles_l[i]) - (fabs(angles_l[im1]) + fabs(angles_l[ip1])) / 2;

//         //L角点阈值
//         if (Lpt_l_found == false && 40. / 180. * PI < conf && conf < 140. / 180. * PI && rpts_l_resample[i][1] >60) {
//             Lpt_l_id = i;
//             Lpt_l_found = true;
//         }
//         //长直道阈值
//         if (conf > 10. / 180. * PI && i < 30) is_straight_l = false;
//     }
//     for (int i = 0; i < rpts_r_resample_num; i++) {  
//         if (angles_nms_r[i] < 1e-4f) continue;  
//         int im1 = limit_int(i - 2, 0, rpts_r_resample_num - 1);  
//         int ip1 = limit_int(i + 2, 0, rpts_r_resample_num - 1);  
    
//         float conf = fabs(angles_r[i]) - (fabs(angles_r[im1]) + fabs(angles_r[ip1])) / 2.0f;  
    
//         // L角点阈值  
//         if (!Lpt_r_found &&  50.0f / 180.0f * PI < conf && conf < 130.0f / 180.0f * PI && i < 80) {  
//             Lpt_r_id = i;  
//             Lpt_r_found = true;  
//         }  
    
//         // 长直道阈值  
//         if (conf > 10.0f / 180.0f * PI && i < 30)   is_straight_r = false;  
//     }  
//     // L点二次检查，车库模式不检查, 依据L角点距离及角点后张开特性
//     }

void find_corners() {
    // 识别L拐点
    Lpt_l_found = Lpt_r_found = false;
    is_straight_l = rpts_l_resample_num > 190;
    is_straight_r = rpts_r_resample_num > 190;
    for (int i = 0; i < rpts_l_resample_num; i++) 
    {
        if (angles_nms_l[i] < 1e-4f) continue;
        int im1 = limit_int(i - 1, 0, rpts_l_resample_num - 1);
        int ip1 = limit_int(i + 1, 0, rpts_l_resample_num - 1);
        float conf = fabs(angles_l[i]) - (fabs(angles_l[im1]) + fabs(angles_l[ip1])) / 2;

        //L角点阈值
        if (Lpt_l_found == false && 40. / 180. * PI < conf && conf < 140. / 180. * PI && rpts_l_resample[i][1] >60) {
            Lpt_l_id = i;
            Lpt_l_found = true;
        }
        //长直道阈值
        if (conf > 10. / 180. * PI && i < 30) is_straight_l = false;
    }
    for (int i = 0; i < rpts_r_resample_num; i++) {  
        if (angles_nms_r[i] < 1e-4f) continue;  
        int im1 = limit_int(i - 2, 0, rpts_r_resample_num - 1);  
        int ip1 = limit_int(i + 2, 0, rpts_r_resample_num - 1);  
    
        float conf = fabs(angles_r[i]) - (fabs(angles_r[im1]) + fabs(angles_r[ip1])) / 2.0f;  
    
        // L角点阈值  
        if (!Lpt_r_found &&  50.0f / 180.0f * PI < conf && conf < 130.0f / 180.0f * PI && i < 80) {  
            Lpt_r_id = i;  
            Lpt_r_found = true;  
        }  
    
        // 长直道阈值  
        if (conf > 10.0f / 180.0f * PI && i < 30)   is_straight_r = false;  
    }  
    // L点二次检查，车库模式不检查, 依据L角点距离及角点后张开特性
    }


