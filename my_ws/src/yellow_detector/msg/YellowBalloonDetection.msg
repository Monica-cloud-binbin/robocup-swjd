# 输入图像时间戳和相机坐标系
std_msgs/Header header

# 当前帧是否检测到符合面积条件的黄色区域
bool detected

# 检测中心像素坐标（包围框中心）
int32 x_pixel
int32 y_pixel

# 包围框坐标；xmax/ymax 使用右侧、下侧开区间坐标
int32 xmin
int32 ymin
int32 xmax
int32 ymax

# 最大黄色轮廓的像素面积，以及占整帧面积的比例 [0, 1]
float32 area
float32 area_ratio
