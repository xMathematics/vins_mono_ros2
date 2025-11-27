#pragma once
#include <rclcpp/rclcpp.hpp>
#include <opencv2/highgui/highgui.hpp>

// 声明外部变量：图像的行数（高度）
extern int ROW;
// 声明外部变量：图像的列数（宽度）
extern int COL;
// 声明外部变量：相机的焦距（像素单位）
extern int FOCAL_LENGTH;

// 定义常量：相机的数量，这里设置为1表示单目相机
const int NUM_OF_CAM = 1;

// 声明外部变量：ROS图像话题的名称
extern std::string IMAGE_TOPIC;
// 声明外部变量：ROS IMU话题的名称
extern std::string IMU_TOPIC;
// 声明外部变量：鱼眼相机掩膜文件的路径
extern std::string FISHEYE_MASK;
// 声明外部变量：相机参数文件路径的向量
extern std::vector<std::string> CAM_NAMES;

// 声明外部变量：最大特征点数量
extern int MAX_CNT;
// 声明外部变量：特征点之间的最小像素距离（用于保证特征点分布均匀）
extern int MIN_DIST;
// 声明外部变量：滑动窗口的大小（用于优化处理的帧数）
extern int WINDOW_SIZE;
// 声明外部变量：图像发布频率（Hz）
extern int FREQ;
// 声明外部变量：基础矩阵/F矩阵的RANSAC阈值（像素单位）
extern double F_THRESHOLD;
// 声明外部变量：是否显示特征点跟踪可视化
extern int SHOW_TRACK;
// 声明外部变量：是否进行双目跟踪（0-单目，1-双目）
extern int STEREO_TRACK;
// 声明外部变量：是否进行图像直方图均衡化（用于应对光照变化）
extern int EQUALIZE;
// 声明外部变量：是否为鱼眼相机模型（0-普通相机，1-鱼眼相机）
extern int FISHEYE;
// 声明外部变量：是否发布当前帧（用于控制发布频率）
extern bool PUB_THIS_FRAME;

/**
 * 参数读取函数
 * 从ROS2节点中读取并设置所有上述参数
 *
 * @param n ROS2节点的共享指针，用于访问节点参数
 *
 * 功能说明：
 * 1. 从ROS参数服务器或launch文件读取配置参数
 * 2. 设置图像和IMU的话题名称
 * 3. 配置特征跟踪的相关参数
 * 4. 设置相机模型和预处理选项
 * 5. 配置滑动窗口和发布设置
 */
void readParameters(rclcpp::Node::SharedPtr &n);
