#include "parameters.h"
// 定义全局变量（这些变量在其他文件中通过extern声明使用）

// 图像话题名称
std::string IMAGE_TOPIC;
// IMU话题名称  
std::string IMU_TOPIC;
// 相机参数文件路径数组
std::vector<std::string> CAM_NAMES;
// 鱼眼相机掩膜文件路径
std::string FISHEYE_MASK;
// 最大特征点数量
int MAX_CNT;
// 特征点间最小像素距离
int MIN_DIST;
// 滑动窗口大小（优化窗口中的关键帧数量）
int WINDOW_SIZE;
// 图像处理频率（Hz）
int FREQ;
// 基础矩阵RANSAC阈值（像素单位）
double F_THRESHOLD;
// 是否显示特征跟踪可视化（0-不显示，1-显示）
int SHOW_TRACK;
// 是否进行双目跟踪（0-单目，1-双目）
int STEREO_TRACK;
// 是否进行图像直方图均衡化（0-不均衡，1-均衡）
int EQUALIZE;
// 图像高度（行数）
int ROW;
// 图像宽度（列数）
int COL;
// 相机焦距（像素单位）
int FOCAL_LENGTH;
// 是否为鱼眼相机（0-普通相机，1-鱼眼相机）
int FISHEYE;
// 是否发布当前帧的标志
bool PUB_THIS_FRAME;

/**
 * 模板函数：从ROS2节点读取参数
 * @tparam T 参数类型（自动推导）
 * @param n ROS2节点共享指针
 * @param name 参数名称
 * @return 读取到的参数值
 * 
 * 功能：声明并读取ROS2参数，如果读取失败则关闭节点
 */
template <typename T>
T readParam(rclcpp::Node::SharedPtr n, std::string name)
{
    T ans;  // 存储读取结果的变量
    std::string default_value = "";  // 默认参数值（空字符串）
    
    // 在ROS2节点中声明参数，指定参数名和默认值
    n->declare_parameter<std::string>(name, default_value);
    
    // 尝试从参数服务器获取参数值
    if (n->get_parameter(name, ans))
    {
        // 读取成功：打印加载信息
        RCLCPP_INFO_STREAM(n->get_logger(), "Loaded " << name << ": " << ans);
    }
    else
    {
        // 读取失败：打印错误信息并关闭节点
        RCLCPP_ERROR_STREAM(n->get_logger(), "Failed to load " << name);
        rclcpp::shutdown();
    }
    return ans;  // 返回读取到的参数值
}

/**
 * 主要参数读取函数
 * 从配置文件中读取所有VINS系统参数并初始化全局变量
 * @param n ROS2节点共享指针
 */
void readParameters(rclcpp::Node::SharedPtr &n)
{
    // 步骤1：读取配置文件路径
    std::string config_file;
    config_file = readParam<std::string>(n, "config_file");
    RCUTILS_LOG_INFO("config_file: %s", config_file.c_str());
    
    // 步骤2：使用OpenCV的FileStorage打开YAML配置文件
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        // 配置文件打开失败，输出错误信息
        RCUTILS_LOG_ERROR("ERROR: Wrong path to settings");
    }
    
    // 步骤3：读取VINS系统文件夹路径
    std::string VINS_FOLDER_PATH = readParam<std::string>(n, "vins_folder");
    
    // 步骤4：从配置文件中读取各项参数并赋值给全局变量
    
    // 读取图像话题名称（ROS话题名）
    fsSettings["image_topic"] >> IMAGE_TOPIC;
    // 读取IMU话题名称（ROS话题名）
    fsSettings["imu_topic"] >> IMU_TOPIC;
    // 读取每帧最大特征点数量
    MAX_CNT = fsSettings["max_cnt"];
    // 读取特征点间最小像素距离
    MIN_DIST = fsSettings["min_dist"];
    // 读取图像高度
    ROW = fsSettings["image_height"];
    // 读取图像宽度
    COL = fsSettings["image_width"];
    // 读取图像处理频率
    FREQ = fsSettings["freq"];
    // 读取基础矩阵RANSAC阈值
    F_THRESHOLD = fsSettings["F_threshold"];
    // 读取是否显示跟踪可视化
    SHOW_TRACK = fsSettings["show_track"];
    // 读取是否进行图像均衡化
    EQUALIZE = fsSettings["equalize"];
    // 读取是否为鱼眼相机
    FISHEYE = fsSettings["fisheye"];
    
    // 如果是鱼眼相机，设置鱼眼掩膜文件路径
    if (FISHEYE == 1)
        FISHEYE_MASK = VINS_FOLDER_PATH + "config/fisheye_mask.jpg";
    
    // 将配置文件路径添加到相机参数文件列表中
    CAM_NAMES.push_back(config_file);
    
    // 步骤5：设置一些固定参数（硬编码）
    WINDOW_SIZE = 20;        // 滑动窗口大小固定为20帧
    STEREO_TRACK = false;    // 默认为单目跟踪
    FOCAL_LENGTH = 460;      // 默认焦距值
    PUB_THIS_FRAME = false;  // 默认不发布当前帧
    
    // 步骤6：参数后处理与验证
    // 如果频率设置为0，使用默认频率100Hz
    if (FREQ == 0)
        FREQ = 100;
    
    // 释放配置文件资源
    fsSettings.release();
}