#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include "../parameters.h"

class CameraPoseVisualization {
public:
    // 标记的命名空间，用于在RViz中区分不同的可视化对象
    std::string m_marker_ns;

    // 构造函数：设置相机位姿可视化器的基本颜色和透明度
    // 参数：r, g, b, a -  RGBA颜色值（红、绿、蓝、透明度）
    CameraPoseVisualization(float r, float g, float b, float a);
    
    // 设置图像边界的颜色
    // 参数：r, g, b, a - RGBA颜色值，a默认为1.0（不透明）
    void setImageBoundaryColor(float r, float g, float b, float a=1.0);
    
    // 设置光学中心连接线的颜色
    // 参数：r, g, b, a - RGBA颜色值，a默认为1.0
    void setOpticalCenterConnectorColor(float r, float g, float b, float a=1.0);
    
    // 设置可视化标记的缩放比例
    // 参数：s - 缩放比例因子
    void setScale(double s);
    
    // 设置线条的宽度
    // 参数：width - 线条宽度
    void setLineWidth(double width);

    // 添加一个相机位姿到可视化中
    // 参数：p - 相机位置（平移向量）
    //       q - 相机朝向（四元数表示旋转）
    void add_pose(const Eigen::Vector3d& p, const Eigen::Quaterniond& q);
    
    // 重置可视化器，清除所有已添加的位姿和标记
    void reset();

    // 通过指定的发布器发布MarkerArray消息
    // 参数：pub   - ROS2发布器，用于发布MarkerArray消息
    //       header - 消息头，包含时间戳和坐标系信息
    void publish_by(rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub, const std_msgs::msg::Header& header);
    
    // 添加一条普通边（连接两个点）
    // 参数：p0, p1 - 边的起点和终点坐标
    void add_edge(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1);
    
    // 添加一条回环边（通常用于SLAM中的回环检测可视化）
    // 参数：p0, p1 - 回环边的起点和终点坐标
    void add_loopedge(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1);
    
    // 注释掉的功能：添加图像纹理到可视化中
    //void add_image(const Eigen::Vector3d& T, const Eigen::Matrix3d& R, const cv::Mat &src);
    
    // 通过指定的发布器发布图像标记消息
    // 参数：pub    - ROS2发布器，用于发布Marker消息
    //       header - 消息头，包含时间戳和坐标系信息
    void publish_image_by( rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub, const std_msgs::msg::Header &header);

private:
    // 存储所有可视化标记的容器
    std::vector<visualization_msgs::msg::Marker> m_markers;
    
    // 图像边界线的颜色
    std_msgs::msg::ColorRGBA m_image_boundary_color;
    
    // 光学中心连接线的颜色
    std_msgs::msg::ColorRGBA m_optical_center_connector_color;
    
    // 整体缩放比例
    double m_scale;
    
    // 线条宽度
    double m_line_width;
    
    // 图像纹理标记对象
    visualization_msgs::msg::Marker image;
    
    // 回环边的数量限制
    int LOOP_EDGE_NUM;
    
    // 临时回环边计数
    int tmp_loop_edge_num;

    // 静态常量：定义相机图像平面的四个角点（在相机坐标系下）
    static const Eigen::Vector3d imlt;  // 图像左上角 (Image Left-Top)
    static const Eigen::Vector3d imlb;  // 图像左下角 (Image Left-Bottom)
    static const Eigen::Vector3d imrt;  // 图像右上角 (Image Right-Top)
    static const Eigen::Vector3d imrb;  // 图像右下角 (Image Right-Bottom)
    
    // 静态常量：相机光学中心位置（在相机坐标系下，通常为原点）
    static const Eigen::Vector3d oc  ;  // 光学中心 (Optical Center)
    
    // 静态常量：定义相机视锥体的其他点（用于绘制相机金字塔形状）
    static const Eigen::Vector3d lt0 ;  // 视锥体点0
    static const Eigen::Vector3d lt1 ;  // 视锥体点1  
    static const Eigen::Vector3d lt2 ;  // 视锥体点2
};
