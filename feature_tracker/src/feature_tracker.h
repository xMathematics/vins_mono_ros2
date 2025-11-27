#pragma once

#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>
#include <rcutils/logging_macros.h>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

#include "parameters.h"
#include "tic_toc.h"
// 使用标准命名空间
using namespace std;
// 使用camodocal相机标定库命名空间
using namespace camodocal;
// 使用Eigen数学库命名空间（用于矩阵运算）
using namespace Eigen;

// 函数声明：检查特征点是否在图像边界内
// @param pt 要检查的特征点坐标
// @return 如果点在图像边界内返回true，否则返回false
bool inBorder(const cv::Point2f &pt);

// 函数重载：根据状态向量减少点向量（光流跟踪后使用）
// @param v 要缩减的点向量（输入输出参数）
// @param status 状态向量，1表示跟踪成功，0表示跟踪失败
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
// 函数重载：根据状态向量减少整型向量（如ID、计数等）
// @param v 要缩减的整型向量（输入输出参数）
// @param status 状态向量，1表示保留，0表示删除
void reduceVector(vector<int> &v, vector<uchar> status);

// 特征跟踪器类 - VINS系统的视觉前端核心组件
class FeatureTracker
{
  public:
    // 构造函数：初始化特征跟踪器
    FeatureTracker();

    /**
     * 主要接口：读取新图像并进行特征跟踪
     * @param _img 输入图像
     * @param _cur_time 当前图像时间戳
     */
    void readImage(const cv::Mat &_img, double _cur_time);

    /**
     * 设置掩膜：避免在已有特征点附近重复提取新特征点
     * 通过在已有特征点周围画圆来创建掩膜
     */
    void setMask();

    /**
     * 添加新的特征点：当特征点数量不足时补充新特征点
     * 使用FAST或Shi-Tomasi角点检测
     */
    void addPoints();

    /**
     * 更新特征点ID
     * @param i 特征点索引
     * @return 如果分配了新ID返回true，否则返回false
     */
    bool updateID(unsigned int i);

    /**
     * 读取相机内参和畸变系数
     * @param calib_file 相机标定文件路径
     */
    void readIntrinsicParameter(const string &calib_file);

    /**
     * 显示去畸变效果：用于调试和验证相机参数
     * @param name 显示窗口名称
     */
    void showUndistortion(const string &name);

    /**
     * 使用基础矩阵F剔除异常匹配点（RANSAC方法）
     * 通过计算两帧之间的基础矩阵来剔除误匹配
     */
    void rejectWithF();

    /**
     * 对特征点进行去畸变处理，并转换到归一化平面
     * 将像素坐标转换为相机坐标系下的归一化坐标
     */
    void undistortedPoints();

    // 成员变量：

    // 掩膜图像：用于控制特征点提取区域，避免特征点聚集
    cv::Mat mask;
    // 鱼眼相机专用掩膜：去除鱼眼图像边缘失真严重区域
    cv::Mat fisheye_mask;
    
    // 图像序列：维护连续三帧图像用于光流跟踪
    cv::Mat prev_img;  // 上一帧图像（已处理）
    cv::Mat cur_img;   // 当前帧图像（正在处理）
    cv::Mat forw_img;  // 向前传播帧（新输入图像）
    
    // 归一化平面坐标点：去畸变后的特征点在归一化平面的坐标
    vector<cv::Point2f> n_pts;
    
    // 特征点像素坐标序列：
    vector<cv::Point2f> prev_pts;  // 上一帧特征点坐标
    vector<cv::Point2f> cur_pts;   // 当前帧特征点坐标  
    vector<cv::Point2f> forw_pts;  // 向前传播帧特征点坐标
    
    // 去畸变后的特征点坐标：
    vector<cv::Point2f> prev_un_pts;  // 上一帧去畸变坐标
    vector<cv::Point2f> cur_un_pts;   // 当前帧去畸变坐标
    
    // 特征点速度：像素位移/时间间隔，用于后续VIO初始化
    vector<cv::Point2f> pts_velocity;
    
    // 特征点ID：每个特征点的唯一标识，用于跨帧跟踪
    vector<int> ids;
    // 特征点跟踪计数：记录每个特征点被成功跟踪的帧数
    vector<int> track_cnt;
    
    // 特征点映射表：以ID为键，便于快速查找和匹配
    map<int, cv::Point2f> cur_un_pts_map;   // 当前帧去畸变特征点映射
    map<int, cv::Point2f> prev_un_pts_map;  // 上一帧去畸变特征点映射
    
    // 相机模型指针：包含内参、畸变系数等相机参数
    camodocal::CameraPtr m_camera;
    
    // 时间戳：用于计算特征点速度和IMU预积分
    double cur_time;    // 当前帧时间戳
    double prev_time;   // 上一帧时间戳

    // 静态变量：用于生成唯一特征点ID（所有实例共享）
    static int n_id;
};