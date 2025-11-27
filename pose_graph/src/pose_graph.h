#pragma once

#include <thread>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <string>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <queue>
#include <assert.h>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <stdio.h>
#include <rclcpp/rclcpp.hpp>
#include "keyframe.h"
#include "utility/tic_toc.h"
#include "utility/utility.h"
#include "utility/CameraPoseVisualization.h"
#include "utility/tic_toc.h"
#include "ThirdParty/DBoW/DBoW2.h"
#include "ThirdParty/DVision/DVision.h"
#include "ThirdParty/DBoW/TemplatedDatabase.h"
#include "ThirdParty/DBoW/TemplatedVocabulary.h"

// 宏定义：控制是否显示短边和长边，以及是否保存回环路径
#define SHOW_S_EDGE false  // 是否显示短边（相邻关键帧之间的边）
#define SHOW_L_EDGE true   // 是否显示长边（回环边）
#define SAVE_LOOP_PATH true // 是否保存回环路径

// 使用命名空间
using namespace DVision;   // 视觉特征相关的命名空间
using namespace DBoW2;    // 词袋模型相关的命名空间

// 位姿图优化类
class PoseGraph
{
public:
    // 构造函数和析构函数
    PoseGraph();
    ~PoseGraph();
    
    // 注册ROS2发布器
    // 参数：n - ROS2节点共享指针
    void registerPub(rclcpp::Node::SharedPtr n);
    
    // 添加关键帧到位姿图中
    // 参数：cur_kf - 当前关键帧指针
    //       flag_detect_loop - 是否进行回环检测的标志
    void addKeyFrame(KeyFrame* cur_kf, bool flag_detect_loop);
    
    // 加载关键帧（从文件或其他来源）
    // 参数：cur_kf - 当前关键帧指针  
    //       flag_detect_loop - 是否进行回环检测的标志
    void loadKeyFrame(KeyFrame* cur_kf, bool flag_detect_loop);
    
    // 加载词袋模型词汇表
    // 参数：voc_path - 词汇表文件路径
    void loadVocabulary(std::string voc_path);
    
    // 更新关键帧的回环信息
    // 参数：index - 关键帧索引
    //       _loop_info - 回环信息（8维向量）
    void updateKeyFrameLoop(int index, Eigen::Matrix<double, 8, 1 > &_loop_info);
    
    // 根据索引获取关键帧
    // 参数：index - 关键帧索引
    // 返回：关键帧指针
    KeyFrame* getKeyFrame(int index);
    
    // 路径可视化数组（最多10条路径）
    nav_msgs::msg::Path path[10];
    // 基础路径
    nav_msgs::msg::Path base_path;
    // 位姿图可视化对象
    CameraPoseVisualization* posegraph_visualization;
    
    // 保存位姿图到文件
    void savePoseGraph();
    // 从文件加载位姿图
    void loadPoseGraph();
    // 发布可视化消息
    void publish();
    
    // 漂移量（用于多序列对齐）
    Vector3d t_drift;  // 平移漂移
    double yaw_drift;  // 偏航角漂移
    Matrix3d r_drift;  // 旋转漂移
    
    // 世界坐标系与VIO坐标系之间的变换
    // world frame( base sequence or first sequence)<----> cur sequence frame  
    Vector3d w_t_vio;  // 世界系到VIO系的平移
    Matrix3d w_r_vio;  // 世界系到VIO系的旋转

private:
    // 回环检测函数
    // 参数：keyframe - 待检测的关键帧
    //       frame_index - 帧索引
    // 返回：检测到的回环关键帧索引，-1表示未检测到回环
    int detectLoop(KeyFrame* keyframe, int frame_index);
    
    // 将关键帧添加到词袋数据库中
    // 参数：keyframe - 要添加的关键帧
    void addKeyFrameIntoVoc(KeyFrame* keyframe);
    
    // 4自由度位姿图优化（优化x,y,z和偏航角yaw，固定pitch和roll）
    void optimize4DoF();
    
    // 更新路径信息
    void updatePath();
    
    // 关键帧列表（使用链表存储）
    list<KeyFrame*> keyframelist;
    
    // 互斥锁，用于线程同步
    std::mutex m_keyframelist;    // 保护关键帧列表
    std::mutex m_optimize_buf;    // 保护优化缓冲区
    std::mutex m_path;            // 保护路径数据
    std::mutex m_drift;           // 保护漂移量数据
    
    // 优化线程
    std::thread t_optimization;
    // 优化缓冲区队列（存储需要优化的关键帧索引）
    std::queue<int> optimize_buf;

    // 全局索引计数器
    int global_index;
    // 序列计数器（用于多序列SLAM）
    int sequence_cnt;
    // 序列回环标记数组
    vector<bool> sequence_loop;
    // 图像池，存储关键帧图像（用于可视化）
    map<int, cv::Mat> image_pool;
    // 最早的回环索引
    int earliest_loop_index;
    // 基础序列ID
    int base_sequence;

    // 词袋数据库
    BriefDatabase db;
    // 词袋词汇表指针
    BriefVocabulary* voc;

    // ROS2发布器
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_pg_path;        // 位姿图路径发布器
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_base_path;      // 基础路径发布器
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_pose_graph;  // 位姿图可视化发布器
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path[10];       // 多条路径发布器数组
};

// 角度归一化模板函数：将角度归一化到[-180, 180]度范围内
// 参数：angle_degrees - 输入角度（度）
// 返回：归一化后的角度
template <typename T>
T NormalizeAngle(const T& angle_degrees) {
  if (angle_degrees > T(180.0))
      return angle_degrees - T(360.0);  // 大于180度，减去360度
  else if (angle_degrees < T(-180.0))
      return angle_degrees + T(360.0);  // 小于-180度，加上360度
  else
      return angle_degrees;             // 在范围内，直接返回
};

// 角度局部参数化类（用于Ceres优化器）
class AngleLocalParameterization {
 public:
  // 操作符重载：实现角度加法并自动归一化
  // 参数：theta_radians - 当前角度（弧度）
  //       delta_theta_radians - 角度增量（弧度）
  //       theta_radians_plus_delta - 输出：相加并归一化后的角度
  template <typename T>
  bool operator()(const T* theta_radians, const T* delta_theta_radians,
                  T* theta_radians_plus_delta) const {
    *theta_radians_plus_delta =
        NormalizeAngle(*theta_radians + *delta_theta_radians);

    return true;
  }

  // 创建局部参数化对象
  static ceres::LocalParameterization* Create() {
    return (new ceres::AutoDiffLocalParameterization<AngleLocalParameterization,
                                                     1, 1>);
  }
};

// 将欧拉角转换为旋转矩阵的模板函数
// 参数：yaw, pitch, roll - 欧拉角（度）
//       R - 输出：3x3旋转矩阵（以数组形式存储）
template <typename T> 
void YawPitchRollToRotationMatrix(const T yaw, const T pitch, const T roll, T R[9])
{
    // 将角度从度转换为弧度
    T y = yaw / T(180.0) * T(M_PI);
    T p = pitch / T(180.0) * T(M_PI);
    T r = roll / T(180.0) * T(M_PI);

    // 计算旋转矩阵的各元素（ZYX旋转顺序）
    R[0] = cos(y) * cos(p);
    R[1] = -sin(y) * cos(r) + cos(y) * sin(p) * sin(r);
    R[2] = sin(y) * sin(r) + cos(y) * sin(p) * cos(r);
    R[3] = sin(y) * cos(p);
    R[4] = cos(y) * cos(r) + sin(y) * sin(p) * sin(r);
    R[5] = -cos(y) * sin(r) + sin(y) * sin(p) * cos(r);
    R[6] = -sin(p);
    R[7] = cos(p) * sin(r);
    R[8] = cos(p) * cos(r);
};

// 计算旋转矩阵的转置
// 参数：R - 输入旋转矩阵
//       inv_R - 输出：转置矩阵（即逆矩阵，因为旋转矩阵是正交矩阵）
template <typename T> 
void RotationMatrixTranspose(const T R[9], T inv_R[9])
{
    inv_R[0] = R[0];  // [0,0]
    inv_R[1] = R[3];  // [0,1] 
    inv_R[2] = R[6];  // [0,2]
    inv_R[3] = R[1];  // [1,0]
    inv_R[4] = R[4];  // [1,1]
    inv_R[5] = R[7];  // [1,2]
    inv_R[6] = R[2];  // [2,0]
    inv_R[7] = R[5];  // [2,1]
    inv_R[8] = R[8];  // [2,2]
};

// 用旋转矩阵旋转点
// 参数：R - 旋转矩阵
//       t - 输入点坐标
//       r_t - 输出：旋转后的点坐标
template <typename T> 
void RotationMatrixRotatePoint(const T R[9], const T t[3], T r_t[3])
{
    r_t[0] = R[0] * t[0] + R[1] * t[1] + R[2] * t[2];  // x分量
    r_t[1] = R[3] * t[0] + R[4] * t[1] + R[5] * t[2];  // y分量
    r_t[2] = R[6] * t[0] + R[7] * t[1] + R[8] * t[2];  // z分量
};

// 4自由度误差结构体（用于Ceres优化）
struct FourDOFError
{
    // 构造函数：初始化观测值
    // 参数：t_x, t_y, t_z - 相对平移观测值
    //       relative_yaw - 相对偏航角观测值
    //       pitch_i, roll_i - 关键帧i的俯仰角和横滚角（固定）
    FourDOFError(double t_x, double t_y, double t_z, double relative_yaw, double pitch_i, double roll_i)
                  :t_x(t_x), t_y(t_y), t_z(t_z), relative_yaw(relative_yaw), pitch_i(pitch_i), roll_i(roll_i){}

    // 误差计算函数（Ceres要求）
    // 参数：yaw_i - 关键帧i的偏航角
    //       ti - 关键帧i的位置
    //       yaw_j - 关键帧j的偏航角  
    //       tj - 关键帧j的位置
    //       residuals - 输出：残差向量
    template <typename T>
    bool operator()(const T* const yaw_i, const T* ti, const T* yaw_j, const T* tj, T* residuals) const
    {
        // 计算关键帧j相对于关键帧i的平移（世界坐标系下）
        T t_w_ij[3];
        t_w_ij[0] = tj[0] - ti[0];  // x方向平移
        t_w_ij[1] = tj[1] - ti[1];  // y方向平移
        t_w_ij[2] = tj[2] - ti[2];  // z方向平移

        // 将关键帧i的欧拉角转换为旋转矩阵（世界系到关键帧i坐标系）
        T w_R_i[9];
        YawPitchRollToRotationMatrix(yaw_i[0], T(pitch_i), T(roll_i), w_R_i);
        
        // 计算旋转矩阵的转置（关键帧i坐标系到世界系）
        T i_R_w[9];
        RotationMatrixTranspose(w_R_i, i_R_w);
        
        // 将平移向量转换到关键帧i坐标系下
        T t_i_ij[3];
        RotationMatrixRotatePoint(i_R_w, t_w_ij, t_i_ij);

        // 计算残差：
        // 前3个是平移残差（观测值-预测值）
        residuals[0] = (t_i_ij[0] - T(t_x));  // x方向平移残差
        residuals[1] = (t_i_ij[1] - T(t_y));  // y方向平移残差
        residuals[2] = (t_i_ij[2] - T(t_z));  // z方向平移残差
        
        // 第4个是偏航角残差（归一化角度差）
        residuals[3] = NormalizeAngle(yaw_j[0] - yaw_i[0] - T(relative_yaw));

        return true;
    }

    // 创建Ceres代价函数
    // 参数：t_x, t_y, t_z - 相对平移观测值
    //       relative_yaw - 相对偏航角观测值
    //       pitch_i, roll_i - 关键帧i的固定角度
    // 返回：Ceres代价函数指针
    static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z,
                                       const double relative_yaw, const double pitch_i, const double roll_i) 
    {
      return (new ceres::AutoDiffCostFunction<
              FourDOFError, 4, 1, 3, 1, 3>(
                new FourDOFError(t_x, t_y, t_z, relative_yaw, pitch_i, roll_i)));
    }

    // 成员变量：存储观测值
    double t_x, t_y, t_z;          // 相对平移观测值
    double relative_yaw, pitch_i, roll_i;  // 相对偏航角和固定角度

};

// 带权重的4自由度误差结构体
struct FourDOFWeightError
{
    // 构造函数：初始化观测值和权重
    FourDOFWeightError(double t_x, double t_y, double t_z, double relative_yaw, double pitch_i, double roll_i)
                  :t_x(t_x), t_y(t_y), t_z(t_z), relative_yaw(relative_yaw), pitch_i(pitch_i), roll_i(roll_i){
                    weight = 1;  // 初始化权重为1
                  }

    // 误差计算函数（带权重）
    template <typename T>
    bool operator()(const T* const yaw_i, const T* ti, const T* yaw_j, const T* tj, T* residuals) const
    {
        // 计算关键帧j相对于关键帧i的平移（世界坐标系下）
        T t_w_ij[3];
        t_w_ij[0] = tj[0] - ti[0];
        t_w_ij[1] = tj[1] - ti[1];
        t_w_ij[2] = tj[2] - ti[2];

        // 将关键帧i的欧拉角转换为旋转矩阵
        T w_R_i[9];
        YawPitchRollToRotationMatrix(yaw_i[0], T(pitch_i), T(roll_i), w_R_i);
        
        // 计算旋转矩阵的转置
        T i_R_w[9];
        RotationMatrixTranspose(w_R_i, i_R_w);
        
        // 将平移向量转换到关键帧i坐标系下
        T t_i_ij[3];
        RotationMatrixRotatePoint(i_R_w, t_w_ij, t_i_ij);

        // 计算带权重的残差：
        residuals[0] = (t_i_ij[0] - T(t_x)) * T(weight);  // x平移残差带权重
        residuals[1] = (t_i_ij[1] - T(t_y)) * T(weight);  // y平移残差带权重
        residuals[2] = (t_i_ij[2] - T(t_z)) * T(weight);  // z平移残差带权重
        
        // 角度残差带权重（角度残差的权重较小，除以10.0）
        residuals[3] = NormalizeAngle((yaw_j[0] - yaw_i[0] - T(relative_yaw))) * T(weight) / T(10.0);

        return true;
    }

    // 创建带权重的Ceres代价函数
    static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z,
                                       const double relative_yaw, const double pitch_i, const double roll_i) 
    {
      return (new ceres::AutoDiffCostFunction<
              FourDOFWeightError, 4, 1, 3, 1, 3>(
                new FourDOFWeightError(t_x, t_y, t_z, relative_yaw, pitch_i, roll_i)));
    }

    // 成员变量
    double t_x, t_y, t_z;          // 相对平移观测值
    double relative_yaw, pitch_i, roll_i;  // 角度观测值
    double weight;                  // 权重值

};