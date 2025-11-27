#include "utility.h"


/*
具体步骤：

1将输入向量g归一化。
2构建一个目标向量ng2，即z轴正方向(0,0,1)。
3计算从归一化的输入向量ng1到目标向量ng2的旋转，得到一个旋转矩阵R0。这个旋转矩阵可以将ng1旋转到ng2。
4但是，这个旋转矩阵可能含有yaw角（绕z轴的旋转），我们希望通过调整使yaw角为0。
5因此，我们计算R0的yaw角，然后构造一个反向的yaw旋转，将R0的yaw角抵消。
6最后返回调整后的旋转矩阵。
*/


// 将重力向量转换为旋转矩阵，使z轴与重力方向对齐
Eigen::Matrix3d Utility::g2R(const Eigen::Vector3d &g)
{
    Eigen::Matrix3d R0;  // 声明一个3x3的旋转矩阵
    
    // 将输入的重力向量归一化（单位化）
    Eigen::Vector3d ng1 = g.normalized();
    
    // 定义目标重力方向 - 指向z轴正方向（通常表示向上的方向）
    Eigen::Vector3d ng2{0, 0, 1.0};
    
    // 计算从当前重力方向(ng1)到目标重力方向(ng2)的旋转
    // FromTwoVectors会找到一个旋转，使得ng1旋转后与ng2对齐
    R0 = Eigen::Quaterniond::FromTwoVectors(ng1, ng2).toRotationMatrix();
    
    // 将旋转矩阵转换为欧拉角（yaw-pitch-roll格式），提取yaw角
    // R2ypr可能返回Vector3d(yaw, pitch, roll)
    double yaw = Utility::R2ypr(R0).x();
    
    // 消除yaw角的影响，使旋转矩阵只包含pitch和roll分量
    // 通过乘以一个反向的yaw旋转来实现
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    
    // 注释掉的代码：如果使用这个，会强制旋转-90度（可能是特定应用场景）
    // R0 = Utility::ypr2R(Eigen::Vector3d{-90, 0, 0}) * R0;
    
    return R0;  // 返回最终的旋转矩阵
}