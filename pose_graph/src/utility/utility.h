#pragma once
#include <sys/types.h>
#include <sys/stat.h>
#include <libgen.h>
#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <cassert>
#include <cstring>
#include <eigen3/Eigen/Dense>
class Utility
{
public:
    // 将旋转向量转换为四元数的增量（用于李代数到李群的映射）
    template <typename Derived>
    static Eigen::Quaternion<typename Derived::Scalar> deltaQ(const Eigen::MatrixBase<Derived> &theta)
    {
        typedef typename Derived::Scalar Scalar_t;

        Eigen::Quaternion<Scalar_t> dq;
        // 将旋转向量除以2（因为四元数表示旋转时使用半角）
        Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
        half_theta /= static_cast<Scalar_t>(2.0);
        // 构造四元数：实部为1，虚部为half_theta
        dq.w() = static_cast<Scalar_t>(1.0);
        dq.x() = half_theta.x();
        dq.y() = half_theta.y();
        dq.z() = half_theta.z();
        return dq;
    }

    // 计算向量的反对称矩阵（叉乘矩阵）
    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> skewSymmetric(const Eigen::MatrixBase<Derived> &q)
    {
        Eigen::Matrix<typename Derived::Scalar, 3, 3> ans;
        // 反对称矩阵定义：[v]× = [0, -vz, vy; vz, 0, -vx; -vy, vx, 0]
        ans << typename Derived::Scalar(0), -q(2), q(1),
            q(2), typename Derived::Scalar(0), -q(0),
            -q(1), q(0), typename Derived::Scalar(0);
        return ans;
    }

    // 确保四元数的实部为非负（保持四元数的唯一表示）
    template <typename Derived>
    static Eigen::Quaternion<typename Derived::Scalar> positify(const Eigen::QuaternionBase<Derived> &q)
    {
        // 注释掉的代码原本的功能：如果w为负，则取相反的四元数（表示相同的旋转）
        //printf("a: %f %f %f %f", q.w(), q.x(), q.y(), q.z());
        //Eigen::Quaternion<typename Derived::Scalar> p(-q.w(), -q.x(), -q.y(), -q.z());
        //printf("b: %f %f %f %f", p.w(), p.x(), p.y(), p.z());
        //return q.template w() >= (typename Derived::Scalar)(0.0) ? q : Eigen::Quaternion<typename Derived::Scalar>(-q.w(), -q.x(), -q.y(), -q.z());
        return q;  // 当前实现直接返回，没有实际的正规化操作
    }

    // 计算四元数的左乘矩阵（用于四元数乘法：q ⊗ p = Qleft(q) * p）
    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qleft(const Eigen::QuaternionBase<Derived> &q)
    {
        Eigen::Quaternion<typename Derived::Scalar> qq = positify(q);
        Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
        // 构造左乘矩阵：[[w, -vᵀ], [v, wI + [v]×]]
        ans(0, 0) = qq.w(), ans.template block<1, 3>(0, 1) = -qq.vec().transpose();
        ans.template block<3, 1>(1, 0) = qq.vec(), ans.template block<3, 3>(1, 1) = qq.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() + skewSymmetric(qq.vec());
        return ans;
    }

    // 计算四元数的右乘矩阵（用于四元数乘法：p ⊗ q = Qright(p) * q）
    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qright(const Eigen::QuaternionBase<Derived> &p)
    {
        Eigen::Quaternion<typename Derived::Scalar> pp = positify(p);
        Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
        // 构造右乘矩阵：[[w, -vᵀ], [v, wI - [v]×]]
        ans(0, 0) = pp.w(), ans.template block<1, 3>(0, 1) = -pp.vec().transpose();
        ans.template block<3, 1>(1, 0) = pp.vec(), ans.template block<3, 3>(1, 1) = pp.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() - skewSymmetric(pp.vec());
        return ans;
    }

    // 将旋转矩阵转换为欧拉角（yaw-pitch-roll，单位：度）
    static Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R)
    {
        // 提取旋转矩阵的列向量：n(x轴), o(y轴), a(z轴)
        Eigen::Vector3d n = R.col(0);
        Eigen::Vector3d o = R.col(1);
        Eigen::Vector3d a = R.col(2);

        Eigen::Vector3d ypr(3);
        // 计算yaw角（绕z轴旋转）：atan2(ny, nx)
        double y = atan2(n(1), n(0));
        // 计算pitch角（绕y轴旋转）：atan2(-nz, nx*cos(y) + ny*sin(y))
        double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
        // 计算roll角（绕x轴旋转）
        double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
        ypr(0) = y;
        ypr(1) = p;
        ypr(2) = r;

        // 将弧度转换为角度
        return ypr / M_PI * 180.0;
    }

    // 将欧拉角（yaw-pitch-roll，单位：度）转换为旋转矩阵
    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> ypr2R(const Eigen::MatrixBase<Derived> &ypr)
    {
        typedef typename Derived::Scalar Scalar_t;

        // 将角度转换为弧度
        Scalar_t y = ypr(0) / 180.0 * M_PI;
        Scalar_t p = ypr(1) / 180.0 * M_PI;
        Scalar_t r = ypr(2) / 180.0 * M_PI;

        // 绕z轴的旋转矩阵（yaw）
        Eigen::Matrix<Scalar_t, 3, 3> Rz;
        Rz << cos(y), -sin(y), 0,
            sin(y), cos(y), 0,
            0, 0, 1;

        // 绕y轴的旋转矩阵（pitch）
        Eigen::Matrix<Scalar_t, 3, 3> Ry;
        Ry << cos(p), 0., sin(p),
            0., 1., 0.,
            -sin(p), 0., cos(p);

        // 绕x轴的旋转矩阵（roll）
        Eigen::Matrix<Scalar_t, 3, 3> Rx;
        Rx << 1., 0., 0.,
            0., cos(r), -sin(r),
            0., sin(r), cos(r);

        // 组合旋转：R = Rz * Ry * Rx（zyx旋转顺序）
        return Rz * Ry * Rx;
    }

    // 将重力向量转换为旋转矩阵（在类外定义）
    static Eigen::Matrix3d g2R(const Eigen::Vector3d &g);

    // 模板元编程相关的辅助结构，用于循环展开
    template <size_t N>
    struct uint_
    {
    };

    // 递归展开循环（模板元编程技术）
    template <size_t N, typename Lambda, typename IterT>
    void unroller(const Lambda &f, const IterT &iter, uint_<N>)
    {
        unroller(f, iter, uint_<N - 1>());
        f(iter + N);
    }

    // 递归终止条件
    template <typename Lambda, typename IterT>
    void unroller(const Lambda &f, const IterT &iter, uint_<0>)
    {
        f(iter);
    }

    // 将角度归一化到(-180, 180]度范围内
    template <typename T>
    static T normalizeAngle(const T& angle_degrees) {
      T two_pi(2.0 * 180);  // 360度
      if (angle_degrees > 0)
      return angle_degrees -
          two_pi * std::floor((angle_degrees + T(180)) / two_pi);
      else
        return angle_degrees +
            two_pi * std::floor((-angle_degrees + T(180)) / two_pi);
    };
};
class FileSystemHelper
{
public:

    /******************************************************************************
     * 递归创建目录（如果路径不存在）
     * 成功返回0，失败返回1
     *****************************************************************************/
    static int createDirectoryIfNotExists(const char *path)
    {
        // 定义文件状态结构体，用于获取路径信息
        struct stat info;
        
        // 获取路径状态信息，返回值：0=成功，-1=失败
        int statRC = stat(path, &info);
        
        // 如果stat调用失败（路径不存在或其他错误）
        if( statRC != 0 )
        {
            // 错误号ENOENT表示文件或目录不存在
            if (errno == ENOENT)  
            {
                // 输出提示信息，目录不存在，尝试创建
                printf("%s not exists, trying to create it \n", path);
                
                // 递归创建父目录
                // strdupa(path): 在栈上复制路径字符串（自动内存管理）
                // dirname(): 提取路径的目录部分（去掉最后一级）
                // 递归调用自身创建父目录，如果父目录创建成功（返回0）
                if (! createDirectoryIfNotExists(dirname(strdupa(path))))
                {
                    // 创建当前目录
                    // mkdir参数：
                    // - path: 要创建的目录路径
                    // - 权限模式: S_IRWXU(用户读写执行) | S_IRWXG(组读写执行) | 
                    //   S_IROTH(其他读) | S_IXOTH(其他执行)
                    // 对应权限：drwxrwxr-x (用户和组有全部权限，其他用户有读和执行权限)
                    if (mkdir(path, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) != 0)
                    {
                        // 创建目录失败，输出错误信息
                        fprintf(stderr, "Failed to create folder %s \n", path);
                        return 1;  // 返回失败
                    }
                    else
                        return 0;  // 创建成功，返回0
                }
                else 
                    return 1;  // 父目录创建失败，返回1
            } // 目录不存在的情况处理结束
            
            // 错误号ENOTDIR表示路径中的某个组件不是目录
            if (errno == ENOTDIR) 
            { 
                // 输出错误信息：路径前缀中的某个组件不是目录
                fprintf(stderr, "%s is not a directory path \n", path);
                return 1;  // 返回失败
            } // 路径前缀不是目录的情况处理结束
            
            // 其他stat错误，返回失败
            return 1;
        }
        
        // stat调用成功，检查获取到的信息是否是目录
        // info.st_mode: 文件模式位
        // S_IFDIR: 目录文件的模式位
        // (info.st_mode & S_IFDIR): 如果结果是S_IFDIR，说明path是目录
        // 如果是目录返回0（成功），否则返回1（失败，因为路径存在但不是目录）
        return ( info.st_mode & S_IFDIR ) ? 0 : 1;
    }
};