#ifndef CAMERACALIBRATION_H
#define CAMERACALIBRATION_H

#include <opencv2/core/core.hpp>

#include "camodocal/camera_models/Camera.h"
// camodocal命名空间，包含相机标定相关功能
namespace camodocal
{

// 相机标定类
class CameraCalibration
{
public:
    // Eigen库内存对齐宏，确保使用Eigen数据类型时内存正确对齐
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // 默认构造函数
    CameraCalibration();

    /**
     * 带参数的构造函数
     * @param modelType 相机模型类型（如针孔、鱼眼等）
     * @param cameraName 相机名称标识
     * @param imageSize 图像尺寸（宽度×高度）
     * @param boardSize 标定板角点尺寸（列数×行数）
     * @param squareSize 标定板方格的实际物理尺寸（单位：米）
     */
    CameraCalibration(Camera::ModelType modelType,
                      const std::string& cameraName,
                      const cv::Size& imageSize,
                      const cv::Size& boardSize,
                      float squareSize);

    // 清空所有标定数据
    void clear(void);

    /**
     * 添加棋盘格角点数据
     * @param corners 检测到的棋盘格角点像素坐标向量
     */
    void addChessboardData(const std::vector<cv::Point2f>& corners);

    // 执行相机标定，返回标定是否成功
    bool calibrate(void);

    // 获取样本（图像）数量
    int sampleCount(void) const;

    // 获取图像点坐标（可修改版本）
    std::vector<std::vector<cv::Point2f> >& imagePoints(void);
    // 获取图像点坐标（只读版本）
    const std::vector<std::vector<cv::Point2f> >& imagePoints(void) const;

    // 获取场景点坐标（可修改版本）
    std::vector<std::vector<cv::Point3f> >& scenePoints(void);
    // 获取场景点坐标（只读版本）
    const std::vector<std::vector<cv::Point3f> >& scenePoints(void) const;

    // 获取相机对象指针（可修改版本）
    CameraPtr& camera(void);
    // 获取相机对象指针（只读版本）
    const CameraConstPtr camera(void) const;

    // 获取测量协方差矩阵（可修改版本）
    Eigen::Matrix2d& measurementCovariance(void);
    // 获取测量协方差矩阵（只读版本）
    const Eigen::Matrix2d& measurementCovariance(void) const;

    // 获取相机位姿（可修改版本）
    cv::Mat& cameraPoses(void);
    // 获取相机位姿（只读版本）
    const cv::Mat& cameraPoses(void) const;

    /**
     * 绘制标定结果
     * @param images 输入图像序列，将在图像上绘制标定结果
     */
    void drawResults(std::vector<cv::Mat>& images) const;

    /**
     * 将标定参数写入文件
     * @param filename 输出文件名
     */
    void writeParams(const std::string& filename) const;

    /**
     * 将棋盘格数据写入文件
     * @param filename 输出文件名
     * @return 写入是否成功
     */
    bool writeChessboardData(const std::string& filename) const;

    /**
     * 从文件读取棋盘格数据
     * @param filename 输入文件名
     * @return 读取是否成功
     */
    bool readChessboardData(const std::string& filename);

    /**
     * 设置是否输出详细信息
     * @param verbose true表示输出详细信息，false表示不输出
     */
    void setVerbose(bool verbose);

private:
    /**
     * 标定辅助函数
     * @param camera 相机对象指针
     * @param rvecs 输出的旋转向量序列
     * @param tvecs 输出的平移向量序列
     * @return 标定是否成功
     */
    bool calibrateHelper(CameraPtr& camera,
                         std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs) const;

    /**
     * 优化函数，对标定结果进行非线性优化
     * @param camera 相机对象指针
     * @param rvecs 旋转向量序列
     * @param tvecs 平移向量序列
     */
    void optimize(CameraPtr& camera,
                  std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs) const;

    /**
     * 模板函数：从文件读取数据
     * @param ifs 输入文件流
     * @param data 要读取的数据引用
     */
    template<typename T>
    void readData(std::ifstream& ifs, T& data) const;

    /**
     * 模板函数：向文件写入数据
     * @param ofs 输出文件流
     * @param data 要写入的数据
     */
    template<typename T>
    void writeData(std::ofstream& ofs, T data) const;

    // 私有成员变量：

    // 标定板尺寸（列数×行数），如9×6表示每行9个角点，每列6个角点
    cv::Size m_boardSize;
    // 标定板方格的实际物理尺寸（单位：米）
    float m_squareSize;

    // 相机模型智能指针，包含内参、畸变系数等
    CameraPtr m_camera;
    // 相机位姿矩阵，每行代表一个相机姿态
    cv::Mat m_cameraPoses;

    // 图像点坐标集合：每个元素是一幅图像中检测到的角点像素坐标
    // 格式：vector<每幅图像的角点向量>
    std::vector<std::vector<cv::Point2f> > m_imagePoints;
    // 场景点坐标集合：每个元素是标定板角点的世界坐标
    // 格式：vector<每幅图像对应的世界坐标点向量>
    std::vector<std::vector<cv::Point3f> > m_scenePoints;

    // 测量协方差矩阵，用于描述角点检测的噪声特性
    Eigen::Matrix2d m_measurementCovariance;

    // 是否输出详细信息的标志位
    bool m_verbose;
};

} // namespace camodocal

#endif
