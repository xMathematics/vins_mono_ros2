#include "CameraPoseVisualization.h"
// 静态成员变量初始化：定义相机图像平面的四个角点坐标（在相机坐标系下）
// 这些点定义了相机视锥体的近平面（图像平面）
const Eigen::Vector3d CameraPoseVisualization::imlt = Eigen::Vector3d(-1.0, -0.5, 1.0);  // 图像左上角 (Image Left-Top)
const Eigen::Vector3d CameraPoseVisualization::imrt = Eigen::Vector3d( 1.0, -0.5, 1.0);  // 图像右上角 (Image Right-Top)
const Eigen::Vector3d CameraPoseVisualization::imlb = Eigen::Vector3d(-1.0,  0.5, 1.0);  // 图像左下角 (Image Left-Bottom)
const Eigen::Vector3d CameraPoseVisualization::imrb = Eigen::Vector3d( 1.0,  0.5, 1.0);  // 图像右下角 (Image Right-Bottom)

// 定义相机视锥体的其他辅助点，用于绘制更详细的相机形状
const Eigen::Vector3d CameraPoseVisualization::lt0 = Eigen::Vector3d(-0.7, -0.5, 1.0);  // 辅助点0
const Eigen::Vector3d CameraPoseVisualization::lt1 = Eigen::Vector3d(-0.7, -0.2, 1.0);  // 辅助点1
const Eigen::Vector3d CameraPoseVisualization::lt2 = Eigen::Vector3d(-1.0, -0.2, 1.0);  // 辅助点2

// 定义相机光学中心位置（在相机坐标系下，通常为原点）
const Eigen::Vector3d CameraPoseVisualization::oc = Eigen::Vector3d(0.0, 0.0, 0.0);  // 光学中心 (Optical Center)

// 工具函数：将Eigen向量转换为ROS geometry_msgs::Point类型
// 参数：v - 输入的Eigen向量
//       p - 输出的ROS Point（引用传递，直接修改）
void Eigen2Point(const Eigen::Vector3d& v, geometry_msgs::msg::Point& p) {
    p.x = v.x();  // 将Eigen向量的x分量赋给Point的x
    p.y = v.y();  // 将Eigen向量的y分量赋给Point的y
    p.z = v.z();  // 将Eigen向量的z分量赋给Point的z
}

// 构造函数：初始化相机位姿可视化器
// 参数：r, g, b, a - RGBA颜色值
CameraPoseVisualization::CameraPoseVisualization(float r, float g, float b, float a)
    : m_marker_ns("CameraPoseVisualization"),  // 初始化标记命名空间
      m_scale(0.2),                           // 初始化缩放比例为0.2
      m_line_width(0.01) {                    // 初始化线宽为0.01
    // 设置图像边界颜色
    m_image_boundary_color.r = r;  // 红色分量
    m_image_boundary_color.g = g;  // 绿色分量
    m_image_boundary_color.b = b;  // 蓝色分量
    m_image_boundary_color.a = a;  // 透明度分量
    
    // 设置光学中心连接线颜色（使用相同的颜色）
    m_optical_center_connector_color.r = r;  // 红色分量
    m_optical_center_connector_color.g = g;  // 绿色分量
    m_optical_center_connector_color.b = b;  // 蓝色分量
    m_optical_center_connector_color.a = a;  // 透明度分量
    
    LOOP_EDGE_NUM = 20;        // 初始化回环边的数量限制为20
    tmp_loop_edge_num = 1;     // 初始化临时回环边计数器为1
}

// 设置图像边界颜色的函数
// 参数：r, g, b, a - RGBA颜色值
void CameraPoseVisualization::setImageBoundaryColor(float r, float g, float b, float a) {
    m_image_boundary_color.r = r;  // 设置红色分量
    m_image_boundary_color.g = g;  // 设置绿色分量
    m_image_boundary_color.b = b;  // 设置蓝色分量
    m_image_boundary_color.a = a;  // 设置透明度分量
}

// 设置光学中心连接线颜色的函数
// 参数：r, g, b, a - RGBA颜色值
void CameraPoseVisualization::setOpticalCenterConnectorColor(float r, float g, float b, float a) {
    m_optical_center_connector_color.r = r;  // 设置红色分量
    m_optical_center_connector_color.g = g;  // 设置绿色分量
    m_optical_center_connector_color.b = b;  // 设置蓝色分量
    m_optical_center_connector_color.a = a;  // 设置透明度分量
}

// 设置整体缩放比例的函数
// 参数：s - 缩放比例因子
void CameraPoseVisualization::setScale(double s) {
    m_scale = s;  // 更新缩放比例
}

// 设置线条宽度的函数
// 参数：width - 线条宽度
void CameraPoseVisualization::setLineWidth(double width) {
    m_line_width = width;  // 更新线条宽度
}

// 添加普通边的函数（连接两个点的直线）
// 参数：p0, p1 - 边的起点和终点坐标
void CameraPoseVisualization::add_edge(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1){
    // 创建新的标记对象
    visualization_msgs::msg::Marker marker;

    // 设置标记的命名空间
    marker.ns = m_marker_ns;
    // 设置标记的ID（使用当前标记数量+1）
    marker.id = m_markers.size() + 1;
    // 设置标记类型为线列表（LINE_LIST：每两个点构成一条独立的线段）
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    // 设置动作为添加（ADD）
    marker.action = visualization_msgs::msg::Marker::ADD;
    // 设置线宽
    marker.scale.x = 0.01;

    // 设置颜色为蓝色
    marker.color.b = 1.0f;  // 蓝色分量设为1.0
    marker.color.a = 1.0;   // 不透明度设为1.0（完全不透明）

    // 定义两个点用于存储转换后的坐标
    geometry_msgs::msg::Point point0, point1;

    // 将Eigen向量转换为ROS Point类型
    Eigen2Point(p0, point0);
    Eigen2Point(p1, point1);

    // 将两个点添加到标记的点列表中（LINE_LIST需要成对的点）
    marker.points.push_back(point0);
    marker.points.push_back(point1);

    // 将标记添加到标记列表中
    m_markers.push_back(marker);
}

// 添加回环边的函数（用于SLAM中的回环检测可视化）
// 参数：p0, p1 - 回环边的起点和终点坐标
void CameraPoseVisualization::add_loopedge(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1){
    // 注释掉的代码：清除所有标记（如果需要重新开始）
    //m_markers.clear();
    
    // 创建新的标记对象
    visualization_msgs::msg::Marker marker;

    // 设置标记的命名空间
    marker.ns = m_marker_ns;
    // 设置标记的ID（使用当前标记数量+1）
    marker.id = m_markers.size() + 1;
    
    // 注释掉的回环边计数逻辑
    //tmp_loop_edge_num++;
    //if(tmp_loop_edge_num >= LOOP_EDGE_NUM)
    //  tmp_loop_edge_num = 1;
    
    // 设置标记类型为线带（LINE_STRIP：所有点连接成一条连续的折线）
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    // 设置动作为添加（ADD）
    marker.action = visualization_msgs::msg::Marker::ADD;
    // 设置生存时间为0（永久显示）
    marker.lifetime = rclcpp::Duration::from_seconds(0);
    // 设置线宽为0.02
    marker.scale.x = 0.02;
    // 设置颜色为红色
    marker.color.r = 1.0f;  // 红色分量设为1.0
    // 注释掉的其他颜色设置
    //marker.color.g = 1.0f;
    //marker.color.b = 1.0f;
    marker.color.a = 1.0;   // 不透明度设为1.0

    // 定义两个点用于存储转换后的坐标
    geometry_msgs::msg::Point point0, point1;

    // 将Eigen向量转换为ROS Point类型
    Eigen2Point(p0, point0);
    Eigen2Point(p1, point1);

    // 将两个点添加到标记的点列表中（LINE_STRIP会连接所有点成一条线）
    marker.points.push_back(point0);
    marker.points.push_back(point1);

    // 将标记添加到标记列表中
    m_markers.push_back(marker);
}
// 添加相机位姿到可视化中的函数
// 参数：p - 相机位置（世界坐标系下的平移向量）
//       q - 相机朝向（世界坐标系下的四元数旋转）
void CameraPoseVisualization::add_pose(const Eigen::Vector3d& p, const Eigen::Quaterniond& q) {
    // 创建新的可视化标记
    visualization_msgs::msg::Marker marker;

    // 设置标记的命名空间
    marker.ns = m_marker_ns;
    // 设置标记ID为0（所有相机位姿共享同一个ID，每次添加会覆盖之前的）
    marker.id = 0;
    // 设置标记类型为线带（LINE_STRIP：连续线段）
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    // 设置动作为添加
    marker.action = visualization_msgs::msg::Marker::ADD;
    // 设置线宽
    marker.scale.x = m_line_width;

    // 设置标记的位姿（这里设为单位位姿，因为点的坐标已经在世界坐标系中计算好了）
    marker.pose.position.x = 0.0;      // 位置x设为0
    marker.pose.position.y = 0.0;      // 位置y设为0
    marker.pose.position.z = 0.0;      // 位置z设为0
    marker.pose.orientation.w = 1.0;   // 四元数w分量设为1（无旋转）
    marker.pose.orientation.x = 0.0;   // 四元数x分量设为0
    marker.pose.orientation.y = 0.0;   // 四元数y分量设为0
    marker.pose.orientation.z = 0.0;   // 四元数z分量设为0

    // 定义各个特征点的变量
    geometry_msgs::msg::Point pt_lt, pt_lb, pt_rt, pt_rb, pt_oc, pt_lt0, pt_lt1, pt_lt2;

    // 将相机坐标系下的特征点转换到世界坐标系：
    // 1. 先缩放：m_scale * point
    // 2. 再旋转：q * (缩放后的点)  
    // 3. 再平移：+ p
    // 4. 最后转换为ROS Point类型
    Eigen2Point(q * (m_scale *imlt) + p, pt_lt);  // 图像左上角
    Eigen2Point(q * (m_scale *imlb) + p, pt_lb);  // 图像左下角
    Eigen2Point(q * (m_scale *imrt) + p, pt_rt);  // 图像右上角
    Eigen2Point(q * (m_scale *imrb) + p, pt_rb);  // 图像右下角
    Eigen2Point(q * (m_scale *lt0 ) + p, pt_lt0); // 辅助点0
    Eigen2Point(q * (m_scale *lt1 ) + p, pt_lt1); // 辅助点1
    Eigen2Point(q * (m_scale *lt2 ) + p, pt_lt2); // 辅助点2
    Eigen2Point(q * (m_scale *oc  ) + p, pt_oc);  // 光学中心

    // 绘制图像边界（矩形框）
    // 左上角 -> 左下角
    marker.points.push_back(pt_lt);  // 添加起点
    marker.points.push_back(pt_lb);  // 添加终点
    marker.colors.push_back(m_image_boundary_color);  // 起点颜色
    marker.colors.push_back(m_image_boundary_color);  // 终点颜色

    // 左下角 -> 右下角
    marker.points.push_back(pt_lb);  // 添加起点
    marker.points.push_back(pt_rb);  // 添加终点
    marker.colors.push_back(m_image_boundary_color);  // 起点颜色
    marker.colors.push_back(m_image_boundary_color);  // 终点颜色

    // 右下角 -> 右上角
    marker.points.push_back(pt_rb);  // 添加起点
    marker.points.push_back(pt_rt);  // 添加终点
    marker.colors.push_back(m_image_boundary_color);  // 起点颜色
    marker.colors.push_back(m_image_boundary_color);  // 终点颜色

    // 右上角 -> 左上角
    marker.points.push_back(pt_rt);  // 添加起点
    marker.points.push_back(pt_lt);  // 添加终点
    marker.colors.push_back(m_image_boundary_color);  // 起点颜色
    marker.colors.push_back(m_image_boundary_color);  // 终点颜色

    // 绘制左上角指示器（L形状，用于标识相机朝向）
    // 水平线段
    marker.points.push_back(pt_lt0);  // 起点
    marker.points.push_back(pt_lt1);  // 终点
    marker.colors.push_back(m_image_boundary_color);  // 起点颜色
    marker.colors.push_back(m_image_boundary_color);  // 终点颜色

    // 垂直线段
    marker.points.push_back(pt_lt1);  // 起点
    marker.points.push_back(pt_lt2);  // 终点
    marker.colors.push_back(m_image_boundary_color);  // 起点颜色
    marker.colors.push_back(m_image_boundary_color);  // 终点颜色

    // 绘制光学中心连接线（从图像角点到光学中心的线，形成金字塔形状）
    // 左上角 -> 光学中心
    marker.points.push_back(pt_lt);  // 起点
    marker.points.push_back(pt_oc);  // 终点
    marker.colors.push_back(m_optical_center_connector_color);  // 起点颜色
    marker.colors.push_back(m_optical_center_connector_color);  // 终点颜色

    // 左下角 -> 光学中心
    marker.points.push_back(pt_lb);  // 起点
    marker.points.push_back(pt_oc);  // 终点
    marker.colors.push_back(m_optical_center_connector_color);  // 起点颜色
    marker.colors.push_back(m_optical_center_connector_color);  // 终点颜色

    // 右上角 -> 光学中心
    marker.points.push_back(pt_rt);  // 起点
    marker.points.push_back(pt_oc);  // 终点
    marker.colors.push_back(m_optical_center_connector_color);  // 起点颜色
    marker.colors.push_back(m_optical_center_connector_color);  // 终点颜色

    // 右下角 -> 光学中心
    marker.points.push_back(pt_rb);  // 起点
    marker.points.push_back(pt_oc);  // 终点
    marker.colors.push_back(m_optical_center_connector_color);  // 起点颜色
    marker.colors.push_back(m_optical_center_connector_color);  // 终点颜色

    // 将完整的相机位姿标记添加到标记列表中
    m_markers.push_back(marker);
}

// 重置函数：清除所有存储的可视化标记
void CameraPoseVisualization::reset() {
    // 清除所有标记
    m_markers.clear();
    // 注释掉的图像相关清理代码
    //image.points.clear();
    //image.colors.clear();
}

// 通过发布器发布所有标记
// 参数：pub    - ROS2 MarkerArray发布器
//       header - 消息头（包含时间戳和坐标系信息）
void CameraPoseVisualization::publish_by( rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub, const std_msgs::msg::Header &header ) {
    // 创建MarkerArray消息
    visualization_msgs::msg::MarkerArray markerArray_msg;
    
    // 注释掉的限制发布数量的代码（只发布最近5个标记）
    //int k = (int)m_markers.size();
    /*
    for (int i = 0; i < 5 && k > 0; i++)
    {
        k--;
        m_markers[k].header = header;
        markerArray_msg.markers.push_back(m_markers[k]);
    }
    */

    // 遍历所有标记，设置消息头并添加到MarkerArray中
    for(auto& marker : m_markers) {
        marker.header = header;  // 设置每个标记的消息头
        markerArray_msg.markers.push_back(marker);  // 添加到MarkerArray
    }
    
    // 发布MarkerArray消息
    pub->publish(markerArray_msg);
}

// 发布图像标记的函数
// 参数：pub    - ROS2 Marker发布器  
//       header - 消息头
void CameraPoseVisualization::publish_image_by( rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub, const std_msgs::msg::Header &header ) {
    // 设置图像标记的消息头
    image.header = header;
    // 发布图像标记
    pub->publish(image);
}

/*
// 注释掉的添加图像纹理的函数
// 参数：T   - 相机位置
//       R   - 相机旋转矩阵  
//       src - OpenCV图像数据
void CameraPoseVisualization::add_image(const Eigen::Vector3d& T, const Eigen::Matrix3d& R, const cv::Mat &src)
{
    // 清理图像点云和颜色数据
    //image.points.clear();
    //image.colors.clear();

    // 设置图像标记属性
    image.ns = "image";  // 命名空间
    image.id = 0;        // ID
    image.action = visualization_msgs::Marker::ADD;  // 动作类型
    image.type = visualization_msgs::Marker::TRIANGLE_LIST;  // 三角形列表
    image.scale.x = 1;   // 缩放x
    image.scale.y = 1;   // 缩放y  
    image.scale.z = 1;   // 缩放z

    // 临时变量
    geometry_msgs::Point p;
    std_msgs::ColorRGBA crgb;

    // 计算图像中心
    double center_x = src.rows / 2.0;
    double center_y = src.cols / 2.0;

    // 设置缩放比例
    //double scale = 0.01;
    double scale = IMAGE_VISUAL_SCALE;

    // 遍历图像的每个像素，生成三角形网格
    for(int r = 0; r < src.cols; ++r) {
        for(int c = 0; c < src.rows; ++c) {
            // 获取像素强度值并转换为颜色
            float intensity = (float)( src.at<uchar>(c, r));
            crgb.r = (float)intensity / 255.0;  // 红色分量
            crgb.g = (float)intensity / 255.0;  // 绿色分量
            crgb.b = (float)intensity / 255.0;  // 蓝色分量
            crgb.a = 1.0;                       // 透明度

            // 定义6个点来构成两个三角形（形成一个正方形像素）
            Eigen::Vector3d p_cam, p_w;
            
            // 第一个三角形的三个顶点
            p_cam.z() = 0;
            p_cam.x() = (r - center_x) * scale;
            p_cam.y() = (c - center_y) * scale; 
            p_w = R * p_cam + T;
            p.x = p_w(0); p.y = p_w(1); p.z = p_w(2);
            image.points.push_back(p);
            image.colors.push_back(crgb);

            p_cam.z() = 0;
            p_cam.x() = (r - center_x + 1) * scale;
            p_cam.y() = (c - center_y) * scale; 
            p_w = R * p_cam + T;
            p.x = p_w(0); p.y = p_w(1); p.z = p_w(2);
            image.points.push_back(p);
            image.colors.push_back(crgb);

            p_cam.z() = 0;
            p_cam.x() = (r - center_x) * scale;
            p_cam.y() = (c - center_y + 1) * scale; 
            p_w = R * p_cam + T;
            p.x = p_w(0); p.y = p_w(1); p.z = p_w(2);
            image.points.push_back(p);
            image.colors.push_back(crgb);

            // 第二个三角形的三个顶点
            p_cam.z() = 0;
            p_cam.x() = (r - center_x + 1) * scale;
            p_cam.y() = (c - center_y) * scale; 
            p_w = R * p_cam + T;
            p.x = p_w(0); p.y = p_w(1); p.z = p_w(2);
            image.points.push_back(p);
            image.colors.push_back(crgb);

            p_cam.z() = 0;
            p_cam.x() = (r - center_x + 1) * scale;
            p_cam.y() = (c - center_y + 1) * scale; 
            p_w = R * p_cam + T;
            p.x = p_w(0); p.y = p_w(1); p.z = p_w(2);
            image.points.push_back(p);
            image.colors.push_back(crgb);

            p_cam.z() = 0;
            p_cam.x() = (r - center_x) * scale;
            p_cam.y() = (c - center_y + 1) * scale; 
            p_w = R * p_cam + T;
            p.x = p_w(0); p.y = p_w(1); p.z = p_w(2);
            image.points.push_back(p);
            image.colors.push_back(crgb);
        }
    }
}
*/