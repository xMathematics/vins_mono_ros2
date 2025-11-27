#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging_macros.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cv_bridge/cv_bridge.h>

#include "feature_tracker.h"
// 定义是否显示去畸变效果的宏，0表示不显示
#define SHOW_UNDISTORTION 0

// 全局变量定义

// 光流跟踪状态向量，用于存储LK光流跟踪结果
vector<uchar> r_status;
// 光流跟踪误差向量，存储每个特征点的跟踪误差
vector<float> r_err;
// 图像消息缓冲区队列，用于存储接收到的图像消息
queue<sensor_msgs::msg::Image::ConstPtr> img_buf;

// ROS2发布器：发布特征点云消息
rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_img;
// ROS2发布器：发布带特征点跟踪可视化的图像
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_match;
// ROS2发布器：发布重启标志（当检测到图像流异常时）
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_restart;

// 特征跟踪器数组，每个相机一个跟踪器实例
FeatureTracker trackerData[NUM_OF_CAM];
// 第一帧图像的时间戳
double first_image_time;
// 发布计数器，用于频率控制
int pub_count = 1;
// 第一帧图像标志
bool first_image_flag = true;
// 上一帧图像的时间戳
double last_image_time = 0;
// 初始化发布标志，跳过第一帧发布
bool init_pub = 0;

/**
 * 图像回调函数：处理接收到的图像消息
 * 这是特征跟踪节点的核心处理函数
 * @param img_msg 接收到的图像消息
 */
void img_callback(const sensor_msgs::msg::Image::SharedPtr img_msg)
{
    // 处理第一帧图像
    if(first_image_flag)
    {
        first_image_flag = false;  // 清除第一帧标志
        // 计算第一帧图像的时间戳（秒）
        first_image_time = img_msg->header.stamp.sec+img_msg->header.stamp.nanosec*(1e-9);
        last_image_time = img_msg->header.stamp.sec+img_msg->header.stamp.nanosec*(1e-9);
        return;  // 第一帧不进行处理，只记录时间
    }

    // 检测不稳定的相机数据流（图像间断或时间戳异常）
    // 图像流异常检测
    // 检测条件：
    // 1. 时间间隔 > 1.0秒（图像丢失）
    // 2. 时间戳回退（时间戳异常）
    // 响应：发布重启标志，重置跟踪器
    if (img_msg->header.stamp.sec+img_msg->header.stamp.nanosec*(1e-9) - last_image_time > 1.0 || 
        img_msg->header.stamp.sec+img_msg->header.stamp.nanosec*(1e-9) < last_image_time)
    {
        RCUTILS_LOG_WARN("image discontinue! reset the feature tracker!");
        // 重置跟踪器状态
        first_image_flag = true; 
        last_image_time = 0;
        pub_count = 1;
        // 发布重启标志，通知其他节点重新初始化
        std_msgs::msg::Bool restart_flag;
        restart_flag.data = true;
        pub_restart->publish(restart_flag);
        return;
    }

    // 更新上一帧图像时间戳
    last_image_time = img_msg->header.stamp.sec+img_msg->header.stamp.nanosec*(1e-9);
    
    // 频率控制：根据配置的FREQ参数控制处理频率
    // 目标：维持稳定的处理频率FREQ
    // 方法：计算当前平均频率，动态调整PUB_THIS_FRAME
    if (round(1.0 * pub_count / (img_msg->header.stamp.sec+img_msg->header.stamp.nanosec*(1e-9) - first_image_time)) <= FREQ)
    {
        PUB_THIS_FRAME = true;  // 标记需要发布当前帧
        
        // 重置频率控制（当实际频率接近目标频率时）
        if (abs(1.0 * pub_count / (img_msg->header.stamp.sec+img_msg->header.stamp.nanosec*(1e-9) - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = img_msg->header.stamp.sec+img_msg->header.stamp.nanosec*(1e-9);
            pub_count = 0;
        }
    }
    else
        PUB_THIS_FRAME = false;  // 跳过当前帧的处理和发布

    // 将ROS图像消息转换为OpenCV图像
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")  // 处理8UC1编码格式
    {
        sensor_msgs::msg::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";  // 转换为mono8格式
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat show_img = ptr->image;  // 获取OpenCV图像
    
    TicToc t_r;  // 计时器，测量整个处理时间
    
    // 处理每个相机的图像
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        RCUTILS_LOG_DEBUG("processing camera %d", i);
        // 多相机处理逻辑
        // 单目模式：每个相机独立处理
        // 双目模式：第二个相机特殊处理（可能用于立体匹配）
        if (i != 1 || !STEREO_TRACK)  // 单目模式或非第二个相机
        // 正常处理
            // 读取图像并处理（从多相机图像中提取对应区域）
            trackerData[i].readImage(ptr->image.rowRange(ROW * i, ROW * (i + 1)), 
                                   img_msg->header.stamp.sec+img_msg->header.stamp.nanosec * (1e-9));
        else  // 双目模式的第二个相机特殊处理
        {
            if (EQUALIZE)  // 如果启用直方图均衡化
            {
                cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
                clahe->apply(ptr->image.rowRange(ROW * i, ROW * (i + 1)), trackerData[i].cur_img);
            }
            else
                trackerData[i].cur_img = ptr->image.rowRange(ROW * i, ROW * (i + 1));
        }

// 如果启用去畸变显示，显示去畸变效果（用于调试）
#if SHOW_UNDISTORTION
        trackerData[i].showUndistortion("undistrotion_" + std::to_string(i));
#endif
    }

    // 特征点发布策略
    // 发布条件：
    // 1. track_cnt > 1（至少跟踪2帧）
    // 2. 不是第一帧（init_pub == 1）
    // 3. PUB_THIS_FRAME == true（频率控制）

    // 发布内容：
    // - 归一化平面坐标（x, y, 1）
    // - 特征点ID（编码为p_id * NUM_OF_CAM + i）
    // - 像素坐标(u, v)
    // - 特征点速度(vx, vy)

    // 为所有相机的新特征点分配ID
    // 为所有相机的新特征点统一分配ID
    // 避免不同相机间的ID冲突
    // 使用completed标志确保所有相机都处理完毕
    for (unsigned int i = 0;; i++)
    {
        bool completed = false;
        // 遍历所有相机，更新特征点ID
        for (int j = 0; j < NUM_OF_CAM; j++)
            if (j != 1 || !STEREO_TRACK)
                completed |= trackerData[j].updateID(i);  // 使用位或操作累积结果
        if (!completed)  // 如果所有相机都没有更多的特征点需要更新ID，退出循环
            break;
    }

   // 如果需要发布当前帧
   if (PUB_THIS_FRAME)
   {
        pub_count++;  // 增加发布计数器
        
        // 创建特征点云消息
        sensor_msgs::msg::PointCloud::SharedPtr feature_points(new sensor_msgs::msg::PointCloud);
        // 创建通道数据：特征点ID
        sensor_msgs::msg::ChannelFloat32 id_of_point;
        // 创建通道数据：特征点u坐标（像素）
        sensor_msgs::msg::ChannelFloat32 u_of_point;
        // 创建通道数据：特征点v坐标（像素）  
        sensor_msgs::msg::ChannelFloat32 v_of_point;
        // 创建通道数据：特征点x方向速度
        sensor_msgs::msg::ChannelFloat32 velocity_x_of_point;
        // 创建通道数据：特征点y方向速度
        sensor_msgs::msg::ChannelFloat32 velocity_y_of_point;

        // 设置消息头
        feature_points->header = img_msg->header;
        feature_points->header.frame_id = "world";

        // 创建哈希集合用于存储每个相机的特征点ID（避免重复）
        vector<set<int>> hash_ids(NUM_OF_CAM);
        
        // 遍历所有相机，收集特征点数据
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            auto &un_pts = trackerData[i].cur_un_pts;      // 去畸变特征点
            auto &cur_pts = trackerData[i].cur_pts;        // 当前特征点像素坐标
            auto &ids = trackerData[i].ids;                // 特征点ID
            auto &pts_velocity = trackerData[i].pts_velocity;  // 特征点速度
            
            for (unsigned int j = 0; j < ids.size(); j++)
            {
                // 只发布跟踪计数大于1的特征点（至少被跟踪2帧）
                if (trackerData[i].track_cnt[j] > 1)
                {
                    int p_id = ids[j];
                    hash_ids[i].insert(p_id);  // 将ID加入哈希集合
                    
                    // 创建几何点（归一化平面坐标，z=1）
                    geometry_msgs::msg::Point32 p;
                    p.x = un_pts[j].x;
                    p.y = un_pts[j].y;
                    p.z = 1;

                    // 添加到点云
                    feature_points->points.push_back(p);
                    // 编码ID：p_id * NUM_OF_CAM + i，避免不同相机ID冲突
                    id_of_point.values.push_back(p_id * NUM_OF_CAM + i);
                    u_of_point.values.push_back(cur_pts[j].x);        // 像素u坐标
                    v_of_point.values.push_back(cur_pts[j].y);        // 像素v坐标
                    velocity_x_of_point.values.push_back(pts_velocity[j].x);  // x速度
                    velocity_y_of_point.values.push_back(pts_velocity[j].y);  // y速度
                }
            }
        }

        // 添加所有通道到点云消息
        feature_points->channels.push_back(id_of_point);
        feature_points->channels.push_back(u_of_point);
        feature_points->channels.push_back(v_of_point);
        feature_points->channels.push_back(velocity_x_of_point);
        feature_points->channels.push_back(velocity_y_of_point);

        RCUTILS_LOG_DEBUG("publish %f, at %f", feature_points->header.stamp.sec+feature_points->header.stamp.nanosec * (1e-9), rclcpp::Clock().now().nanoseconds()*(1e-9));
        
        // 跳过第一帧发布（因为没有速度信息）
        if (!init_pub)
        {
            init_pub = 1;  // 设置初始化发布标志
        }
        else
            pub_img->publish(*feature_points);  // 发布特征点云

        // 如果启用跟踪可视化
        // 颜色编码：
        // 蓝色(0,0,255) -> 新特征点（track_cnt小）
        // 红色(255,0,0) -> 稳定特征点（track_cnt大）
        // 渐变：根据track_cnt/WINDOW_SIZE计算
        if (SHOW_TRACK)
        {
            // 将灰度图转换为BGR彩色图用于可视化
            ptr = cv_bridge::cvtColor(ptr, sensor_msgs::image_encodings::BGR8);
            cv::Mat stereo_img = ptr->image;

            // 为每个相机绘制特征点
            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                // 提取对应相机的图像区域
                cv::Mat tmp_img = stereo_img.rowRange(i * ROW, (i + 1) * ROW);
                // 转换为彩色图（如果原来是灰度）
                cv::cvtColor(show_img, tmp_img, CV_GRAY2RGB);

                // 绘制当前相机的所有特征点
                for (unsigned int j = 0; j < trackerData[i].cur_pts.size(); j++)
                {
                    // 根据跟踪计数计算颜色（蓝色->红色渐变）
                    // 新特征点：蓝色，长时间跟踪的特征点：红色
                    double len = std::min(1.0, 1.0 * trackerData[i].track_cnt[j] / WINDOW_SIZE);
                    cv::circle(tmp_img, trackerData[i].cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
                    
                    // 可选的：绘制速度线（显示特征点运动方向）
                    /*
                    Vector2d tmp_cur_un_pts (trackerData[i].cur_un_pts[j].x, trackerData[i].cur_un_pts[j].y);
                    Vector2d tmp_pts_velocity (trackerData[i].pts_velocity[j].x, trackerData[i].pts_velocity[j].y);
                    Vector3d tmp_prev_un_pts;
                    tmp_prev_un_pts.head(2) = tmp_cur_un_pts - 0.10 * tmp_pts_velocity;
                    tmp_prev_un_pts.z() = 1;
                    Vector2d tmp_prev_uv;
                    trackerData[i].m_camera->spaceToPlane(tmp_prev_un_pts, tmp_prev_uv);
                    cv::line(tmp_img, trackerData[i].cur_pts[j], cv::Point2f(tmp_prev_uv.x(), tmp_prev_uv.y()), cv::Scalar(255 , 0, 0), 1 , 8, 0);
                    */
                    
                    // 可选的：在特征点旁边显示ID
                    //char name[10];
                    //sprintf(name, "%d", trackerData[i].ids[j]);
                    //cv::putText(tmp_img, name, trackerData[i].cur_pts[j], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
                }
            }
            // 发布可视化图像
            pub_match->publish(*(ptr->toImageMsg()));
        }
    }
    // RCUTILS_LOG_INFO("whole feature tracker processing costs: %fms", t_r.toc());
}

/**
 * 主函数：特征跟踪节点入口
 */
int main(int argc, char **argv)
{
    // 初始化ROS2
    rclcpp::init(argc, argv);
    auto n = rclcpp::Node::make_shared("feature_tracker");

    // 读取参数
    readParameters(n);

    // 为每个相机读取内参
    for (int i = 0; i < NUM_OF_CAM; i++)
        trackerData[i].readIntrinsicParameter(CAM_NAMES[i]);

    // 如果是鱼眼相机，加载鱼眼掩膜
    if(FISHEYE)
    {
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            trackerData[i].fisheye_mask = cv::imread(FISHEYE_MASK, 0);  // 以灰度模式读取
            if(!trackerData[i].fisheye_mask.data)
            {
                RCUTILS_LOG_INFO("load mask fail");
            }
            else
                RCUTILS_LOG_INFO("load mask success");
        }
    }

    // 创建图像订阅器
    auto sub_img = n->create_subscription<sensor_msgs::msg::Image>(IMAGE_TOPIC, rclcpp::QoS(rclcpp::KeepLast(100)), img_callback);

    // 创建发布器
    pub_img = n->create_publisher<sensor_msgs::msg::PointCloud>("feature", 1000);
    pub_match = n->create_publisher<sensor_msgs::msg::Image>("feature_img",1000);
    pub_restart = n->create_publisher<std_msgs::msg::Bool>("restart",1000);

    // 启动ROS2节点循环
    rclcpp::spin(n);
    return 0;
}

// 注释中提出的问题：
// 新特征点的速度为0，是否发布？
// 跟踪计数大于1的特征点才发布？