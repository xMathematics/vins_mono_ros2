#include "pose_graph.h"

// PoseGraph类的构造函数
PoseGraph::PoseGraph()
{
    // 创建位姿图可视化对象，设置颜色为紫色(1.0, 0.0, 1.0, 1.0) RGBA
    posegraph_visualization = new CameraPoseVisualization(1.0, 0.0, 1.0, 1.0);
    
    // 设置可视化标记的缩放比例为0.1
    posegraph_visualization->setScale(0.1);
    
    // 设置可视化线条的宽度为0.01
    posegraph_visualization->setLineWidth(0.01);
    
    // 创建优化线程，该线程将执行optimize4DoF成员函数
    // std::thread参数说明：
    // &PoseGraph::optimize4DoF - 成员函数指针
    // this - 当前对象指针（作为隐式参数传递给成员函数）
    t_optimization = std::thread(&PoseGraph::optimize4DoF, this);
    
    // 初始化最早回环索引为-1，表示尚未检测到回环
    earliest_loop_index = -1;
    
    // 初始化平移漂移量为零向量
    t_drift = Eigen::Vector3d(0, 0, 0);
    
    // 初始化偏航角漂移量为0
    yaw_drift = 0;
    
    // 初始化旋转漂移量为单位矩阵（无旋转）
    r_drift = Eigen::Matrix3d::Identity();
    
    // 初始化世界坐标系到VIO坐标系的平移为零向量
    w_t_vio = Eigen::Vector3d(0, 0, 0);
    
    // 初始化世界坐标系到VIO坐标系的旋转为单位矩阵
    w_r_vio = Eigen::Matrix3d::Identity();
    
    // 初始化全局索引为0（用于关键帧编号）
    global_index = 0;
    
    // 初始化序列计数器为0
    sequence_cnt = 0;
    
    // 初始化序列回环标记向量，第一个元素设为0（表示第一个序列）
    sequence_loop.push_back(0);
    
    // 设置基础序列ID为1
    base_sequence = 1;
}

// PoseGraph类的析构函数
PoseGraph::~PoseGraph()
{
    // 等待优化线程结束（join会阻塞直到线程完成）
    // 这是重要的资源清理步骤，确保线程安全退出
    t_optimization.join();
    
    // 注意：这里没有删除posegraph_visualization指针，可能存在内存泄漏
    // 应该添加：delete posegraph_visualization;
}

// 注册ROS2发布器的函数
// 参数：n - ROS2节点的共享指针
void PoseGraph::registerPub(rclcpp::Node::SharedPtr n)
{
    // 创建位姿图路径发布器
    // 参数："pose_graph_path" - 话题名称
    //       1000 - 消息队列大小
    pub_pg_path = n->create_publisher<nav_msgs::msg::Path>("pose_graph_path", 1000);
    
    // 创建基础路径发布器
    pub_base_path = n->create_publisher<nav_msgs::msg::Path>("base_path", 1000);
    
    // 创建位姿图可视化标记数组发布器
    pub_pose_graph = n->create_publisher<visualization_msgs::msg::MarkerArray>("pose_graph", 1000);
    
    // 创建多条路径的发布器（共9条，索引1-9）
    // 用于发布不同序列或不同属性的路径
    for (int i = 1; i < 10; i++)
        // 创建路径发布器，话题名为"path_1"到"path_9"
        pub_path[i] = n->create_publisher<nav_msgs::msg::Path>("path_" + to_string(i), 1000);
        
    // 注意：pub_path[0]没有被初始化，使用时需要注意
}

// 加载词袋词汇表的函数
// 参数：voc_path - 词汇表文件路径
void PoseGraph::loadVocabulary(std::string voc_path)
{
    // 从指定路径创建Brief词汇表对象
    // BriefVocabulary是DBoW2库中的类，用于视觉词袋模型
    voc = new BriefVocabulary(voc_path);
    
    // 设置数据库使用的词汇表
    // 参数说明：
    // *voc - 解引用词汇表指针
    // false - 不使用直接索引（direct index）
    // 0 - 直接索引级别（由于不使用直接索引，此参数被忽略）
    db.setVocabulary(*voc, false, 0);
    
    // 注意：这里没有检查词汇表文件是否加载成功
    // 在实际应用中应该添加错误处理
}
// 向位姿图中添加关键帧的函数
// 参数：cur_kf - 当前关键帧指针
//       flag_detect_loop - 是否进行回环检测的标志
void PoseGraph::addKeyFrame(KeyFrame* cur_kf, bool flag_detect_loop)
{
    // 定义变量：当前关键帧在VIO坐标系下的位置和旋转
    Vector3d vio_P_cur;
    Matrix3d vio_R_cur;
    
    // 检查是否开始了新的序列
    if (sequence_cnt != cur_kf->sequence)
    {
        sequence_cnt++;  // 序列计数器加1
        sequence_loop.push_back(0);  // 为新序列添加回环标记，初始为0（未检测到回环）
        
        // 重置世界坐标系到VIO坐标系的变换（新序列开始时）
        w_t_vio = Eigen::Vector3d(0, 0, 0);      // 平移重置为零
        w_r_vio = Eigen::Matrix3d::Identity();   // 旋转重置为单位矩阵
        
        // 加锁保护漂移量数据
        m_drift.lock();
        t_drift = Eigen::Vector3d(0, 0, 0);      // 平移漂移重置为零
        r_drift = Eigen::Matrix3d::Identity();   // 旋转漂移重置为单位矩阵
        m_drift.unlock();
    }

    // 获取当前关键帧在VIO坐标系下的位姿
    cur_kf->getVioPose(vio_P_cur, vio_R_cur);
    
    // 将VIO坐标系下的位姿转换到世界坐标系
    vio_P_cur = w_r_vio * vio_P_cur + w_t_vio;  // 应用旋转和平移变换
    vio_R_cur = w_r_vio * vio_R_cur;            // 应用旋转变换
    
    // 更新关键帧的世界坐标系位姿
    cur_kf->updateVioPose(vio_P_cur, vio_R_cur);
    
    // 为关键帧分配全局索引并递增
    cur_kf->index = global_index;
    global_index++;
    
    // 初始化回环索引为-1（表示未检测到回环）
    int loop_index = -1;
    
    // 根据标志决定是否进行回环检测
    if (flag_detect_loop)
    {
        TicToc tmp_t;  // 计时器，用于测量回环检测耗时
        // 执行回环检测，返回检测到的回环关键帧索引
        loop_index = detectLoop(cur_kf, cur_kf->index);
    }
    else
    {
        // 如果不检测回环，只将关键帧添加到词袋数据库中
        addKeyFrameIntoVoc(cur_kf);
    }
    
    // 如果检测到回环（loop_index不为-1）
    if (loop_index != -1)
    {
        // 获取回环关键帧对象
        KeyFrame* old_kf = getKeyFrame(loop_index);

        // 尝试在当前关键帧和回环关键帧之间建立连接（特征匹配验证）
        if (cur_kf->findConnection(old_kf))
        {
            // 更新最早回环索引（用于优化窗口）
            if (earliest_loop_index > loop_index || earliest_loop_index == -1)
                earliest_loop_index = loop_index;

            // 定义变量存储位姿信息
            Vector3d w_P_old, w_P_cur, vio_P_cur;
            Matrix3d w_R_old, w_R_cur, vio_R_cur;
            
            // 获取回环关键帧和当前关键帧的位姿
            old_kf->getVioPose(w_P_old, w_R_old);
            cur_kf->getVioPose(vio_P_cur, vio_R_cur);

            // 获取通过特征匹配计算出的相对位姿变换
            Vector3d relative_t;
            Quaterniond relative_q;
            relative_t = cur_kf->getLoopRelativeT();        // 相对平移
            relative_q = (cur_kf->getLoopRelativeQ()).toRotationMatrix();  // 相对旋转（转换为旋转矩阵）
            
            // 根据回环关键帧的位姿和相对变换，计算当前关键帧在世界坐标系下的新位姿
            w_P_cur = w_R_old * relative_t + w_P_old;  // 计算新的位置
            w_R_cur = w_R_old * relative_q;            // 计算新的旋转

            // 计算VIO位姿与回环调整后位姿之间的漂移量
            double shift_yaw;
            Matrix3d shift_r;
            Vector3d shift_t;
            
            // 计算偏航角（yaw）的漂移
            shift_yaw = Utility::R2ypr(w_R_cur).x() - Utility::R2ypr(vio_R_cur).x();
            // 根据偏航角漂移创建旋转矩阵（只考虑yaw，固定pitch和roll）
            shift_r = Utility::ypr2R(Vector3d(shift_yaw, 0, 0));
            // 计算平移漂移
            shift_t = w_P_cur - w_R_cur * vio_R_cur.transpose() * vio_P_cur;
            
            // 如果回环关键帧和当前关键帧不在同一个序列，且当前序列还没有进行过回环调整
            if (old_kf->sequence != cur_kf->sequence && sequence_loop[cur_kf->sequence] == 0)
            {
                // 更新世界坐标系到VIO坐标系的变换（漂移校正）
                w_r_vio = shift_r;   // 更新旋转变换
                w_t_vio = shift_t;   // 更新平移变换
                
                // 用新的漂移校正量重新计算当前关键帧的位姿
                vio_P_cur = w_r_vio * vio_P_cur + w_t_vio;
                vio_R_cur = w_r_vio * vio_R_cur;
                cur_kf->updateVioPose(vio_P_cur, vio_R_cur);
                
                // 遍历关键帧列表，对同一序列的所有关键帧应用相同的漂移校正
                list<KeyFrame*>::iterator it = keyframelist.begin();
                for (; it != keyframelist.end(); it++)
                {
                    // 只处理与当前关键帧同一序列的关键帧
                    if((*it)->sequence == cur_kf->sequence)
                    {
                        Vector3d vio_P_cur;
                        Matrix3d vio_R_cur;
                        (*it)->getVioPose(vio_P_cur, vio_R_cur);
                        // 应用漂移校正
                        vio_P_cur = w_r_vio * vio_P_cur + w_t_vio;
                        vio_R_cur = w_r_vio * vio_R_cur;
                        (*it)->updateVioPose(vio_P_cur, vio_R_cur);
                    }
                }
                // 标记当前序列已经进行过回环调整
                sequence_loop[cur_kf->sequence] = 1;
            }
            
            // 将当前关键帧索引加入优化队列，触发后端优化
            m_optimize_buf.lock();
            optimize_buf.push(cur_kf->index);
            m_optimize_buf.unlock();
        }
    }
    
    // 加锁保护关键帧列表和相关数据
    m_keyframelist.lock();
    
    // 获取当前关键帧的VIO位姿（可能已经经过漂移校正）
    Vector3d P;
    Matrix3d R;
    cur_kf->getVioPose(P, R);
    
    // 应用优化后的漂移量（如果已经进行过优化）
    P = r_drift * P + t_drift;  // 应用旋转漂移和平移漂移
    R = r_drift * R;            // 应用旋转漂移
    
    // 更新关键帧的最终位姿（经过优化后的位姿）
    cur_kf->updatePose(P, R);
    
    // 将旋转矩阵转换为四元数，用于ROS消息
    Quaterniond Q{R};
    
    // 创建ROS位姿消息
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.stamp = rclcpp::Time(cur_kf->time_stamp);  // 设置时间戳
    pose_stamped.header.frame_id = "world";                       // 设置坐标系
    
    // 设置位置（添加可视化偏移，避免重叠）
    pose_stamped.pose.position.x = P.x() + VISUALIZATION_SHIFT_X;
    pose_stamped.pose.position.y = P.y() + VISUALIZATION_SHIFT_Y;
    pose_stamped.pose.position.z = P.z();
    
    // 设置朝向（四元数）
    pose_stamped.pose.orientation.x = Q.x();
    pose_stamped.pose.orientation.y = Q.y();
    pose_stamped.pose.orientation.z = Q.z();
    pose_stamped.pose.orientation.w = Q.w();
    
    // 将位姿添加到对应序列的路径中
    path[sequence_cnt].poses.push_back(pose_stamped);
    path[sequence_cnt].header = pose_stamped.header;

    // 如果设置了保存回环路径，则将位姿保存到文件
    if (SAVE_LOOP_PATH)
    {
        // 打开结果文件（追加模式）
        ofstream loop_path_file(VINS_RESULT_PATH, ios::app);
        loop_path_file.setf(ios::fixed, ios::floatfield);  // 设置固定小数点格式
        loop_path_file.precision(0);                       // 时间戳精度为0（整数）
        loop_path_file << cur_kf->time_stamp * 1e9 << ","; // 写入时间戳（纳秒）
        loop_path_file.precision(5);                       // 位姿精度为5位小数
        
        // 写入位姿数据：位置(x,y,z)和四元数(w,x,y,z)
        loop_path_file  << P.x() << ","
              << P.y() << ","
              << P.z() << ","
              << Q.w() << ","
              << Q.x() << ","
              << Q.y() << ","
              << Q.z() << ","
              << endl;
        loop_path_file.close();  // 关闭文件
    }
    
    // 如果设置了显示短边（相邻关键帧之间的连接），则绘制局部连接
    if (SHOW_S_EDGE)
    {
        // 从关键帧列表末尾开始反向遍历（最新的关键帧在末尾）
        list<KeyFrame*>::reverse_iterator rit = keyframelist.rbegin();
        // 只连接最近的4个关键帧
        for (int i = 0; i < 4; i++)
        {
            if (rit == keyframelist.rend())  // 检查是否到达列表开头
                break;
            
            Vector3d conncected_P;
            Matrix3d connected_R;
            
            // 只连接同一序列的关键帧
            if((*rit)->sequence == cur_kf->sequence)
            {
                (*rit)->getPose(conncected_P, connected_R);
                // 添加边到可视化
                posegraph_visualization->add_edge(P, conncected_P);
            }
            rit++;  // 移动到前一个关键帧
        }
    }
    
    // 如果设置了显示长边（回环连接），并且当前关键帧有回环，则绘制回环边
    if (SHOW_L_EDGE)
    {
        if (cur_kf->has_loop)  // 检查当前关键帧是否有回环
        {
            // 获取回环关键帧
            KeyFrame* connected_KF = getKeyFrame(cur_kf->loop_index);
            Vector3d connected_P, P0;
            Matrix3d connected_R, R0;
            
            // 获取回环关键帧和当前关键帧的位姿
            connected_KF->getPose(connected_P, connected_R);
            cur_kf->getPose(P0, R0);
            
            // 只对非基础序列（sequence > 0）绘制回环边
            if(cur_kf->sequence > 0)
            {
                // 添加回环边到可视化（终点添加可视化偏移）
                posegraph_visualization->add_loopedge(P0, connected_P + Vector3d(VISUALIZATION_SHIFT_X, VISUALIZATION_SHIFT_Y, 0));
            }
        }
    }
    
    // 注释掉的代码：添加单个位姿到可视化
    //posegraph_visualization->add_pose(P + Vector3d(VISUALIZATION_SHIFT_X, VISUALIZATION_SHIFT_Y, 0), Q);

    // 将当前关键帧添加到关键帧列表
    keyframelist.push_back(cur_kf);
    
    // 发布所有可视化消息
    publish();
    
    // 解锁关键帧列表
    m_keyframelist.unlock();
}

// 加载关键帧到位姿图中的函数
// 参数：cur_kf - 当前关键帧指针（通常是从文件或其他来源加载的关键帧）
//       flag_detect_loop - 是否进行回环检测的标志
void PoseGraph::loadKeyFrame(KeyFrame* cur_kf, bool flag_detect_loop)
{
    // 为加载的关键帧分配全局索引并递增
    // 注意：这里直接使用全局索引，没有检查序列变化
    cur_kf->index = global_index;
    global_index++;
    
    // 初始化回环索引为-1（表示未检测到回环）
    int loop_index = -1;
    
    // 根据标志决定是否进行回环检测
    if (flag_detect_loop)
        // 执行回环检测，返回检测到的回环关键帧索引
        loop_index = detectLoop(cur_kf, cur_kf->index);
    else
        // 如果不检测回环，只将关键帧添加到词袋数据库中
        addKeyFrameIntoVoc(cur_kf);
    
    // 如果检测到回环（loop_index不为-1）
    if (loop_index != -1)
    {
        // 输出回环检测结果信息
        printf(" %d detect loop with %d \n", cur_kf->index, loop_index);
        
        // 获取回环关键帧对象
        KeyFrame* old_kf = getKeyFrame(loop_index);
        
        // 尝试在当前关键帧和回环关键帧之间建立连接（特征匹配验证）
        if (cur_kf->findConnection(old_kf))
        {
            // 更新最早回环索引（用于优化窗口）
            if (earliest_loop_index > loop_index || earliest_loop_index == -1)
                earliest_loop_index = loop_index;
            
            // 将当前关键帧索引加入优化队列，触发后端优化
            m_optimize_buf.lock();
            optimize_buf.push(cur_kf->index);
            m_optimize_buf.unlock();
        }
    }
    
    // 加锁保护关键帧列表和相关数据
    m_keyframelist.lock();
    
    // 获取当前关键帧的位姿（注意：这里使用getPose而不是getVioPose）
    // 因为加载的关键帧通常已经包含优化后的位姿
    Vector3d P;
    Matrix3d R;
    cur_kf->getPose(P, R);
    
    // 将旋转矩阵转换为四元数，用于ROS消息
    Quaterniond Q{R};
    
    // 创建ROS位姿消息
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.stamp = rclcpp::Time(cur_kf->time_stamp);  // 设置时间戳
    pose_stamped.header.frame_id = "world";                       // 设置坐标系
    
    // 设置位置（添加可视化偏移，避免重叠）
    pose_stamped.pose.position.x = P.x() + VISUALIZATION_SHIFT_X;
    pose_stamped.pose.position.y = P.y() + VISUALIZATION_SHIFT_Y;
    pose_stamped.pose.position.z = P.z();
    
    // 设置朝向（四元数）
    pose_stamped.pose.orientation.x = Q.x();
    pose_stamped.pose.orientation.y = Q.y();
    pose_stamped.pose.orientation.z = Q.z();
    pose_stamped.pose.orientation.w = Q.w();
    
    // 将位姿添加到基础路径中（base_path用于显示原始轨迹）
    base_path.poses.push_back(pose_stamped);
    base_path.header = pose_stamped.header;

    // 如果设置了显示短边（相邻关键帧之间的连接），则绘制局部连接
    if (SHOW_S_EDGE)
    {
        // 从关键帧列表末尾开始反向遍历（最新的关键帧在末尾）
        list<KeyFrame*>::reverse_iterator rit = keyframelist.rbegin();
        
        // 只连接最近的1个关键帧（与addKeyFrame中的4个不同）
        for (int i = 0; i < 1; i++)
        {
            if (rit == keyframelist.rend())  // 检查是否到达列表开头
                break;
            
            Vector3d conncected_P;
            Matrix3d connected_R;
            
            // 只连接同一序列的关键帧
            if((*rit)->sequence == cur_kf->sequence)
            {
                (*rit)->getPose(conncected_P, connected_R);
                // 添加边到可视化
                posegraph_visualization->add_edge(P, conncected_P);
            }
            rit++;  // 移动到前一个关键帧
        }
    }
    
    /*
    // 注释掉的回环边绘制代码
    // 如果当前关键帧有回环，则绘制回环连接
    if (cur_kf->has_loop)
    {
        KeyFrame* connected_KF = getKeyFrame(cur_kf->loop_index);
        Vector3d connected_P;
        Matrix3d connected_R;
        connected_KF->getPose(connected_P,  connected_R);
        posegraph_visualization->add_loopedge(P, connected_P, SHIFT);
    }
    */

    // 将当前关键帧添加到关键帧列表
    keyframelist.push_back(cur_kf);
    
    // 注释掉的发布函数：通常加载时不立即发布，等待所有数据加载完成后再统一发布
    //publish();
    
    // 解锁关键帧列表
    m_keyframelist.unlock();
}
// 根据索引获取关键帧的函数
// 参数：index - 要查找的关键帧索引
// 返回：找到的关键帧指针，如果未找到则返回NULL
KeyFrame* PoseGraph::getKeyFrame(int index)
{
    // 注释掉的锁：如果需要在多线程环境中使用，应该加锁
    //unique_lock<mutex> lock(m_keyframelist);
    
    // 遍历关键帧列表查找指定索引的关键帧
    list<KeyFrame*>::iterator it = keyframelist.begin();
    for (; it != keyframelist.end(); it++)
    {
        // 检查当前关键帧的索引是否匹配目标索引
        if((*it)->index == index)
            break;  // 找到目标，跳出循环
    }
    
    // 检查是否找到了关键帧
    if (it != keyframelist.end())
        return *it;  // 返回找到的关键帧指针
    else
        return NULL; // 未找到，返回空指针
}

// 回环检测函数
// 参数：keyframe - 待检测的关键帧
//       frame_index - 当前关键帧的索引
// 返回：检测到的回环关键帧索引，-1表示未检测到回环
int PoseGraph::detectLoop(KeyFrame* keyframe, int frame_index)
{
    // 定义压缩图像变量，用于可视化
    cv::Mat compressed_image;
    
    // 如果启用了调试图像功能
    if (DEBUG_IMAGE)
    {
        // 获取当前关键帧的特征点数量
        int feature_num = keyframe->keypoints.size();
        
        // 将原图像缩放到固定尺寸(376x240)，减少内存占用
        cv::resize(keyframe->image, compressed_image, cv::Size(376, 240));
        
        // 在图像上添加特征点数量信息
        putText(compressed_image, "feature_num:" + to_string(feature_num), 
                cv::Point2f(10, 10), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255));
        
        // 将处理后的图像存入图像池，用于后续可视化
        image_pool[frame_index] = compressed_image;
    }
    
    // 计时器，用于测量查询耗时
    TicToc tmp_t;
    
    // 第一步：先查询数据库，寻找相似的关键帧
    QueryResults ret;  // 存储查询结果
    TicToc t_query;    // 查询计时器
    
    // 在词袋数据库中查询与当前关键帧相似的关键帧
    // 参数说明：
    // keyframe->brief_descriptors - 当前关键帧的BRIEF描述子
    // ret - 查询结果容器
    // 4 - 返回最相似的4个结果
    // frame_index - 50 - 排除最近50帧，避免检测到临近的关键帧
    db.query(keyframe->brief_descriptors, ret, 4, frame_index - 50);
    
    // 注释掉的查询时间输出
    //printf("query time: %f", t_query.toc());
    //cout << "Searching for Image " << frame_index << ". " << ret << endl;

    // 第二步：将当前关键帧添加到数据库中（用于后续查询）
    TicToc t_add;  // 添加计时器
    db.add(keyframe->brief_descriptors);
    
    // 注释掉的添加时间输出
    //printf("add feature time: %f", t_add.toc());
    
    // ret[0]是最近邻的分数，阈值会根据邻居分数变化
    bool find_loop = false;  // 回环检测标志
    cv::Mat loop_result;     // 回环结果可视化图像
    
    // 如果启用了调试图像，准备回环结果可视化
    if (DEBUG_IMAGE)
    {
        loop_result = compressed_image.clone();  // 克隆压缩图像作为基础
        
        // 如果有查询结果，在图像上显示最近邻的分数
        if (ret.size() > 0)
            putText(loop_result, "neighbour score:" + to_string(ret[0].Score), 
                    cv::Point2f(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255));
    }
    
    // 可视化回环结果（调试用）
    if (DEBUG_IMAGE)
    {
        // 遍历所有查询结果
        for (unsigned int i = 0; i < ret.size(); i++)
        {
            int tmp_index = ret[i].Id;  // 候选关键帧索引
            
            // 在图像池中查找对应的图像
            auto it = image_pool.find(tmp_index);
            cv::Mat tmp_image = (it->second).clone();  // 克隆候选关键帧图像
            
            // 在候选图像上添加索引和分数信息
            putText(tmp_image, "index:  " + to_string(tmp_index) + "loop score:" + to_string(ret[i].Score), 
                    cv::Point2f(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255));
            
            // 将当前图像和候选图像水平拼接
            cv::hconcat(loop_result, tmp_image, loop_result);
        }
    }
    
    // 判断是否找到好的匹配：要求至少有一个结果且最佳匹配分数大于0.05
    if (ret.size() >= 1 && ret[0].Score > 0.05)
        // 遍历其他候选结果（从第1个开始，第0个是最佳匹配）
        for (unsigned int i = 1; i < ret.size(); i++)
        {
            // 候选结果的分数需要大于0.015才被认为是有效的回环候选
            // 注释掉的相对阈值判断：if (ret[i].Score > ret[0].Score * 0.3)
            if (ret[i].Score > 0.015)
            {
                find_loop = true;  // 标记找到回环
                int tmp_index = ret[i].Id;  // 记录候选索引
                
                // 调试图像处理（条件为0，实际不执行）
                if (DEBUG_IMAGE && 0)
                {
                    auto it = image_pool.find(tmp_index);
                    cv::Mat tmp_image = (it->second).clone();
                    putText(tmp_image, "loop score:" + to_string(ret[i].Score), 
                            cv::Point2f(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255));
                    cv::hconcat(loop_result, tmp_image, loop_result);
                }
            }
        }
    
    /*
    // 注释掉的图像显示代码
    if (DEBUG_IMAGE)
    {
        cv::imshow("loop_result", loop_result);  // 显示回环结果图像
        cv::waitKey(20);  // 等待20毫秒
    }
    */
    
    // 如果找到回环且当前帧索引大于50（避免在轨迹开始时检测回环）
    if (find_loop && frame_index > 50)
    {
        int min_index = -1;  // 最小索引初始化为-1
        
        // 在所有满足条件的候选结果中寻找索引最小的那个
        for (unsigned int i = 0; i < ret.size(); i++)
        {
            // 选择索引最小且分数大于0.015的候选
            if (min_index == -1 || (ret[i].Id < min_index && ret[i].Score > 0.015))
                min_index = ret[i].Id;
        }
        return min_index;  // 返回最小索引的回环关键帧
    }
    else
        return -1;  // 未找到回环，返回-1
}
// 将关键帧添加到词袋数据库中的函数
// 参数：keyframe - 要添加的关键帧指针
void PoseGraph::addKeyFrameIntoVoc(KeyFrame* keyframe)
{
    // 定义压缩图像变量，用于可视化
    cv::Mat compressed_image;
    
    // 如果启用了调试图像功能
    if (DEBUG_IMAGE)
    {
        // 获取关键帧的特征点数量
        int feature_num = keyframe->keypoints.size();
        
        // 将原图像缩放到固定尺寸(376x240)，减少内存占用
        cv::resize(keyframe->image, compressed_image, cv::Size(376, 240));
        
        // 在图像上添加特征点数量信息
        putText(compressed_image, "feature_num:" + to_string(feature_num), 
                cv::Point2f(10, 10), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255));
        
        // 将处理后的图像存入图像池，使用关键帧索引作为键
        image_pool[keyframe->index] = compressed_image;
    }

    // 将关键帧的BRIEF描述子添加到词袋数据库中
    db.add(keyframe->brief_descriptors);
}

// 4自由度位姿图优化线程函数（后台运行）
void PoseGraph::optimize4DoF()
{
    // 无限循环，持续处理优化任务
    while(true)
    {
        int cur_index = -1;           // 当前需要优化的关键帧索引
        int first_looped_index = -1;  // 最早的回环索引
        
        // 加锁获取优化缓冲区中的任务
        m_optimize_buf.lock();
        // 处理缓冲区中的所有优化请求
        while(!optimize_buf.empty())
        {
            cur_index = optimize_buf.front();        // 获取队列首部的关键帧索引
            first_looped_index = earliest_loop_index; // 获取最早回环索引
            optimize_buf.pop();                      // 弹出已处理的任务
        }
        m_optimize_buf.unlock();
        
        // 如果有优化任务需要处理
        if (cur_index != -1)
        {
            printf("optimize pose graph \n");  // 输出优化开始信息
            TicToc tmp_t;  // 计时器，用于测量优化耗时
            
            // 加锁保护关键帧列表
            m_keyframelist.lock();
            // 获取当前要优化的关键帧
            KeyFrame* cur_kf = getKeyFrame(cur_index);

            // 计算优化问题的最大长度（关键帧数量）
            int max_length = cur_index + 1;

            // 定义优化变量数组：
            // t_array: 位置数组 [max_length][3] (x,y,z)
            double t_array[max_length][3];
            // q_array: 四元数数组
            Quaterniond q_array[max_length];
            // euler_array: 欧拉角数组 [max_length][3] (yaw, pitch, roll)
            double euler_array[max_length][3];
            // sequence_array: 序列ID数组
            double sequence_array[max_length];

            // 创建Ceres优化问题
            ceres::Problem problem;
            // 配置优化器选项
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;  // 使用稀疏Cholesky分解
            //options.minimizer_progress_to_stdout = true;  // 注释掉的进度输出
            //options.max_solver_time_in_seconds = SOLVER_TIME * 3;  // 注释掉的求解时间限制
            options.max_num_iterations = 5;  // 最大迭代次数为5次（保证实时性）
            ceres::Solver::Summary summary;  // 优化结果摘要
            
            // 定义损失函数，用于处理异常值
            ceres::LossFunction *loss_function;
            loss_function = new ceres::HuberLoss(0.1);  // Huber损失函数，参数0.1
            //loss_function = new ceres::CauchyLoss(1.0);  // 注释掉的Cauchy损失函数
            
            // 创建角度局部参数化（处理角度的周期性）
            ceres::LocalParameterization* angle_local_parameterization =
                AngleLocalParameterization::Create();

            // 关键帧列表迭代器
            list<KeyFrame*>::iterator it;

            int i = 0;  // 局部索引计数器
            
            // 遍历关键帧列表，构建优化问题
            for (it = keyframelist.begin(); it != keyframelist.end(); it++)
            {
                // 跳过最早回环索引之前的关键帧（这些帧位姿固定）
                if ((*it)->index < first_looped_index)
                    continue;
                    
                // 设置关键帧的局部索引（在优化问题中的索引）
                (*it)->local_index = i;
                
                // 获取关键帧的VIO位姿
                Quaterniond tmp_q;
                Matrix3d tmp_r;
                Vector3d tmp_t;
                (*it)->getVioPose(tmp_t, tmp_r);
                tmp_q = tmp_r;  // 旋转矩阵转四元数
                
                // 将位姿数据存入数组
                t_array[i][0] = tmp_t(0);  // x坐标
                t_array[i][1] = tmp_t(1);  // y坐标  
                t_array[i][2] = tmp_t(2);  // z坐标
                q_array[i] = tmp_q;        // 四元数

                // 将旋转转换为欧拉角（yaw, pitch, roll）
                Vector3d euler_angle = Utility::R2ypr(tmp_q.toRotationMatrix());
                euler_array[i][0] = euler_angle.x();  // yaw
                euler_array[i][1] = euler_angle.y();  // pitch
                euler_array[i][2] = euler_angle.z();  // roll

                // 存储序列ID
                sequence_array[i] = (*it)->sequence;

                // 向优化问题添加参数块：
                // 欧拉角（只有yaw是优化变量，pitch和roll固定）
                problem.AddParameterBlock(euler_array[i], 1, angle_local_parameterization);
                // 位置（x,y,z）
                problem.AddParameterBlock(t_array[i], 3);

                // 如果是第一个回环关键帧或基础序列（序列0），固定其位姿
                if ((*it)->index == first_looped_index || (*it)->sequence == 0)
                {
                    problem.SetParameterBlockConstant(euler_array[i]);  // 固定欧拉角
                    problem.SetParameterBlockConstant(t_array[i]);      // 固定位置
                }

                // 添加相邻关键帧之间的边（局部约束）
                for (int j = 1; j < 5; j++)  // 连接前4个相邻关键帧
                {
                  // 检查索引有效性和序列一致性
                  if (i - j >= 0 && sequence_array[i] == sequence_array[i-j])
                  {
                    // 获取相邻关键帧的欧拉角
                    Vector3d euler_conncected = Utility::R2ypr(q_array[i-j].toRotationMatrix());
                    // 计算相对平移
                    Vector3d relative_t(t_array[i][0] - t_array[i-j][0], 
                                       t_array[i][1] - t_array[i-j][1], 
                                       t_array[i][2] - t_array[i-j][2]);
                    // 将相对平转到相邻关键帧的坐标系下
                    relative_t = q_array[i-j].inverse() * relative_t;
                    // 计算相对偏航角
                    double relative_yaw = euler_array[i][0] - euler_array[i-j][0];
                    
                    // 创建4自由度误差项
                    ceres::CostFunction* cost_function = FourDOFError::Create(
                        relative_t.x(), relative_t.y(), relative_t.z(),  // 相对平移
                        relative_yaw,                                    // 相对偏航角
                        euler_conncected.y(), euler_conncected.z());     // 固定的pitch和roll
                    
                    // 向问题添加残差块
                    problem.AddResidualBlock(cost_function, NULL, 
                        euler_array[i-j],  // 相邻关键帧的yaw
                        t_array[i-j],      // 相邻关键帧的位置
                        euler_array[i],    // 当前关键帧的yaw  
                        t_array[i]);       // 当前关键帧的位置
                  }
                }

                // 添加回环边（全局约束）
                if((*it)->has_loop)  // 如果当前关键帧有回环
                {
                    // 断言：回环索引必须大于等于最早回环索引
                    assert((*it)->loop_index >= first_looped_index);
                    // 获取回环关键帧的局部索引
                    int connected_index = getKeyFrame((*it)->loop_index)->local_index;
                    // 获取回环关键帧的欧拉角
                    Vector3d euler_conncected = Utility::R2ypr(q_array[connected_index].toRotationMatrix());
                    // 获取回环相对平移
                    Vector3d relative_t;
                    relative_t = (*it)->getLoopRelativeT();
                    // 获取回环相对偏航角
                    double relative_yaw = (*it)->getLoopRelativeYaw();
                    
                    // 创建带权重的4自由度误差项
                    ceres::CostFunction* cost_function = FourDOFWeightError::Create(
                        relative_t.x(), relative_t.y(), relative_t.z(),  // 相对平移
                        relative_yaw,                                    // 相对偏航角
                        euler_conncected.y(), euler_conncected.z());     // 固定的pitch和roll
                    
                    // 向问题添加残差块（使用损失函数处理异常值）
                    problem.AddResidualBlock(cost_function, loss_function, 
                        euler_array[connected_index],  // 回环关键帧的yaw
                        t_array[connected_index],      // 回环关键帧的位置
                        euler_array[i],                // 当前关键帧的yaw
                        t_array[i]);                   // 当前关键帧的位置
                }

                // 如果到达当前关键帧，提前结束遍历
                if ((*it)->index == cur_index)
                    break;
                i++;  // 递增局部索引
            }
            m_keyframelist.unlock();  // 解锁关键帧列表

            // 执行优化
            ceres::Solve(options, &problem, &summary);
            // 注释掉的优化报告输出
            //std::cout << summary.BriefReport() << "\n";
            //printf("pose optimization time: %f \n", tmp_t.toc());
            
            /*
            // 注释掉的优化结果调试输出
            for (int j = 0 ; j < i; j++)
            {
                printf("optimize i: %d p: %f, %f, %f\n", j, t_array[j][0], t_array[j][1], t_array[j][2] );
            }
            */
            
            // 加锁保护关键帧列表，更新优化后的位姿
            m_keyframelist.lock();
            i = 0;  // 重置局部索引
            for (it = keyframelist.begin(); it != keyframelist.end(); it++)
            {
                // 跳过最早回环索引之前的关键帧
                if ((*it)->index < first_looped_index)
                    continue;
                    
                // 将优化后的欧拉角转换回四元数
                Quaterniond tmp_q;
                tmp_q = Utility::ypr2R(Vector3d(euler_array[i][0], euler_array[i][1], euler_array[i][2]));
                // 获取优化后的位置
                Vector3d tmp_t = Vector3d(t_array[i][0], t_array[i][1], t_array[i][2]);
                // 四元数转旋转矩阵
                Matrix3d tmp_r = tmp_q.toRotationMatrix();
                // 更新关键帧的优化后位姿
                (*it)->updatePose(tmp_t, tmp_r);

                // 如果到达当前关键帧，提前结束遍历
                if ((*it)->index == cur_index)
                    break;
                i++;  // 递增局部索引
            }

            // 计算漂移量（优化后位姿与VIO位姿之间的差异）
            Vector3d cur_t, vio_t;
            Matrix3d cur_r, vio_r;
            cur_kf->getPose(cur_t, cur_r);    // 获取优化后位姿
            cur_kf->getVioPose(vio_t, vio_r); // 获取VIO位姿
            
            // 加锁保护漂移量数据
            m_drift.lock();
            // 计算偏航角漂移
            yaw_drift = Utility::R2ypr(cur_r).x() - Utility::R2ypr(vio_r).x();
            // 根据偏航角漂移创建旋转漂移矩阵（只考虑yaw）
            r_drift = Utility::ypr2R(Vector3d(yaw_drift, 0, 0));
            // 计算平移漂移
            t_drift = cur_t - r_drift * vio_t;
            m_drift.unlock();
            
            // 注释掉的漂移量输出
            //cout << "t_drift " << t_drift.transpose() << endl;
            //cout << "r_drift " << Utility::R2ypr(r_drift).transpose() << endl;
            //cout << "yaw drift " << yaw_drift << endl;

            // 对当前关键帧之后的所有关键帧应用漂移校正
            it++;  // 移动到下一个关键帧
            for (; it != keyframelist.end(); it++)
            {
                Vector3d P;
                Matrix3d R;
                (*it)->getVioPose(P, R);  // 获取VIO位姿
                // 应用漂移校正
                P = r_drift * P + t_drift;  // 校正位置
                R = r_drift * R;            // 校正旋转
                (*it)->updatePose(P, R);    // 更新位姿
            }
            m_keyframelist.unlock();  // 解锁关键帧列表
            
            // 更新可视化路径
            updatePath();
        }

        // 如果没有优化任务，线程休眠2秒
        std::chrono::milliseconds dura(2000);
        std::this_thread::sleep_for(dura);
    }
}
// 更新路径和可视化的函数
void PoseGraph::updatePath()
{
    // 加锁保护关键帧列表
    m_keyframelist.lock();
    
    // 定义关键帧列表迭代器
    list<KeyFrame*>::iterator it;
    
    // 清空所有序列的路径（除了序列0）
    for (int i = 1; i <= sequence_cnt; i++)
    {
        path[i].poses.clear();  // 清空第i个序列的路径点
    }
    
    // 清空基础路径（序列0的路径）
    base_path.poses.clear();
    
    // 重置位姿图可视化（清除所有可视化标记）
    posegraph_visualization->reset();

    // 如果设置了保存回环路径，清空结果文件（重新开始记录）
    if (SAVE_LOOP_PATH)
    {
        // 以输出模式打开文件，这会清空文件内容
        ofstream loop_path_file_tmp(VINS_RESULT_PATH, ios::out);
        loop_path_file_tmp.close();  // 立即关闭文件
    }

    // 遍历所有关键帧，构建路径和可视化
    for (it = keyframelist.begin(); it != keyframelist.end(); it++)
    {
        // 获取关键帧的优化后位姿
        Vector3d P;
        Matrix3d R;
        (*it)->getPose(P, R);  // 获取位置和旋转
        
        // 将旋转矩阵转换为四元数
        Quaterniond Q;
        Q = R;
        
        // 注释掉的调试输出
        //printf("path p: %f, %f, %f\n",  P.x(),  P.z(),  P.y() );

        // 创建ROS位姿消息
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp = rclcpp::Time((*it)->time_stamp);  // 设置时间戳
        pose_stamped.header.frame_id = "world";                       // 设置坐标系为世界坐标系
        
        // 设置位置（添加可视化偏移，避免重叠显示）
        pose_stamped.pose.position.x = P.x() + VISUALIZATION_SHIFT_X;
        pose_stamped.pose.position.y = P.y() + VISUALIZATION_SHIFT_Y;
        pose_stamped.pose.position.z = P.z();
        
        // 设置朝向（四元数）
        pose_stamped.pose.orientation.x = Q.x();
        pose_stamped.pose.orientation.y = Q.y();
        pose_stamped.pose.orientation.z = Q.z();
        pose_stamped.pose.orientation.w = Q.w();
        
        // 根据序列ID将位姿添加到对应的路径中
        if((*it)->sequence == 0)  // 基础序列（序列0）
        {
            base_path.poses.push_back(pose_stamped);      // 添加到基础路径
            base_path.header = pose_stamped.header;       // 更新路径头信息
        }
        else  // 其他序列
        {
            path[(*it)->sequence].poses.push_back(pose_stamped);  // 添加到对应序列路径
            path[(*it)->sequence].header = pose_stamped.header;   // 更新路径头信息
        }

        // 如果设置了保存回环路径，将位姿数据保存到文件
        if (SAVE_LOOP_PATH)
        {
            // 以追加模式打开文件
            ofstream loop_path_file(VINS_RESULT_PATH, ios::app);
            loop_path_file.setf(ios::fixed, ios::floatfield);  // 设置固定小数点格式
            loop_path_file.precision(0);                       // 时间戳精度为0（整数）
            loop_path_file << (*it)->time_stamp * 1e9 << ",";  // 写入时间戳（转换为纳秒）
            loop_path_file.precision(5);                       // 位姿精度为5位小数
            
            // 写入位姿数据：位置(x,y,z)和四元数(w,x,y,z)
            loop_path_file  << P.x() << ","
                  << P.y() << ","
                  << P.z() << ","
                  << Q.w() << ","
                  << Q.x() << ","
                  << Q.y() << ","
                  << Q.z() << ","
                  << endl;
            loop_path_file.close();  // 关闭文件
        }
        
        // 如果设置了显示短边（相邻关键帧之间的连接），绘制局部连接
        if (SHOW_S_EDGE)
        {
            // 从关键帧列表末尾开始反向遍历（最新的关键帧在末尾）
            list<KeyFrame*>::reverse_iterator rit = keyframelist.rbegin();
            list<KeyFrame*>::reverse_iterator lrit;  // 局部迭代器
            
            // 遍历到列表开头
            for (; rit != keyframelist.rend(); rit++)
            {
                // 找到当前关键帧在反向迭代器中的位置
                if ((*rit)->index == (*it)->index)
                {
                    lrit = rit;  // 记录当前位置
                    lrit++;      // 移动到下一个（时间上更早的）关键帧
                    
                    // 连接最近的4个关键帧
                    for (int i = 0; i < 4; i++)
                    {
                        if (lrit == keyframelist.rend())  // 检查是否到达列表开头
                            break;
                        
                        // 只连接同一序列的关键帧
                        if((*lrit)->sequence == (*it)->sequence)
                        {
                            Vector3d conncected_P;
                            Matrix3d connected_R;
                            (*lrit)->getPose(conncected_P, connected_R);  // 获取连接关键帧的位姿
                            // 添加边到可视化
                            posegraph_visualization->add_edge(P, conncected_P);
                        }
                        lrit++;  // 移动到前一个关键帧
                    }
                    break;  // 找到当前关键帧后跳出循环
                }
            }
        }
        
        // 如果设置了显示长边（回环连接），并且当前关键帧有回环，绘制回环边
        if (SHOW_L_EDGE)
        {
            // 检查：当前关键帧有回环且属于最新序列
            if ((*it)->has_loop && (*it)->sequence == sequence_cnt)
            {
                // 获取回环关键帧
                KeyFrame* connected_KF = getKeyFrame((*it)->loop_index);
                Vector3d connected_P;
                Matrix3d connected_R;
                connected_KF->getPose(connected_P, connected_R);  // 获取回环关键帧位姿
                
                // 重新获取当前关键帧位姿（确保使用最新位姿）
                //(*it)->getVioPose(P, R);  // 注释掉的VIO位姿获取
                (*it)->getPose(P, R);       // 使用优化后的位姿
                
                // 只对非基础序列（sequence > 0）绘制回环边
                if((*it)->sequence > 0)
                {
                    // 添加回环边到可视化（终点添加可视化偏移）
                    posegraph_visualization->add_loopedge(P, connected_P + Vector3d(VISUALIZATION_SHIFT_X, VISUALIZATION_SHIFT_Y, 0));
                }
            }
        }
    }
    
    // 发布所有可视化消息（路径、位姿图等）
    publish();
    
    // 解锁关键帧列表
    m_keyframelist.unlock();
}
// 保存位姿图到文件的函数
void PoseGraph::savePoseGraph()
{
    // 加锁保护关键帧列表
    m_keyframelist.lock();
    TicToc tmp_t;  // 计时器，用于测量保存耗时
    
    FILE *pFile;  // 文件指针
    printf("pose graph path: %s\n",POSE_GRAPH_SAVE_PATH.c_str());  // 输出保存路径
    printf("pose graph saving... \n");  // 输出保存开始信息
    
    // 构建主位姿图文件路径
    string file_path = POSE_GRAPH_SAVE_PATH + "pose_graph.txt";
    pFile = fopen (file_path.c_str(),"w");  // 以写入模式打开文件
    
    // 注释掉的文件头
    //fprintf(pFile, "index time_stamp Tx Ty Tz Qw Qx Qy Qz loop_index loop_info\n");
    
    // 遍历所有关键帧，保存数据
    list<KeyFrame*>::iterator it;
    for (it = keyframelist.begin(); it != keyframelist.end(); it++)
    {
        // 定义各种文件路径
        std::string image_path, descriptor_path, brief_path, keypoints_path;
        
        // 如果启用了调试图像，保存关键帧图像
        if (DEBUG_IMAGE)
        {
            image_path = POSE_GRAPH_SAVE_PATH + to_string((*it)->index) + "_image.png";
            imwrite(image_path.c_str(), (*it)->image);  // 保存图像到文件
        }
        
        // 获取VIO位姿和优化后的位姿（PG: Pose Graph）
        Quaterniond VIO_tmp_Q{(*it)->vio_R_w_i};  // VIO旋转（四元数）
        Quaterniond PG_tmp_Q{(*it)->R_w_i};       // 优化后旋转（四元数）
        Vector3d VIO_tmp_T = (*it)->vio_T_w_i;    // VIO平移
        Vector3d PG_tmp_T = (*it)->T_w_i;         // 优化后平移

        // 将关键帧数据写入主位姿图文件
        // 格式：索引 时间戳 VIO_Tx VIO_Ty VIO_Tz PG_Tx PG_Ty PG_Tz 
        //        VIO_Qw VIO_Qx VIO_Qy VIO_Qz PG_Qw PG_Qx PG_Qy PG_Qz
        //        回环索引 回环信息(8维) 特征点数量
        fprintf (pFile, " %d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %d %f %f %f %f %f %f %f %f %d\n",
                (*it)->index, (*it)->time_stamp,
                VIO_tmp_T.x(), VIO_tmp_T.y(), VIO_tmp_T.z(),     // VIO平移
                PG_tmp_T.x(), PG_tmp_T.y(), PG_tmp_T.z(),        // 优化后平移
                VIO_tmp_Q.w(), VIO_tmp_Q.x(), VIO_tmp_Q.y(), VIO_tmp_Q.z(),  // VIO旋转
                PG_tmp_Q.w(), PG_tmp_Q.x(), PG_tmp_Q.y(), PG_tmp_Q.z(),      // 优化后旋转
                (*it)->loop_index,  // 回环索引
                (*it)->loop_info(0), (*it)->loop_info(1), (*it)->loop_info(2), (*it)->loop_info(3),  // 回环信息前4维
                (*it)->loop_info(4), (*it)->loop_info(5), (*it)->loop_info(6), (*it)->loop_info(7),  // 回环信息后4维
                (int)(*it)->keypoints.size());  // 特征点数量

        // 保存特征点和描述子
        // 断言：特征点数量和描述子数量必须相等
        assert((*it)->keypoints.size() == (*it)->brief_descriptors.size());
        
        // 保存BRIEF描述子（二进制格式）
        brief_path = POSE_GRAPH_SAVE_PATH + to_string((*it)->index) + "_briefdes.dat";
        std::ofstream brief_file(brief_path, std::ios::binary);  // 二进制模式打开
        // 保存特征点坐标（像素坐标和归一化坐标）
        keypoints_path = POSE_GRAPH_SAVE_PATH + to_string((*it)->index) + "_keypoints.txt";
        FILE *keypoints_file;
        keypoints_file = fopen(keypoints_path.c_str(), "w");  // 文本模式打开
        
        // 遍历所有特征点
        for (int i = 0; i < (int)(*it)->keypoints.size(); i++)
        {
            brief_file << (*it)->brief_descriptors[i] << endl;  // 写入描述子（二进制）
            // 写入特征点坐标：像素坐标(x,y) 和 归一化坐标(x,y)
            fprintf(keypoints_file, "%f %f %f %f\n", 
                    (*it)->keypoints[i].pt.x, (*it)->keypoints[i].pt.y,
                    (*it)->keypoints_norm[i].pt.x, (*it)->keypoints_norm[i].pt.y);
        }
        brief_file.close();    // 关闭描述子文件
        fclose(keypoints_file); // 关闭特征点文件
    }
    fclose(pFile);  // 关闭主位姿图文件

    // 输出保存耗时
    printf("save pose graph time: %f s\n", tmp_t.toc() / 1000);
    m_keyframelist.unlock();  // 解锁关键帧列表
}

// 从文件加载位姿图的函数
void PoseGraph::loadPoseGraph()
{
    TicToc tmp_t;  // 计时器，用于测量加载耗时
    FILE * pFile;   // 文件指针
    
    // 构建主位姿图文件路径
    string file_path = POSE_GRAPH_SAVE_PATH + "pose_graph.txt";
    printf("lode pose graph from: %s \n", file_path.c_str());  // 输出加载路径
    printf("pose graph loading...\n");  // 输出加载开始信息
    
    pFile = fopen (file_path.c_str(),"r");  // 以读取模式打开文件
    if (pFile == NULL)  // 检查文件是否存在
    {
        printf("lode previous pose graph error: wrong previous pose graph path or no previous pose graph \n the system will start with new pose graph \n");
        return;  // 文件不存在，返回（系统将使用新的位姿图）
    }
    
    // 定义变量用于读取文件数据
    int index;                    // 关键帧索引
    double time_stamp;            // 时间戳
    double VIO_Tx, VIO_Ty, VIO_Tz; // VIO平移
    double PG_Tx, PG_Ty, PG_Tz;   // 优化后平移
    double VIO_Qw, VIO_Qx, VIO_Qy, VIO_Qz;  // VIO旋转四元数
    double PG_Qw, PG_Qx, PG_Qy, PG_Qz;      // 优化后旋转四元数
    double loop_info_0, loop_info_1, loop_info_2, loop_info_3;  // 回环信息前4维
    double loop_info_4, loop_info_5, loop_info_6, loop_info_7;  // 回环信息后4维
    int loop_index;               // 回环索引
    int keypoints_num;            // 特征点数量
    Eigen::Matrix<double, 8, 1 > loop_info;  // 回环信息向量
    int cnt = 0;  // 计数器
    
    // 循环读取文件，直到文件结尾
    while (fscanf(pFile,"%d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %lf %lf %lf %lf %lf %lf %lf %lf %d", 
                  &index, &time_stamp,
                  &VIO_Tx, &VIO_Ty, &VIO_Tz,     // VIO平移
                  &PG_Tx, &PG_Ty, &PG_Tz,        // 优化后平移
                  &VIO_Qw, &VIO_Qx, &VIO_Qy, &VIO_Qz,  // VIO旋转
                  &PG_Qw, &PG_Qx, &PG_Qy, &PG_Qz,      // 优化后旋转
                  &loop_index,                    // 回环索引
                  &loop_info_0, &loop_info_1, &loop_info_2, &loop_info_3,  // 回环信息
                  &loop_info_4, &loop_info_5, &loop_info_6, &loop_info_7,
                  &keypoints_num) != EOF)        // 特征点数量
    {
        /*
        // 注释掉的调试输出：显示读取的每一行数据
        printf("I read: %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %lf %lf %lf %lf %lf %lf %lf %lf %d\n", index, time_stamp,
                                    VIO_Tx, VIO_Ty, VIO_Tz,
                                    PG_Tx, PG_Ty, PG_Tz,
                                    VIO_Qw, VIO_Qx, VIO_Qy, VIO_Qz,
                                    PG_Qw, PG_Qx, PG_Qy, PG_Qz,
                                    loop_index,
                                    loop_info_0, loop_info_1, loop_info_2, loop_info_3,
                                    loop_info_4, loop_info_5, loop_info_6, loop_info_7,
                                    keypoints_num);
        */
        
        cv::Mat image;  // 图像变量
        std::string image_path, descriptor_path;
        
        // 如果启用了调试图像，加载关键帧图像
        if (DEBUG_IMAGE)
        {
            image_path = POSE_GRAPH_SAVE_PATH + to_string(index) + "_image.png";
            image = cv::imread(image_path.c_str(), 0);  // 以灰度模式读取图像
        }

        // 将从文件读取的数据转换为Eigen类型
        Vector3d VIO_T(VIO_Tx, VIO_Ty, VIO_Tz);  // VIO平移向量
        Vector3d PG_T(PG_Tx, PG_Ty, PG_Tz);      // 优化后平移向量
        
        Quaterniond VIO_Q;  // VIO旋转四元数
        VIO_Q.w() = VIO_Qw;
        VIO_Q.x() = VIO_Qx;
        VIO_Q.y() = VIO_Qy;
        VIO_Q.z() = VIO_Qz;
        
        Quaterniond PG_Q;   // 优化后旋转四元数
        PG_Q.w() = PG_Qw;
        PG_Q.x() = PG_Qx;
        PG_Q.y() = PG_Qy;
        PG_Q.z() = PG_Qz;
        
        Matrix3d VIO_R, PG_R;  // 旋转矩阵
        VIO_R = VIO_Q.toRotationMatrix();  // 四元数转旋转矩阵
        PG_R = PG_Q.toRotationMatrix();
        
        // 构建回环信息向量
        loop_info << loop_info_0, loop_info_1, loop_info_2, loop_info_3, 
                    loop_info_4, loop_info_5, loop_info_6, loop_info_7;

        // 更新最早回环索引
        if (loop_index != -1)
            if (earliest_loop_index > loop_index || earliest_loop_index == -1)
            {
                earliest_loop_index = loop_index;
            }

        // 加载特征点和描述子
        string brief_path = POSE_GRAPH_SAVE_PATH + to_string(index) + "_briefdes.dat";
        std::ifstream brief_file(brief_path, std::ios::binary);  // 二进制模式打开描述子文件
        string keypoints_path = POSE_GRAPH_SAVE_PATH + to_string(index) + "_keypoints.txt";
        FILE *keypoints_file;
        keypoints_file = fopen(keypoints_path.c_str(), "r");  // 文本模式打开特征点文件
        
        vector<cv::KeyPoint> keypoints;          // 特征点容器（像素坐标）
        vector<cv::KeyPoint> keypoints_norm;     // 特征点容器（归一化坐标）
        vector<BRIEF::bitset> brief_descriptors; // 描述子容器
        
        // 读取所有特征点和描述子
        for (int i = 0; i < keypoints_num; i++)
        {
            BRIEF::bitset tmp_des;
            brief_file >> tmp_des;  // 读取描述子（二进制）
            brief_descriptors.push_back(tmp_des);
            
            cv::KeyPoint tmp_keypoint;
            cv::KeyPoint tmp_keypoint_norm;
            double p_x, p_y, p_x_norm, p_y_norm;
            
            // 读取特征点坐标
            if(!fscanf(keypoints_file,"%lf %lf %lf %lf", &p_x, &p_y, &p_x_norm, &p_y_norm))
                printf(" fail to load pose graph \n");  // 读取失败提示
            
            tmp_keypoint.pt.x = p_x;        // 像素坐标x
            tmp_keypoint.pt.y = p_y;        // 像素坐标y
            tmp_keypoint_norm.pt.x = p_x_norm;  // 归一化坐标x
            tmp_keypoint_norm.pt.y = p_y_norm;  // 归一化坐标y
            
            keypoints.push_back(tmp_keypoint);
            keypoints_norm.push_back(tmp_keypoint_norm);
        }
        brief_file.close();      // 关闭描述子文件
        fclose(keypoints_file);  // 关闭特征点文件

        // 创建关键帧对象
        KeyFrame* keyframe = new KeyFrame(time_stamp, index, VIO_T, VIO_R, PG_T, PG_R, 
                                         image, loop_index, loop_info, keypoints, keypoints_norm, brief_descriptors);
        
        // 加载关键帧到位姿图中（不进行回环检测）
        loadKeyFrame(keyframe, 0);
        
        // 每加载20个关键帧发布一次可视化消息（避免过于频繁的发布）
        if (cnt % 20 == 0)
        {
            publish();
        }
        cnt++;  // 递增计数器
    }
    fclose (pFile);  // 关闭主位姿图文件
    
    // 输出加载耗时
    printf("load pose graph time: %f s\n", tmp_t.toc()/1000);
    base_sequence = 0;  // 设置基础序列为0
}
// 发布可视化消息的函数
void PoseGraph::publish()
{
    // 遍历所有序列（从1到sequence_cnt）
    for (int i = 1; i <= sequence_cnt; i++)
    {
        // 条件判断：总是发布所有序列的路径（条件 1 || i == base_sequence 总是为真）
        // 注释掉的原条件：if (sequence_loop[i] == true || i == base_sequence)
        if (1 || i == base_sequence)
        {
            // 发布位姿图路径到通用话题
            pub_pg_path->publish(path[i]);
            // 发布序列路径到专用话题（path_1, path_2, ...）
            pub_path[i]->publish(path[i]);
            // 发布位姿图可视化标记
            posegraph_visualization->publish_by(pub_pose_graph, path[sequence_cnt].header);
        }
    }
    
    // 设置基础路径的坐标系并发布
    base_path.header.frame_id = "world";
    pub_base_path->publish(base_path);
    
    // 注释掉的重复发布（上面已经在循环中发布了）
    //posegraph_visualization->publish_by(pub_pose_graph, path[sequence_cnt].header);
}

// 更新关键帧回环信息的函数
// 参数：index - 关键帧索引
//       _loop_info - 8维回环信息向量
void PoseGraph::updateKeyFrameLoop(int index, Eigen::Matrix<double, 8, 1 > &_loop_info)
{
    // 根据索引获取关键帧
    KeyFrame* kf = getKeyFrame(index);
    
    // 更新关键帧的回环信息
    kf->updateLoop(_loop_info);
    
    // 检查回环信息的可靠性条件：
    // 1. 回环信息第7个元素（可能是时间差或分数）绝对值小于30.0
    // 2. 回环相对平移向量的模长小于20.0（避免过大的相对运动）
    if (abs(_loop_info(7)) < 30.0 && Vector3d(_loop_info(0), _loop_info(1), _loop_info(2)).norm() < 20.0)
    {
        // 如果启用了快速重定位功能
        if (FAST_RELOCALIZATION)
        {
            // 获取回环关键帧
            KeyFrame* old_kf = getKeyFrame(kf->loop_index);
            
            // 定义变量存储位姿信息
            Vector3d w_P_old, w_P_cur, vio_P_cur;
            Matrix3d w_R_old, w_R_cur, vio_R_cur;
            
            // 获取回环关键帧的优化后位姿（世界坐标系）
            old_kf->getPose(w_P_old, w_R_old);
            // 获取当前关键帧的VIO位姿
            kf->getVioPose(vio_P_cur, vio_R_cur);

            // 获取通过特征匹配计算出的相对位姿变换
            Vector3d relative_t;
            Quaterniond relative_q;
            relative_t = kf->getLoopRelativeT();        // 相对平移
            relative_q = (kf->getLoopRelativeQ()).toRotationMatrix();  // 相对旋转（转换为旋转矩阵）
            
            // 根据回环关键帧的位姿和相对变换，计算当前关键帧在世界坐标系下的新位姿
            w_P_cur = w_R_old * relative_t + w_P_old;  // 计算新的位置
            w_R_cur = w_R_old * relative_q;            // 计算新的旋转

            // 计算VIO位姿与回环调整后位姿之间的漂移量
            double shift_yaw;
            Matrix3d shift_r;
            Vector3d shift_t;
            
            // 计算偏航角（yaw）的漂移
            shift_yaw = Utility::R2ypr(w_R_cur).x() - Utility::R2ypr(vio_R_cur).x();
            // 根据偏航角漂移创建旋转矩阵（只考虑yaw，固定pitch和roll）
            shift_r = Utility::ypr2R(Vector3d(shift_yaw, 0, 0));
            // 计算平移漂移
            shift_t = w_P_cur - w_R_cur * vio_R_cur.transpose() * vio_P_cur;

            // 加锁保护漂移量数据
            m_drift.lock();
            // 更新漂移量
            yaw_drift = shift_yaw;   // 偏航角漂移
            r_drift = shift_r;       // 旋转漂移
            t_drift = shift_t;       // 平移漂移
            m_drift.unlock();
        }
    }
}