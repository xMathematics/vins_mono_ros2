#include "feature_tracker.h"
// 静态成员变量初始化：特征点ID计数器，从0开始
int FeatureTracker::n_id = 0;

/**
 * 检查特征点是否在图像有效边界内
 * @param pt 特征点坐标
 * @return 如果在边界内返回true，否则false
 */
bool inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;  // 边界大小，避免使用图像边缘像素
    int img_x = cvRound(pt.x);  // 四舍五入取整x坐标
    int img_y = cvRound(pt.y);  // 四舍五入取整y坐标
    // 检查点是否在有效区域内（避开图像边缘）
    return BORDER_SIZE <= img_x && img_x < COL - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ROW - BORDER_SIZE;
}

/**
 * 根据状态向量缩减点向量（光流跟踪后使用）
 * @param v 要缩减的点向量（输入输出参数）
 * @param status 状态向量，1表示保留，0表示删除
 */
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    int j = 0;  // 新向量的索引
    // 遍历所有元素
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])          // 如果状态为1（保留）
            v[j++] = v[i];      // 复制到新位置
    v.resize(j);  // 调整向量大小为有效元素数量
}

/**
 * 根据状态向量缩减整型向量
 * @param v 要缩减的整型向量（输入输出参数）
 * @param status 状态向量，1表示保留，0表示删除
 */
void reduceVector(vector<int> &v, vector<uchar> status)
{
    int j = 0;  // 新向量的索引
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])          // 如果状态为1（保留）
            v[j++] = v[i];      // 复制到新位置
    v.resize(j);  // 调整向量大小
}

// 特征跟踪器构造函数
FeatureTracker::FeatureTracker()
{
    // 构造函数体为空，成员变量会在其他地方初始化
}

/**
 * 设置掩膜：避免在已有特征点附近提取新特征点
 * 优先保留跟踪时间长的特征点
 */

//掩膜机制
// 目的: 保证特征点均匀分布
// 方法: 在已有特征点周围MIN_DIST半径内画黑圆
// 效果: goodFeaturesToTrack不会在这些区域提取新特征点
//特征点优先级 
// 按track_cnt降序排序 → 优先保留跟踪时间长的特征点
// 稳定性: 长时间跟踪的特征点更可靠
// 效率: 减少特征点ID分配和管理的开销
void FeatureTracker::setMask()
{
    // 初始化掩膜：如果是鱼眼相机使用鱼眼掩膜，否则创建全白掩膜
    if(FISHEYE)
        mask = fisheye_mask.clone();  // 克隆鱼眼掩膜
    else
        // 创建全白掩膜，尺寸为ROW×COL，8位无符号单通道
        mask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));
    
    // 创建临时容器：存储(跟踪计数, (特征点坐标, 特征点ID))的三元组
    // 这样可以根据跟踪计数排序
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

    // 遍历所有当前特征点，构建三元组
    for (unsigned int i = 0; i < forw_pts.size(); i++)
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(forw_pts[i], ids[i])));

    // 按跟踪计数从大到小排序（lambda表达式作为比较函数）
    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
         {
            return a.first > b.first;  // 按跟踪计数降序排列
         });

    // 清空原有数据，准备重新填充
    forw_pts.clear();
    ids.clear();
    track_cnt.clear();

    // 遍历排序后的特征点
    for (auto &it : cnt_pts_id)
    {
        // 检查该特征点位置在掩膜中是否可用（255表示可用）
        if (mask.at<uchar>(it.second.first) == 255)
        {
            // 添加特征点到新向量
            forw_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);
            // 在掩膜上以特征点为中心画黑圆，禁止在该区域提取新特征点
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
        }
    }
}

/**
 * 添加新检测到的特征点
 * 新特征点的ID设为-1，跟踪计数设为1
 */
void FeatureTracker::addPoints()
{
    for (auto &p : n_pts)  // 遍历所有新检测的特征点
    {
        forw_pts.push_back(p);    // 添加到当前特征点列表
        ids.push_back(-1);        // ID设为-1（未分配正式ID）
        track_cnt.push_back(1);   // 跟踪计数初始化为1
    }
}

/**
 * 主要图像处理函数：读取新图像并进行特征跟踪
 * @param _img 输入图像
 * @param _cur_time 当前时间戳
 */

/*光流跟踪流程
 cur_img (上一帧) → forw_img (当前帧)
 cur_pts → forw_pts (通过LK光流)
*/
// 特征点生命周期管理
// 新特征点: ids = -1, track_cnt = 1
// 持续跟踪: track_cnt++, ids保持不变  
// 跟踪失败: 从所有向量中移除
void FeatureTracker::readImage(const cv::Mat &_img, double _cur_time)
{
    cv::Mat img;        // 处理后的图像
    TicToc t_r;         // 计时器，测量整个函数执行时间
    cur_time = _cur_time;  // 更新当前时间戳

    // 图像预处理：如果启用直方图均衡化
    if (EQUALIZE)
    {
        // 创建CLAHE（对比度受限的自适应直方图均衡化）对象
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        TicToc t_c;
        clahe->apply(_img, img);  // 应用CLAHE
        RCUTILS_LOG_DEBUG("CLAHE costs: %fms", t_c.toc());
    }
    else
        img = _img;  // 直接使用原图

    // 初始化：如果是第一帧图像
    if (forw_img.empty())
    {
        prev_img = cur_img = forw_img = img;  // 所有图像帧都设为当前图像
    }
    else
    {
        forw_img = img;  // 更新向前传播帧
    }

    forw_pts.clear();  // 清空当前帧特征点

    // 如果存在上一帧的特征点，进行光流跟踪
    if (cur_pts.size() > 0)
    {
        TicToc t_o;        // 光流跟踪计时器
        vector<uchar> status;  // 跟踪状态（1成功，0失败）
        vector<float> err;     // 跟踪误差
        
        // LK光流跟踪：从cur_img到forw_img跟踪cur_pts
        cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21), 3);

        // 检查跟踪到的点是否在图像边界内
        for (int i = 0; i < int(forw_pts.size()); i++)
            if (status[i] && !inBorder(forw_pts[i]))  // 跟踪成功但超出边界
                status[i] = 0;  // 标记为失败
        
        // 根据跟踪状态缩减所有相关向量
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(ids, status);
        reduceVector(cur_un_pts, status);
        reduceVector(track_cnt, status);
        
        RCUTILS_LOG_DEBUG("temporal optical flow costs: %fms", t_o.toc());
    }

    // 所有特征点的跟踪计数加1
    for (auto &n : track_cnt)
        n++;

    // 如果发布当前帧（控制发布频率）
    if (PUB_THIS_FRAME)
    {
        // 使用基础矩阵剔除误匹配
        rejectWithF();
        
        RCUTILS_LOG_DEBUG("set mask begins");
        TicToc t_m;
        setMask();  // 设置特征提取掩膜
        RCUTILS_LOG_DEBUG("set mask costs %fms", t_m.toc());

        RCUTILS_LOG_DEBUG("detect feature begins");
        TicToc t_t;
        // 计算需要补充的特征点数量
        int n_max_cnt = MAX_CNT - static_cast<int>(forw_pts.size());
        if (n_max_cnt > 0)  // 如果需要补充特征点
        {
            // 掩膜检查（调试用）
            if(mask.empty())
                RCUTILS_LOG_INFO("mask is empty ");
            if (mask.type() != CV_8UC1)
                RCUTILS_LOG_INFO("mask type wrong ");
            if (mask.size() != forw_img.size())
                RCUTILS_LOG_INFO("wrong size ");
            
            // 使用Shi-Tomasi角点检测补充新特征点
            cv::goodFeaturesToTrack(forw_img, n_pts, MAX_CNT - forw_pts.size(), 0.01, MIN_DIST, mask);
        }
        else
            n_pts.clear();  // 不需要补充，清空新特征点
        RCUTILS_LOG_DEBUG("detect feature costs: %fms", t_t.toc());

        RCUTILS_LOG_DEBUG("add feature begins");
        TicToc t_a;
        addPoints();  // 将新特征点添加到跟踪列表
        RCUTILS_LOG_DEBUG("selectFeature costs: %fms", t_a.toc());
    }

    //状态更新流程
    /*
    时间: prev_time ← cur_time
    图像: prev_img ← cur_img ← forw_img
    特征点: prev_pts ← cur_pts ← forw_pts
    */
    
    // 更新帧状态：当前帧变为上一帧
    prev_img = cur_img;
    prev_pts = cur_pts;
    prev_un_pts = cur_un_pts;
    
    // 向前传播帧变为当前帧
    cur_img = forw_img;
    cur_pts = forw_pts;
    
    // 对当前特征点进行去畸变处理
    undistortedPoints();
    
    // 更新时间戳
    prev_time = cur_time;
}

/**
 * 使用基础矩阵F和RANSAC方法剔除误匹配的特征点
 * 基于对极几何约束，剔除不符合相机运动模型的误匹配
 */
// 对极几何约束：x2' * F * x1 = 0
// 其中x1, x2是两帧中的匹配点，F是基础矩阵
// RANSAC过程：
// 1. 随机采样8个点计算F矩阵
// 2. 计算所有点对到对极线的距离
// 3. 统计内点（距离 < F_THRESHOLD）
// 4. 重复多次，选择内点最多的F矩阵

/*去畸变处理流程
像素坐标 (cur_pts) 
    → liftProjective() 去畸变 
    → 相机坐标系3D点 (b)
    → 归一化平面坐标 (b.x()/b.z(), b.y()/b.z())
    → 存储到cur_un_pts和cur_un_pts_map
*/
void FeatureTracker::rejectWithF()
{
    // 至少需要8个点才能计算基础矩阵
    if (forw_pts.size() >= 8)
    {
        RCUTILS_LOG_DEBUG("FM ransac begins");
        TicToc t_f;  // 计时器，测量RANSAC计算时间
        
        // 创建临时向量存储去畸变后的特征点坐标
        vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_forw_pts(forw_pts.size());
        
        // 对当前帧和前一帧的特征点进行去畸变处理
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            Eigen::Vector3d tmp_p;  // 临时存储3D点
            
            // 对前一帧特征点去畸变：将像素坐标转换到归一化平面
            m_camera->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
            // 重新投影到虚拟的理想像素平面（用于基础矩阵计算）
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            // 对当前帧特征点去畸变
            m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
            // 重新投影到虚拟的理想像素平面
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        vector<uchar> status;  // RANSAC状态向量，1表示内点，0表示外点
        // 使用RANSAC方法计算基础矩阵并剔除外点
        cv::findFundamentalMat(un_cur_pts, un_forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        
        int size_a = cur_pts.size();  // 记录剔除前的特征点数量
        // 根据RANSAC结果缩减所有相关向量（剔除外点）
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(cur_un_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        
        // 调试信息：显示剔除比例（注释状态）
        // RCUTILS_LOG_INFO("FM ransac: %d -> %lu: %f", size_a, forw_pts.size(), 1.0 * forw_pts.size() / size_a);
        // RCUTILS_LOG_INFO("FM ransac costs: %fms", t_f.toc());
    }
}

/**
 * 更新特征点ID：为新特征点分配唯一ID
 * @param i 特征点索引
 * @return 如果成功更新返回true，索引越界返回false
 */
// ID管理机制
// 新特征点: ids[i] = -1
// 分配ID: ids[i] = n_id++ (递增分配)
// 跨帧跟踪: 通过ids[i]保持特征点身份一致
bool FeatureTracker::updateID(unsigned int i)
{
    if (i < ids.size())  // 检查索引有效性
    {
        if (ids[i] == -1)  // 如果特征点还没有分配ID
            ids[i] = n_id++;  // 分配新ID并递增计数器
        return true;
    }
    else
        return false;  // 索引越界
}

/**
 * 读取相机内参和畸变参数
 * @param calib_file 相机标定文件路径（YAML格式）
 */
void FeatureTracker::readIntrinsicParameter(const string &calib_file)
{
    RCUTILS_LOG_INFO("reading paramerter of camera %s", calib_file.c_str());
    // 使用相机工厂从YAML文件生成相机模型
    m_camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
}

/**
 * 显示去畸变效果：用于调试和验证相机参数
 * 将原始图像的去畸变版本可视化显示
 * @param name 显示窗口名称
 */
// m_camera->liftProjective() 是相机模型的核心接口
// 支持多种相机模型：
// - PinholeCamera: 针孔模型
// - EquidistantCamera: 等距模型  
// - RadtanCamera: 径向切向畸变模型
// - FovCamera: 视野模型
void FeatureTracker::showUndistortion(const string &name)
{
    // 创建去畸变图像，尺寸比原图大（600像素边距）
    cv::Mat undistortedImg(ROW + 600, COL + 600, CV_8UC1, cv::Scalar(0));
    vector<Eigen::Vector2d> distortedp, undistortedp;  // 存储原始点和去畸变点
    
    // 遍历图像中每个像素
    for (int i = 0; i < COL; i++)
        for (int j = 0; j < ROW; j++)
        {
            Eigen::Vector2d a(i, j);  // 原始像素坐标
            Eigen::Vector3d b;        // 去畸变后的归一化坐标
            // 对每个像素进行去畸变
            m_camera->liftProjective(a, b);
            distortedp.push_back(a);  // 保存原始坐标
            // 保存去畸变后的归一化坐标（除以z得到归一化平面坐标）
            undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
        }
    
    // 将去畸变点重新投影到图像平面并复制像素值
    for (int i = 0; i < int(undistortedp.size()); i++)
    {
        cv::Mat pp(3, 1, CV_32FC1);  // 齐次坐标
        // 将归一化坐标转换回像素坐标（使用焦距和图像中心）
        pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + COL / 2;
        pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + ROW / 2;
        pp.at<float>(2, 0) = 1.0;  // 齐次坐标的z分量
        
        // 检查重新投影后的坐标是否在显示图像范围内
        if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < ROW + 600 && 
            pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < COL + 600)
        {
            // 将原始图像的像素值复制到去畸变图像的对应位置
            undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = 
                cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
        }
    }
    // 显示去畸变图像并等待按键
    cv::imshow(name, undistortedImg);
    cv::waitKey(0);
}

/**
 * 对当前帧特征点进行去畸变处理，并计算特征点速度
 * 将像素坐标转换到归一化平面，为后续VIO优化做准备
 */

// 在归一化平面上计算特征点移动速度
// 速度 = (当前帧坐标 - 上一帧坐标) / 时间间隔
// 用途：
// - VIO初始化时提供视觉测量
// - 判断特征点跟踪质量
// - 辅助运动估计
void FeatureTracker::undistortedPoints()
{
    // 清空上一帧的去畸变点
    cur_un_pts.clear();
    cur_un_pts_map.clear();

    // 遍历所有当前特征点进行去畸变
    for (unsigned int i = 0; i < cur_pts.size(); i++)
    {
        Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);  // 像素坐标
        Eigen::Vector3d b;  // 去畸变后的3D点（在相机坐标系，z=1的归一化平面）
        // 调用相机模型进行去畸变
        m_camera->liftProjective(a, b);
        // 将3D点投影到归一化平面（除以z），保存为2D点
        cur_un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
        // 同时保存到映射表中，以特征点ID为键
        cur_un_pts_map.insert(make_pair(ids[i], cv::Point2f(b.x() / b.z(), b.y() / b.z())));
    }
    
    // 计算特征点速度（在归一化平面上的移动速度）
    if (!prev_un_pts_map.empty())  // 确保有上一帧数据
    {
        double dt = cur_time - prev_time;  // 时间间隔
        pts_velocity.clear();  // 清空速度向量
        
        for (unsigned int i = 0; i < cur_un_pts.size(); i++)
        {
            if (ids[i] != -1)  // 只处理已分配ID的特征点（有效特征点）
            {
                // 在上一帧的映射表中查找相同ID的特征点
                std::map<int, cv::Point2f>::iterator it;
                it = prev_un_pts_map.find(ids[i]);
                if (it != prev_un_pts_map.end())  // 如果找到匹配的特征点
                {
                    // 计算在归一化平面上的速度（像素/秒）
                    double v_x = (cur_un_pts[i].x - it->second.x) / dt;
                    double v_y = (cur_un_pts[i].y - it->second.y) / dt;
                    pts_velocity.push_back(cv::Point2f(v_x, v_y));
                }
                else
                    // 没有找到匹配点，速度设为0
                    pts_velocity.push_back(cv::Point2f(0, 0));
            }
            else
            {
                // 无效特征点（新特征点），速度设为0
                pts_velocity.push_back(cv::Point2f(0, 0));
            }
        }
    }
    else  // 第一帧或没有上一帧数据
    {
        // 所有特征点速度初始化为0
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            pts_velocity.push_back(cv::Point2f(0, 0));
        }
    }
    
    // 更新上一帧的映射表为当前帧，为下一帧计算速度做准备
    prev_un_pts_map = cur_un_pts_map;
}