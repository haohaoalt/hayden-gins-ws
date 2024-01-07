//
// Created by meng on 2021/2/24.
//
#ifndef GPS_IMU_FUSION_ESKF_FLOW_H
#define GPS_IMU_FUSION_ESKF_FLOW_H

#include "eskf.h"
#include "imu_tool.h"
#include "gps_tool.h"
#include "config_parameters.h"
#include "observability_analysis.h"

#include <memory>
#include <deque>
#include <iostream>

class ESKFFlow {
public:
    // 不能在没有提供参数的情况下创建该类的实例
    ESKFFlow() = delete;
    // 显式构造函数
    explicit ESKFFlow(const std::string &config_file_path, std::string data_file_path);

    /*!
     * 从本地文件中读取IMU和GPS的数据
     * @return
     */
    void ReadData();

    /*!
     * 对IMU和GPS数据进行时间戳对齐，该函数只在ESKF初始化时使用
     * @return
     */
    bool ValidGPSAndIMUData();

    bool Run();

    // bool TestRun();

    /*!
     * 保存位姿，为kitti格式 静态函数
     * @param ofs
     * @param pose
     */
    static void SavePose(std::ofstream &ofs, const Eigen::Matrix4d &pose);

    /*!
     * Save TUM pose 静态函数
     *
     * note:
     * timestamp x y z q_x q_y q_z q_w
     *
     * @param ofs
     * @param pose
     */
    static void SaveTUMPose(std::ofstream &ofs, const Eigen::Quaterniond &q,
                            const Eigen::Vector3d &t, double timestamp);

private:
    ConfigParameters config_parameters_;

    // 指向 ErrorStateKalmanFilter 对象的共享指针。
    std::shared_ptr<ErrorStateKalmanFilter> eskf_ptr_;
    // 指向 IMUTool 对象的共享指针。
    std::shared_ptr<IMUTool> imu_flow_ptr_;
    // 指向 GPSTool 对象的共享指针。
    std::shared_ptr<GPSTool> gps_flow_ptr_;
    // 类的一个实例，可能用于可观测性分析。
    ObservabilityAnalysis observability_analysis;//可观测度分析工具
    // 用于存储IMU数据的双端队列。
    std::deque<IMUData> imu_data_buff_;
    // 用于存储GPS数据的双端队列。
    std::deque<GPSData> gps_data_buff_;

    IMUData curr_imu_data_;
    GPSData curr_gps_data_;

    bool use_observability_analysis_ = false;//是否进行可观测度分析

    const std::string config_file_path_;
    const std::string data_file_path_;
};

#endif //GPS_IMU_FUSION_ESKF_FLOW_H
