/*V1.8
新增板外控制，
支持方格例程,
新增日志记录，
新增网络传输航点功能
*/
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <sensor_msgs/BatteryState.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <fstream>
#include <iostream>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <tf2/LinearMath/Quaternion.h>
#include "socket_receiver.h"
using namespace std; // 在函数内部或局部使用

bool takeoff_flag = 0;
bool cmd_flag = 0;
ros::Time start_time, end_time;
double t_we_need, t_init_tag = 0;
bool log_en = true;
int pos_source = 1; // 1 vins 2 mocap
nav_msgs::Odometry offboard_odom;

std::string filename_vins;
std::string filename_mocap;

class VinsToMavros
{
public:
    VinsToMavros(ros::NodeHandle &nh)
    {
        // 订阅VINS话题
        sub_ = nh.subscribe("/iris_0/mavros/local_position/odom", 10, &VinsToMavros::vins_callback, this);
        mocap_sub_ = nh.subscribe("/mocap/odom", 10, &VinsToMavros::mocap_callback, this);
        // 发布到MAVROS
        pub_ = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose0", 10);
        if (pos_source == 1)
        {
            ROS_INFO("pos source :vins");
        }
        else if (pos_source == 2)
        {
            ROS_INFO("pos source :mocap");
        }

        std::time_t now = std::time(nullptr);
        std::tm *localTime = std::localtime(&now);
        std::ostringstream oss;                         //
        oss << std::put_time(localTime, "%Y%m%d_%H%M"); // 格式示例: 20231115_1430
        filename_vins = "/home/cat/output/m3588_2_" + oss.str() + "_vins.csv";
        filename_mocap = "/home/cat/output/m3588_2_" + oss.str() + "_mocap.csv";
    }

    void vins_callback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        if (t_init_tag == 0 && abs(msg->pose.pose.position.x) < 0.1 && abs(msg->pose.pose.position.y) < 0.1)
            t_init_tag = msg->header.stamp.toSec();
        if (t_init_tag != 0 && pos_source == 1)
        {
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header = msg->header;
            pose_stamped.header.frame_id = "map"; // 确保坐标系为map
            pose_stamped.pose = msg->pose.pose;   // 直接复制位姿数据
            pub_.publish(pose_stamped);
            // 存储最近一次 odom 数据
            latest_odom_ = *msg;
            if (msg->pose.pose.position.z > 0.1 && takeoff_flag == 0)
            {
                takeoff_flag = 1;
                start_time = ros::Time::now();
            }
        }

        if (log_en && t_init_tag != 0)
        {
            std::ofstream foutC(filename_vins, ios::app);
            t_we_need = msg->header.stamp.toSec() - t_init_tag;
            foutC.setf(ios::fixed, ios::floatfield);
            foutC.precision(7);
            foutC << t_we_need << " ";
            foutC.precision(7);
            foutC << msg->pose.pose.position.x << " "
                  << msg->pose.pose.position.y << " "
                  << msg->pose.pose.position.z << " "
                  << msg->pose.pose.orientation.x << " "
                  << msg->pose.pose.orientation.y << " "
                  << msg->pose.pose.orientation.z << " "
                  << msg->pose.pose.orientation.w << endl;
            foutC.close();
        }
    }
    void mocap_callback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        if (pos_source == 2)
        {
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header = msg->header;
            pose_stamped.header.frame_id = "map"; // 确保坐标系为map
            pose_stamped.pose = msg->pose.pose;   // 直接复制位姿数据
            pub_.publish(pose_stamped);
            // 存储最近一次 odom 数据
            latest_odom_ = *msg;
            if (msg->pose.pose.position.z > 0.1 && takeoff_flag == 0)
            {
                takeoff_flag = 1;
                start_time = ros::Time::now();
            }
        }
        if (log_en && t_init_tag != 0)
        {
            std::ofstream foutC(filename_mocap, ios::app);
            t_we_need = msg->header.stamp.toSec() - t_init_tag;
            foutC.setf(ios::fixed, ios::floatfield);
            foutC.precision(7);
            foutC << t_we_need << " ";
            foutC.precision(7);
            foutC << msg->pose.pose.position.x << " "
                  << msg->pose.pose.position.y << " "
                  << msg->pose.pose.position.z << " "
                  << msg->pose.pose.orientation.x << " "
                  << msg->pose.pose.orientation.y << " "
                  << msg->pose.pose.orientation.z << " "
                  << msg->pose.pose.orientation.w << endl;
            foutC.close();
        }
    }

    // 提供外部访问的接口
    nav_msgs::Odometry getLatestOdom() const
    {
        return latest_odom_;
    }

private:
    ros::Subscriber sub_;
    ros::Subscriber mocap_sub_;
    ros::Publisher pub_;
    nav_msgs::Odometry latest_odom_; // 存储最近一次收到的里程计消息
};

class CmdToMavros
{
public:
    CmdToMavros(ros::NodeHandle &nh, VinsToMavros &vins)
        : vins_node_(vins) // 接收引用
    {
        // 订阅 `/cmd` 话题
        cmd_sub_ = nh.subscribe("/position_cmd", 10, &CmdToMavros::cmdCallback, this);
        battery_sub_ = nh.subscribe("/iris_0/mavros/battery", 10, &CmdToMavros::batteryCallback, this);
        rc_sub_ = nh.subscribe("/iris_0/mavros/rc/in", 10, &CmdToMavros::rcCallback, this);
        state_sub_ = nh.subscribe("/iris_0/mavros/state", 10, &CmdToMavros::stateCallback, this);
        // 发布到 `/mavros/setpoint_raw/local`
        mavros_pub_ = nh.advertise<mavros_msgs::PositionTarget>("/iris_0/mavros/setpoint_raw/local", 10);
        trigger_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/traj_start_trigger", 10);
        arming_client_ = nh.serviceClient<mavros_msgs::CommandBool>("/iris_0/mavros/cmd/arming");
        set_mode_client_ = nh.serviceClient<mavros_msgs::SetMode>("/iris_0/mavros/set_mode");

        mission_timer = nh.createTimer(ros::Duration(0.05), &CmdToMavros::missionTimerCallback, this);

        takeoff_pose.pose.position.x = 0;
        takeoff_pose.pose.position.y = 0;
        takeoff_pose.pose.position.z = 1.2;

        // 定义数组保存坐标
        double square_coords[4][3] = {
            {0, -2, 1},
            {4, -2, 1},
            {4, 1.5, 1},
            {0, 1.5, 1}};

        // 存储成 geometry_msgs::Point 类型的 vector
        // std::vector<geometry_msgs::Point> square_points;

        for (int i = 0; i < 4; ++i)
        {
            geometry_msgs::Point pt;
            pt.x = square_coords[i][0];
            pt.y = square_coords[i][1];
            pt.z = square_coords[i][2];
            square_points.push_back(pt);
        }
    }
    void stateCallback(const mavros_msgs::State::ConstPtr &msg)
    {
        current_state = *msg;
    }
    void cmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr &msg)
    {
        cmd_flag = 1;
        mavros_msgs::PositionTarget target_msg;
        target_msg.header.stamp = ros::Time::now();
        target_msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        target_msg.header.frame_id = "map"; // 适配你的坐标系

        // 设置控制模式（位置 + 速度 + 偏航角）
        target_msg.type_mask =
            mavros_msgs::PositionTarget::IGNORE_AFX |
            mavros_msgs::PositionTarget::IGNORE_AFY |
            mavros_msgs::PositionTarget::IGNORE_AFZ |
            mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

        // 位置 (XYZ)
        target_msg.position.x = msg->position.x;
        target_msg.position.y = msg->position.y;
        target_msg.position.z = msg->position.z;

        // 速度 (XYZ)
        target_msg.velocity.x = msg->velocity.x;
        target_msg.velocity.y = msg->velocity.y;
        target_msg.velocity.z = msg->velocity.z;

        // 偏航角
        target_msg.yaw = msg->yaw;
        ROS_INFO("PosCmd: x: %.2f, y: %.2f, z:%.2f, yaw:%.2f ",msg->position.x,msg->position.y,msg->position.z,msg->yaw);
        //  发布到 MAVROS
        mavros_pub_.publish(target_msg);
    }
    void batteryCallback(const sensor_msgs::BatteryState::ConstPtr &msg)
    {
        float bat_cut = 10.5;
        double flyTime = 0;
        if (takeoff_flag)
        {
            end_time = ros::Time::now();
            ros::Duration elapsed_time = end_time - start_time;
            flyTime = elapsed_time.toSec();
        }

        if (msg->voltage > bat_cut)
            ROS_INFO("Battery Voltage: %.2f V,fly time:%.3f", msg->voltage, flyTime);
        else
            ROS_WARN("LOW BAT: %.2f V,fly time:%.3f", msg->voltage, flyTime);
    }
    void rcCallback(const mavros_msgs::RCIn::ConstPtr &msg)
    {
        static mavros_msgs::RCIn::ConstPtr last_rc_msg;
        // 例如输出前四个通道（roll, pitch, throttle, yaw）
        if (msg->channels.size() >= 8)
        {
            // ROS_INFO("RC Input - Roll: %d, Pitch: %d, Throttle: %d, Yaw: %d, ch4: %d, ch5: %d,ch6: %d, ch7: %d ",
            //          msg->channels[0],
            //          msg->channels[1],
            //          msg->channels[2],
            //          msg->channels[3],
            //          msg->channels[4],
            //          msg->channels[5],
            //          msg->channels[6],
            //          msg->channels[7]);
            if (msg->channels[4] > 1700)
            {
                if (!mission_active)
                {
                    mission_active = true;
                    // square_step = 0;
                    // last_switch_time = ros::Time::now();
                    // ROS_INFO("Starting square mission.");
                }
            }
            else
            {
                mission_active = false;
            }
            if (msg->channels[5] > 1700 && last_rc_msg->channels[5] < 1700)
            {
                geometry_msgs::PoseStamped msg;
                msg.header.stamp = ros::Time::now(); // 加上时间戳
                msg.header.frame_id = "world";
                msg.pose = offboard_odom.pose.pose;
                trigger_pub_.publish(msg);
                ROS_INFO("Starting plan.");
            }
            else if(msg->channels[5]<1700){
                offboard_odom = vins_node_.getLatestOdom(); // 调用 VinsToMavros 的方法
            }

            last_rc_msg = msg; // 更新存储
        }
        else
        {
            ROS_WARN("RC input has fewer than 8 channels!");
        }
    }

    void missionTimerCallback(const ros::TimerEvent &)
    {
        
        static bool inFly = 0;
        static ros::Time last_request = ros::Time::now();
        if (!mission_active)
            return;
        
        const auto &pt = square_points[square_step];
        mavros_msgs::PositionTarget target;
        target.header.stamp = ros::Time::now();
        target.header.frame_id = "map";

        target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        target.type_mask = (mavros_msgs::PositionTarget::IGNORE_VX |
                            mavros_msgs::PositionTarget::IGNORE_VY |
                            mavros_msgs::PositionTarget::IGNORE_VZ |
                            mavros_msgs::PositionTarget::IGNORE_AFX |
                            mavros_msgs::PositionTarget::IGNORE_AFY |
                            mavros_msgs::PositionTarget::IGNORE_AFZ |
                            mavros_msgs::PositionTarget::FORCE |
                            mavros_msgs::PositionTarget::IGNORE_YAW_RATE);

        if (!inFly && 0)
        { // auto takeoff
            offb_set_mode.request.custom_mode = "OFFBOARD";
            arm_cmd.request.value = true;
            if (current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(2.0)))
            {
                if (set_mode_client_.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent)
                {
                    ROS_INFO("Offboard enabled");
                    inFly = true;
                }
                last_request = ros::Time::now();
            }
            // else
            // {
            //     if (!current_state.armed &&
            //         (ros::Time::now() - last_request > ros::Duration(2.0)))
            //     {
            //         if (arming_client_.call(arm_cmd) &&
            //             arm_cmd.response.success)
            //         {
            //             ROS_INFO("Vehicle armed");
            //         }
            //         last_request = ros::Time::now();
            //     }
            // }
            // if(current_state.mode == "OFFBOARD" && current_state.armed){
            target.position.x = 0.0;
            target.position.y = 0.0;
            target.position.z = 1.2;
            target.yaw = 0.0;

            mavros_pub_.publish(target);
            
        }
        if (inFly && 0)
        {
            if ((ros::Time::now() - last_switch_time).toSec() > 5.0)
            {
                square_step = (square_step + 1) % 4;
                last_switch_time = ros::Time::now();
            }

            target.position.x = pt.x;
            target.position.y = pt.y;
            target.position.z = pt.z;
            target.yaw = 0.0;

            mavros_pub_.publish(target);
        }
        if(takeoff_flag && cmd_flag == 0){
            
            target.position.x = offboard_odom.pose.pose.position.x;
            target.position.y = offboard_odom.pose.pose.position.y;
            target.position.z = offboard_odom.pose.pose.position.z;
            // target.position.x = 0;
            // target.position.y = 0;
            // target.position.z = 1;
            target.yaw = 0.0;
            mavros_pub_.publish(target);
            // std::cout << "mission pub" << std::endl;

        }
    }

private:
    ros::Subscriber cmd_sub_;
    ros::Subscriber battery_sub_;
    ros::Subscriber rc_sub_;
    ros::Subscriber state_sub_;

    ros::Publisher mavros_pub_;
    ros::Publisher trigger_pub_;

    ros::Timer mission_timer;

    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;

    mavros_msgs::State current_state;
    mavros_msgs::SetMode offb_set_mode;
    mavros_msgs::CommandBool arm_cmd;

    bool mission_active = false; // 是否启动任务
    int square_step = 0;         // 当前正方形步骤
    ros::Time last_switch_time;  // 上一次切换时间
    std::vector<geometry_msgs::Point> square_points;
    geometry_msgs::PoseStamped takeoff_pose;

    VinsToMavros &vins_node_;
};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "px4ctrlpx4");
    ros::NodeHandle nh; // 只创建一个 NodeHandle，传递给多个类
    ros::NodeHandle private_nh("~");
        // 获取参数
    int port;
     private_nh.param<int>("port", port, 5001);
    std::string output_topic;
    private_nh.param<std::string>("output_topic", output_topic, "waypoints");
    private_nh.param<int>("pos_source", pos_source,1);


    VinsToMavros vins_node(nh);          // 创建VinsToMavros订阅者
    CmdToMavros cmd_node(nh, vins_node); // 创建CmdToMavros订阅者


   
    // 初始化发布器
    ros::Publisher waypoint_pub = nh.advertise<geometry_msgs::PoseStamped>(output_topic, 10);

    // 启动Socket服务器
    SocketReceiver receiver(port);
    receiver.start();

    ROS_INFO_STREAM("Socket waypoint receiver started on port " << port);

    // 处理ROS回调函数
    while (ros::ok())
    {
        // 处理回调函数
        ros::spinOnce();
        auto waypoints = receiver.popProcessedWaypoints(1);
        if (waypoints.empty())
        {
            // std::cout << "航点数据为空!" << std::endl;
        }
        else
        {
            receiver.clearWaypoints(); // 显式清空
            std::cout << "收到 " << waypoints.size() << " 个航点" << std::endl;
            for (const auto &wp : waypoints)
            {
                std::cout << "  x: " << wp.latitude
                          << ", y: " << wp.longitude
                          << ", z: " << wp.altitude
                          << ", yaw: " << wp.yaw << std::endl;
                geometry_msgs::PoseStamped msg;
                msg.header.stamp = ros::Time::now(); // 加上时间戳
                msg.header.frame_id = "world";
                msg.pose.position.x = wp.latitude;  // 经度 → X
                msg.pose.position.y = wp.longitude; // 纬度 → Y
                msg.pose.position.z = wp.altitude;  // 高度 → Z
                // 设置朝向（将yaw转为四元数）
                tf2::Quaternion q;
                q.setRPY(0, 0, wp.yaw); // roll=0, pitch=0, yaw=航向角
                msg.pose.orientation.x = q.x();
                msg.pose.orientation.y = q.y();
                msg.pose.orientation.z = q.z();
                msg.pose.orientation.w = q.w();

                waypoint_pub.publish(msg);
                ROS_INFO("publish target.");
            }
        }
    }

    return 0;
}
