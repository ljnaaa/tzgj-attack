#ifndef TJSP_ATTACK_2020_ATTACK_HPP
#define TJSP_ATTACK_2020_ATTACK_HPP

#include <numeric>
#include "base.hpp"
#include "imageshow.hpp"
#include "ThreadPool.h"
#include <thread>
#include <future>
#include "layers.hpp"
#include <utility>
#include "dirent.h"

//CPP版本，导入tensorflow
#include "tensorflow/core/framework/graph.pb.h"
#include "tensorflow/core/public/session.h"
#include "tensorflow/core/framework/tensor.h"
#include "google/protobuf/wrappers.pb.h"
#include "tensorflow/core/util/command_line_flags.h"


#include "google/protobuf/wrappers.pb.h"
#include <ros/ros.h>
#include <fstream>
#include <roborts_msgs/GimbalAngle.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <roborts_msgs/ShootCmd.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <math.h>
#define debugit std::cout<<__LINE__<<std::endl;
//#include <roborts_msgs/Mess.h>



// 多车版本
#include <roborts_msgs/single_car.h>
#include <roborts_msgs/visual_detection.h>
#include <roborts_msgs/GimbalFb.h>


using tensorflow::string;
using tensorflow::Tensor;
using namespace tensorflow;

//模型路径
const string model_path = "/home/icra01/icra/src/tzgj-attack/tjsp_attack_2020/Model/happyModel.pb";
/*输入输出节点*/
const string input_name = "input_2:0";
const string output_name = "y_1/Softmax:0"; 

const int secmult = 1000000;

namespace armor
{


    typedef enum{
        SEND_STATUS_AUTO_PLACEHOLDER = 0x00, // 占位符
        SEND_STATUS_AUTO_AIM_FORMER = 0x30,  // 打上一帧
        SEND_STATUS_AUTO_AIM = 0x31,         // 去瞄准
        SEND_STATUS_AUTO_SHOOT = 0x32,       // 去打
        SEND_STATUS_AUTO_NOT_FOUND = 0x33,   // 没找
        SEND_STATUS_WM_AIM = 0x34            // 使用大风车
    } emSendStatusA;
/*
  自瞄基类, 多线程共享变量用
 */
    class AttackBase
    {
    protected:
        static std::mutex s_mutex;                     // 互斥锁
        static std::atomic<int64_t> s_latestTimeStamp; // 已经发送的帧编号
        static std::deque<Target> s_historyTargets;    // 打击历史, 最新的在头部, [0, 1, 2, 3, ....]
        static Kalman kalman;                          // 卡尔曼滤波
    };
    std::mutex AttackBase::s_mutex;
    std::atomic<int64_t> AttackBase::s_latestTimeStamp(0);
    std::deque<Target> AttackBase::s_historyTargets;
    Kalman AttackBase::kalman;

    //利用一阶运动模型对IMU进行补偿，减小误差
    class IMUBuff
    {
        public:
            const int LEN = 5;
            std::deque<roborts_msgs::GimbalFb> imu_history;
            ros::Subscriber imu_sub;
            
            IMUBuff()
            {
                ros::NodeHandle imu_n_;
                imu_sub = imu_n_.subscribe("gimbal_feedback", 1, &IMUBuff::storeIMUHistory, this);
            }

            void storeIMUHistory(const roborts_msgs::GimbalFb::ConstPtr &fb)
            {
                //std::cout<<"aaaaaaaaaaaaaaaaaa"<<std::endl;
                while(imu_history.size() >= LEN) imu_history.pop_back();
                imu_history.push_front(*fb);
            }

            double getPredYaw(int64_t timeStamp)
            {
                int64_t post_time, pre_time;
                double post_yaw, pre_yaw;
                if(imu_history.size()<2)
                {
                    return imu_history[0].imu.yaw_angle;
                }
                if((imu_history[0].stamp).toSec()*secmult <= timeStamp)
                {
                    post_yaw = imu_history[0].imu.yaw_angle;
                    post_time = imu_history[0].stamp.toSec()*secmult;
                    pre_yaw = imu_history[1].imu.yaw_angle;
                    pre_time = imu_history[1].stamp.toSec()*secmult;
                }
                else
                {
                    for(auto it=imu_history.begin(); it!=imu_history.end(); it++)
                    {
                        if((it->stamp).toSec()*secmult >= timeStamp)
                        {
                            post_yaw = (*it).imu.yaw_angle;
                            post_time = (*it).stamp.toSec()*secmult;
                        }
                        else
                        {
                            pre_yaw = (*it).imu.yaw_angle;
                            pre_time = (*it).stamp.toSec()*secmult;
                            break;
                        }
                    }
                }
                double k = (post_yaw-pre_yaw)/double(post_time-pre_time);
                double pred_yaw = pre_yaw + k*(timeStamp-pre_time);
                
                while(pred_yaw > M_PI) pred_yaw -= 2*M_PI;
                while(pred_yaw < -M_PI) pred_yaw += 2*M_PI;
                
                int64_t last = imu_history[0].stamp.toSec()*secmult;
                // std::cout<<"PRE YAW: "<<pre_yaw<<" , TIME: "<<pre_time<<"\n";
                // std::cout<<"POST YAW: "<<post_yaw<<" , TIME: "<<post_time<<"\n";
                // std::cout<<"k:"<<k<<std::endl;
                // std::cout<<"PREDICT YAW: " << pred_yaw<<" , TIME: "<<timeStamp<<"\n";
                return pred_yaw;
            }

            double getLatestYaw()
            {
                return imu_history[0].imu.yaw_angle;
            }

            float getPredPitch(int64_t timeStamp)
            {
                int64_t post_time, pre_time;
                float post_pitch, pre_pitch;
                if(imu_history.size()<2)
                {
                    return imu_history[0].imu.pitch_angle;
                }
                if((imu_history[0].stamp).toSec()*secmult <= timeStamp)
                {
                    post_pitch = imu_history[0].imu.pitch_angle;
                    post_time = imu_history[0].stamp.toSec()*secmult;
                    pre_pitch = imu_history[1].imu.pitch_angle;
                    pre_time = imu_history[1].stamp.toSec()*secmult;
                }
                else
                {
                    for(auto it=imu_history.begin(); it!=imu_history.end(); it++)
                    {
                        if((it->stamp).toSec()*secmult >= timeStamp)
                        {
                            post_pitch = (*it).imu.pitch_angle;
                            post_time = (*it).stamp.toSec()*secmult;
                        }
                        else
                        {
                            pre_pitch = (*it).imu.pitch_angle;
                            pre_time = (*it).stamp.toSec()*secmult;
                            break;
                        }
                    }
                }
                float k = (post_pitch-pre_pitch)/float(post_time-pre_time);
                float pred_pitch = pre_pitch + k*(timeStamp-pre_time);
                
                while(pred_pitch > 2*M_PI) pred_pitch -= 2*M_PI;
                while(pred_pitch < -2*M_PI) pred_pitch += 2*M_PI;
                
                return pred_pitch;


            }
    };

    //利用一阶运动模型对Odom进行传感器对齐
    class OdomBuff
    {
        public:
            OdomBuff()
            {
                ros::NodeHandle n;
                odom_sub = n.subscribe("odom",1,&OdomBuff::odom_cb,this);
            }

            void pose_judge(cv::Point3d& inchassis, const ros::Time& image_timeStamp)
            {
                update_last_frame(image_timeStamp);
                double delta_yaw = transform(last_frame,standard);
                            /* yaw 为绕y轴旋转的 */
                m_rotY = (cv::Mat_<double>(3, 3)
                          << std::cos(delta_yaw), 0, std::sin(delta_yaw),
                            0, 1, 0,
                            -std::sin(delta_yaw),0, std::cos(delta_yaw));
                cv::Mat _pts = (cv::Mat_<double>(3, 1) << inchassis.x, inchassis.y, inchassis.z);
                cv::Mat ptsInWorldMat = m_rotY * _pts;
                inchassis.x = ptsInWorldMat.at<double>(0);
                inchassis.y = ptsInWorldMat.at<double>(1);
                inchassis.z = ptsInWorldMat.at<double>(2);
            }

            void transback(cv::Point3d& inworld)
            {
                cv::Mat _pts = (cv::Mat_<double>(3, 1) << inworld.x, inworld.y, inworld.z);
                cv::Mat ptsInWorldMat = m_rotY.inv() * _pts;
                inworld.x = ptsInWorldMat.at<double>(0);
                inworld.y = ptsInWorldMat.at<double>(1);
                inworld.z = ptsInWorldMat.at<double>(2);
            }

            void reset_standard()
            {
                standard = last_frame;
            }

        private:
            nav_msgs::Odometry last_frame;
            const int LEN = 5;
            std::deque<nav_msgs::Odometry> odom_history;
            nav_msgs::Odometry standard;    //standard pose of one kalman filter
            ros::Subscriber odom_sub;
            cv::Mat m_rotY;

            ~OdomBuff(){}
            
            void odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
            {
                while(odom_history.size() >= LEN) odom_history.pop_back();
                odom_history.push_front(*msg);
                last_frame = *msg;
            }

            void update_last_frame(const ros::Time& image_timeStamp)
            {
                last_frame.header.stamp = image_timeStamp;
                nav_msgs::Odometry post_msg, pre_msg;
                if(odom_history.size()<2)
                {
                    last_frame = odom_history[0];
                    return;
                }
                if(odom_history[0].header.stamp <= image_timeStamp)
                {
                    post_msg = odom_history[0];
                    pre_msg = odom_history[1];
                }
                else
                {
                    for(auto it=odom_history.begin(); it!=odom_history.end(); it++)
                    {
                        if(it->header.stamp >= image_timeStamp)
                        {
                            post_msg = *it;
                        }
                        else
                        {
                            pre_msg = *it;
                            break;
                        }
                    }
                }
                update_pose(last_frame, post_msg, pre_msg, image_timeStamp);
                update_twist(last_frame, post_msg, pre_msg, image_timeStamp);
                
                // std::cout<<"POST POSE: "<<post_msg.pose.pose<<" , TIME: "<<post_msg.header.stamp<<"\n";
                // std::cout<<"PRE POSE: "<<pre_msg.pose.pose<<" , TIME: "<<pre_msg.header.stamp<<"\n";
                // std::cout<<"PRED POSE: "<<last_frame.pose.pose<<" , TIME: "<<last_frame.header.stamp<<"\n";
                // std::cout<<"POST TWIST: "<<post_msg.twist.twist<<" , TIME: "<<post_msg.header.stamp<<"\n";
                // std::cout<<"PRE TWIST: "<<pre_msg.twist.twist<<" , TIME: "<<pre_msg.header.stamp<<"\n";
                // std::cout<<"PRED TWIST: "<<last_frame.twist.twist<<" , TIME: "<<last_frame.header.stamp<<"\n";                
            }

            void update_pose(nav_msgs::Odometry&p,  nav_msgs::Odometry p1, nav_msgs::Odometry p2, const ros::Time& image_timeStamp)
            {
                geometry_msgs::Pose pred_pose;
                float k_x = (p1.pose.pose.position.x-p2.pose.pose.position.x) / (p1.header.stamp.toSec()*secmult - p2.header.stamp.toSec()*secmult);
                float k_y = (p1.pose.pose.position.y-p2.pose.pose.position.y) / (p1.header.stamp.toSec()*secmult - p2.header.stamp.toSec()*secmult);
                float k_z = (p1.pose.pose.position.z-p2.pose.pose.position.z) / (p1.header.stamp.toSec()*secmult - p2.header.stamp.toSec()*secmult);
                float k_x1 = (p1.pose.pose.orientation.x-p2.pose.pose.orientation.x) / (p1.header.stamp.toSec()*secmult - p2.header.stamp.toSec()*secmult);
                float k_y1 = (p1.pose.pose.orientation.y-p2.pose.pose.orientation.y) / (p1.header.stamp.toSec()*secmult - p2.header.stamp.toSec()*secmult);
                float k_z1 = (p1.pose.pose.orientation.z-p2.pose.pose.orientation.z) / (p1.header.stamp.toSec()*secmult - p2.header.stamp.toSec()*secmult);
                float k_w = (p1.pose.pose.orientation.w-p2.pose.pose.orientation.w) / (p1.header.stamp.toSec()*secmult - p2.header.stamp.toSec()*secmult);
                
                pred_pose.position.x = p2.pose.pose.position.x + (image_timeStamp.toSec()*secmult - p2.header.stamp.toSec()*secmult)*k_x;
                pred_pose.position.y = p2.pose.pose.position.y + (image_timeStamp.toSec()*secmult - p2.header.stamp.toSec()*secmult)*k_y;
                pred_pose.position.z = p2.pose.pose.position.z + (image_timeStamp.toSec()*secmult - p2.header.stamp.toSec()*secmult)*k_z;
                pred_pose.orientation.x = p2.pose.pose.orientation.x + (image_timeStamp.toSec()*secmult - p2.header.stamp.toSec()*secmult)*k_x1;
                pred_pose.orientation.y = p2.pose.pose.orientation.y + (image_timeStamp.toSec()*secmult - p2.header.stamp.toSec()*secmult)*k_y1;
                pred_pose.orientation.z = p2.pose.pose.orientation.z + (image_timeStamp.toSec()*secmult - p2.header.stamp.toSec()*secmult)*k_z1;
                pred_pose.orientation.w = p2.pose.pose.orientation.w + (image_timeStamp.toSec()*secmult - p2.header.stamp.toSec()*secmult)*k_w;

                p.pose.pose = pred_pose;

                for(int i=0; i<36; i++)
                {
                    p.pose.covariance[i] = (p1.pose.covariance[i] + p2.pose.covariance[i]) / 4;
                }
                
            }

            void update_twist(nav_msgs::Odometry&t,  nav_msgs::Odometry t1, nav_msgs::Odometry t2, const ros::Time& image_timeStamp)
            {
                geometry_msgs::Twist pred_twist;
                float kl_x = (t1.twist.twist.linear.x - t2.twist.twist.linear.x) / (t1.header.stamp.toSec()*secmult - t2.header.stamp.toSec()*secmult);
                float kl_y = (t1.twist.twist.linear.y - t2.twist.twist.linear.y) / (t1.header.stamp.toSec()*secmult - t2.header.stamp.toSec()*secmult);
                float kl_z = (t1.twist.twist.linear.z - t2.twist.twist.linear.z) / (t1.header.stamp.toSec()*secmult - t2.header.stamp.toSec()*secmult);
                float ka_x = (t1.twist.twist.angular.x - t2.twist.twist.angular.x) / (t1.header.stamp.toSec()*secmult - t2.header.stamp.toSec()*secmult);
                float ka_y = (t1.twist.twist.angular.y - t2.twist.twist.angular.y) / (t1.header.stamp.toSec()*secmult - t2.header.stamp.toSec()*secmult);
                float ka_z = (t1.twist.twist.angular.z - t2.twist.twist.angular.z) / (t1.header.stamp.toSec()*secmult - t2.header.stamp.toSec()*secmult);

                pred_twist.linear.x = t2.twist.twist.linear.x + (image_timeStamp.toSec()*secmult - t2.header.stamp.toSec()*secmult)*kl_x;
                pred_twist.linear.y = t2.twist.twist.linear.y + (image_timeStamp.toSec()*secmult - t2.header.stamp.toSec()*secmult)*kl_y;
                pred_twist.linear.z = t2.twist.twist.linear.z + (image_timeStamp.toSec()*secmult - t2.header.stamp.toSec()*secmult)*kl_z;
                pred_twist.angular.x = t2.twist.twist.angular.x + (image_timeStamp.toSec()*secmult - t2.header.stamp.toSec()*secmult)*ka_x;
                pred_twist.angular.y = t2.twist.twist.angular.y + (image_timeStamp.toSec()*secmult - t2.header.stamp.toSec()*secmult)*ka_y;
                pred_twist.angular.z = t2.twist.twist.angular.z + (image_timeStamp.toSec()*secmult - t2.header.stamp.toSec()*secmult)*ka_z;

                t.twist.twist = pred_twist;
                for(int i=0; i<36; i++)
                {
                    t.twist.covariance[i] = (t1.twist.covariance[i] + t2.twist.covariance[i]) / 4;
                }
            }





            double transform(nav_msgs::Odometry& last,nav_msgs::Odometry&standard)
            {
                double last_yaw = orien2yaw(last_frame);
                double standard_yaw = orien2yaw(standard);
                return standard_yaw - last_yaw;   //I don't know why need to negative
            }

            double orien2yaw(nav_msgs::Odometry& frame)
            {
                tf::Quaternion quat;
                tf::quaternionMsgToTF(frame.pose.pose.orientation,quat);
                
                double roll,pitch,yaw;
                tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);
                return yaw;
            }


            




    };


/*
  自瞄主类
 */
    class Attack : AttackBase
    {
    private:
        ImageShowClient &m_is;
        cv::Mat m_bgr;
        cv::Mat m_bgr_raw;
        // 目标
        std::vector<Target> m_preTargets; // 预检测目标
        std::vector<Target> m_targets;    // 本次有效目标集合
        // 开小图
        cv::Point2i m_startPt;
        bool m_isEnablePredict;     // 是否开预测

        int64_t m_currentTimeStamp; // 当前时间戳
        PID &m_pid;                 // PID
        bool m_isUseDialte;         // 是否膨胀
        bool mode;                  // 红蓝模式
        std::ofstream fs;
        tf::TransformListener* listener;
        int cur_frame;
        IMUBuff* imu_buff;
        OdomBuff* odom_buff;
        roborts_msgs::visual_detection last_vision_data;
        Session *session;

    public:
        explicit Attack(ImageShowClient &isClient, PID &pid) : m_pid(pid),
                                                               m_is(isClient),
                                                               m_isEnablePredict(true), m_currentTimeStamp(0), m_isUseDialte(false)
        {
            mycnn::loadWeights("../info/dumpe2.nnet");
            m_isUseDialte = stConfig.get<bool>("auto.is-dilate");
            listener = new tf::TransformListener;
            cur_frame=0;
            imu_buff = new IMUBuff();
            odom_buff = new OdomBuff();

            //C++ tensorflow
            /* 创建session */
            Status status = NewSession(SessionOptions(), &session);
            if (!status.ok())
                std::cout << status.ToString() << std::endl;
            /* 初始化session */
            init_my_tf(session);

            // fs.open("data.csv");
        }
        ~Attack()
        {
            session->Close();
            delete session;

            // fs.close();
        }
        void setMode(bool colorMode) { mode = colorMode; }

    private:

    /**
     * @name m_preDetect
     * @func 通过hsv筛选和进行预处理获得装甲板
     */
        void m_preDetect(int pmode)
        {
            DEBUG("m_preDetect")
            /* 使用inRange对颜色进行筛选 */
            cv::Mat bgrChecked;
            m_is.clock("inRange");
            if (mode)
            {
                /* 蓝色 */
                cv::inRange(m_bgr, cv::Scalar(0, 0, 140), cv::Scalar(180, 255, 255), bgrChecked);
            }
            else
            {
                /* 红色 */
                cv::inRange(m_bgr, cv::Scalar(140, 0, 0), cv::Scalar(255, 255, 180), bgrChecked);
            }
            m_is.clock("inRange");
            DEBUG("inRange end")
            /* 进行膨胀操作（默认关闭） */
            m_is.addImg("bgrChecked", bgrChecked, false);
            if (m_isUseDialte)
            {
                cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
                dilate(bgrChecked, bgrChecked, element);
                m_is.addImg("dilate",  bgrChecked, false);
            }
            /* 寻找边缘，并圈出countours */
            std::vector<std::vector<cv::Point2i>> contours;
            cv::findContours(bgrChecked, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
            // m_is.addEvent("contours", contours);
            DEBUG("findContours end")
            /* 对灯条进行筛选 */
            std::vector<Light> lights;
            for (const auto &_pts : contours)
            {
                /* 设定最小面积>=5 */
                if (_pts.size() < 5)
                    continue;
                /* 寻找最小外接矩形 */
                cv::RotatedRect rRect = cv::minAreaRect(_pts);
                /* 设定长宽比2/3～3/2 */
                double hw = rRect.size.height / rRect.size.width;
                if (0.6667 < hw && hw < 1.5)
                    continue;
                /* 寻找灯条的顶部中点，底部中点与倾斜角 */
                Light _light;
                cv::Point2f topPt;      //顶部中点
                cv::Point2f bottomPt;   //底部中点
                cv::Point2f pts[4];

                rRect.points(pts);
                if (rRect.size.width > rRect.size.height)//根据外接矩形的特性需调整点
                {
                    bottomPt = (pts[2] + pts[3]) / 2.0;
                    topPt = (pts[0] + pts[1]) / 2.0;
                    _light.angle = cv::abs(rRect.angle);
                }
                else
                {
                    bottomPt = (pts[1] + pts[2]) / 2;
                    topPt = (pts[0] + pts[3]) / 2;
                    _light.angle = cv::abs(rRect.angle - 90);
                }
                /* 判断顶部和底部中点是否设置正确，并将中心点与长度一并写入_light参数中 */
                if (topPt.y > bottomPt.y)
                {
                    _light.topPt = bottomPt;
                    _light.bottomPt = topPt;
                }
                else
                {
                    _light.topPt = topPt;
                    _light.bottomPt = bottomPt;
                }
                //std::cout << "Top:" << _light.topPt.x << ", " <<_light.topPt.y << std::endl;
                //std::cout << "Bottom" << _light.bottomPt.x << ", " << _light.bottomPt.y << std::endl;
                _light.centerPt = rRect.center;             //中心点
                if(rRect.center.y + m_startPt.y < 200)  continue; 
                std::cout << rRect.center.x << ", " << rRect.center.y + m_startPt.y << std::endl;
                _light.length = cv::norm(bottomPt - topPt); //长度
                 /* 判断长度和倾斜角是否合乎要求 */
                if (_light.length < 3.0 || 800.0 < _light.length || cv::abs(_light.angle - 90) > 30.0)
                    continue;
                lights.emplace_back(_light);
            }
            DEBUG("lights end")
            // m_is.addEvent("lights", lights);
            /* 对筛选出的灯条按x大小进行排序 */
            std::sort(lights.begin(), lights.end(), [](Light &a_, Light &b_) -> bool {
                return a_.centerPt.x < b_.centerPt.x;
            });
            /* 对灯条进行两两组合并筛选出预检测的装甲板 */
            for (size_t i = 0; i < lights.size(); ++i)
            {
                for (size_t j = i + 1; j < lights.size(); ++j)
                {
                    cv::Point2f AC2BC = lights[j].centerPt - lights[i].centerPt;
                    double minLength = cv::min(lights[i].length, lights[j].length);
                    double deltaAngle = cv::abs(lights[i].angle - lights[j].angle);
                    /* 对灯条组的长度，角度差，中心点tan值，x位置等进行筛选 */
                    if ((deltaAngle > 23.0 && minLength < 20) || (deltaAngle > 11.0 && minLength >= 20) ||
                        cv::abs(lights[i].length - lights[j].length) / minLength > 0.5 ||
                        cv::fastAtan2(cv::abs(AC2BC.y), cv::abs(AC2BC.x)) > 25.0 ||
                        AC2BC.x / minLength > 7.1 || cv::norm(AC2BC) / minLength > 4.0)
                        continue;
                    Target target;
                    /* 计算像素坐标 */
                    target.setPixelPts(lights[i].topPt, lights[i].bottomPt, lights[j].bottomPt, lights[j].topPt,
                                       m_startPt);
                    /*  我们不用大装甲
                        if (cv::norm(AC2BC) / minLength > 4.9)
                            target.type = TARGET_LARGE; // 大装甲
                    */
                    /* 获得扩展区域像素坐标, 若无法扩展则放弃该目标 */
                    if (!target.convert2ExternalPts2f())
                        continue;
                    if(pmode == 0)
                        m_targets.emplace_back(target);
                    else if(pmode == 1)
                        m_preTargets.emplace_back(target);
                }
            }
            if(pmode == 0)
                m_is.addClassifiedTargets("After Classify", m_targets);
            else if(pmode == 1)
                m_is.addEvent("preTargets", m_preTargets);
            DEBUG("preTargets end")
        }
        int m_cropNameCounter = 0;

        /**
         * @name getThreshold
         * @param mat 图片
         * @param thre_proportion 比例阈值 0.1
         * @func 得到二值化阈值
         * @return i 二值化阈值
         */ 
        int getThreshold(const cv::Mat &mat, double thre_proportion = 0.1)
        {
            /* 计算总像素数目 */
            uint32_t iter_rows = mat.rows;
            uint32_t iter_cols = mat.cols;
            auto sum_pixel = iter_rows * iter_cols;
            /* 判断是否连续*/
            if (mat.isContinuous())
            {
                iter_cols = sum_pixel;
                iter_rows = 1;
            }
            /* 新建数组置零 */
            int histogram[256];
            memset(histogram, 0, sizeof(histogram));
             /* 像素排序 */
            for (uint32_t i = 0; i < iter_rows; ++i)
            {
                const auto *lhs = mat.ptr<uchar>(i);
                for (uint32_t j = 0; j < iter_cols; ++j)
                    ++histogram[*lhs++];
            }
            auto left = thre_proportion * sum_pixel;
            int i = 255;
            while ((left -= histogram[i--]) > 0)
                ;
            return i > 0 ? i : 0;
        }
        
        /**
         * @name init_my_tf
         * @param session 交互接口
         * @func 读取模型并设置到session中
         * @return None
         */ 
        inline void init_my_tf(Session *session)
        {
            /* 从pb文件中读取模型 */
            GraphDef graph_def;
            Status status = ReadBinaryProto(Env::Default(), model_path, &graph_def);  //读取Graph, 如果是文本形式的pb,使用ReadTextProto
            if (!status.ok())
                std::cout << status.ToString() << std::endl;

            /* 将模型设置到创建的Session里 */
            status = session->Create(graph_def);
            if (!status.ok())
                std::cout << status.ToString() << std::endl;
        }


        /**
         * @name loadAndPre
         * @param img 图片
         * @param result
         * @func 进行图片的预处理和高光补偿
         * @return true/false
         */ 
        bool loadAndPre(cv::Mat img, cv::Mat &result)
        {
            if (img.cols == 0)
                return false;
            /* 调整大小 同比缩放至fixedsize*fixedsize以内 */
            if (img.cols < img.rows)
                resize(img, img, {int(img.cols * 1.0 / img.rows * fixedSize), fixedSize});
            else
                resize(img, img, {fixedSize, int(img.rows * 1.0 / img.cols * fixedSize)});
            /* 剪去边上多余部分 */
            int cutRatio1 = 0.15 * img.cols;
            int cutRatio2 = 0.05 * img.rows;
            cv::Mat blank = cv::Mat(cv::Size(fixedSize, fixedSize), img.type(), cv::Scalar(0));                           //新建空白
            cv::Mat mask = img(cv::Rect(cutRatio1, cutRatio2, img.cols - 2 * cutRatio1, img.rows - 2 * cutRatio2));       //建立腌摸
            cv::Mat imageROI = blank(cv::Rect(cutRatio1, cutRatio2, img.cols - 2 * cutRatio1, img.rows - 2 * cutRatio2)); //建立需要覆盖区域的ROI
            mask.copyTo(imageROI, mask);
            int thre = getThreshold(blank);   //均值获取阈值
            result = blank.clone();
            /* 使用二值化阈值补高光 */
            for (int i = 0; i < result.rows; i++)
            {
                for (int j = 0; j < result.cols; j++)
                {
                    if ((int)result.at<u_char>(i, j) > thre)
                        result.at<u_char>(i, j) = 200;    
                }
            }
            return true;
        }

        
        /**
         * @name Mat2Tensor
         * @param img_temp 图片向量
         * @param input 计算图的输入张量
         * @func 将图片向量转化为tensor，加速运算
         */ 
        void Mat2Tensor(std::vector<cv::Mat>& img_temp, Tensor& input, int& size){

            auto input_tensor_mapped = input.tensor<float, 4>();

            for(int i = 0; i < img_temp.size(); ++i)
            {   
                //std::cout << "+++++++++++++++++++++++++++++++++++++" << std::endl;
                //std::cout << img_temp[i].type() << std::endl;
                uint8_t *source_data = (uint8_t *)img_temp[i].data;
                for (int y = 0; y < fixedSize; ++y)
                {
                    uint8_t *source_row = source_data + y * fixedSize;
                    for (int x = 0; x < fixedSize; ++x)
                    {
                        uint8_t *source_pixel = source_row + x;

                        input_tensor_mapped(i, y, x, 0) = uint8_t(*source_pixel);
                    }
                }
                i++;
            }

        }

        /**
         * @name m_classify_single_tensor
         * @param isSave 是否保存样本图片
         * @func 魔改后的分类器节点
         */ 
        void m_classify_single_tensor(bool isSave = false)
        {
            if (m_preTargets.empty())
                return;
            std::vector<cv::Mat> img_temp;
            std::vector<int> seq_temp;
            int i = 0;
            for (auto &_tar : m_preTargets)
            {
                // ros::Time pretime=ros::Time::now();
                cv::Rect tmp = cv::boundingRect(_tar.pixelPts2f_Ex);
                cv::Mat tmp2 = m_bgr_raw(tmp).clone();


                                /* 将图片变成目标大小 */
                cv::Mat transMat = cv::getPerspectiveTransform(_tar.pixelPts2f_Ex,
                                                               _tar.pixelPts2f_Ex);
                cv::Mat _crop;

                /* 投影变换 */
                cv::warpPerspective(tmp2, _crop, transMat, cv::Size(tmp2.size())); 
                /* 转灰度图 */
                cv::cvtColor(_crop, _crop, cv::COLOR_BGR2GRAY);
                /* 储存图 */
                if (isSave)
                {
                    cv::imwrite(cv::format("/home/icra01/images/%d.png", m_cropNameCounter++), _crop);
                }
                // ros::Time now=ros::Time::now();
                // std::cout<<"first"<<" "<<now-pretime<<"  ";
                // std::cout<<std::endl;
                cv::Mat image;

                    // ros::Time now=ros::Time::now();
                    // std::cout<<"first"<<" "<<now-pretime<<"  ";
                    // std::cout<<std::endl;
                if(_crop.cols){
                    resize(_crop,_crop,{fixedSize, fixedSize});
                    img_temp.emplace_back(_crop);
                    //因为最后要押的是Target，而不是cv::Mar，故需要记录一一对应的序号
                    seq_temp.emplace_back(i);
                    i++;
                }

            }
            int size = img_temp.size();
            //cout << (size == seq_temp.size());
            if(size == 0)   return ;
            Tensor input(DT_FLOAT, TensorShape({size, fixedSize, fixedSize, 1}));

            Mat2Tensor(img_temp, input, size);

            std::vector<tensorflow::Tensor> outputs;

            /* 计算最后结果 */
            ros::Time pretime1=ros::Time::now();

            TF_CHECK_OK(session->Run({std::pair<string, Tensor>(input_name, input)}, {output_name}, {}, &outputs));
            
            Tensor t;
            //cout << (outputs.size() == size) << endl;
            cout << outputs[0].shape() << "," << size << endl;
            t = outputs[0];                   // 找tensor

            auto tmap = t.tensor<float, 2>();        // Tensor Shape: [batch_size, target_class_num]
            int output_dim = t.shape().dim_size(1);  // Get the target_class_num from 1st dimension
            for(int i = 0; i < size; ++i){
                // Argmax: Get Final Prediction Label and Probability
                int output_class_id = -1;
                double output_prob = -1;
                std::cout << "---------------------------------------------" << endl;
                std::cout << i << ":" << std::endl;
                for (int j = 0; j < output_dim; j++)
                {
                    std::cout << "Class " << j << " prob:" << tmap(0, j) << "," << std::endl;
                    if (tmap(i, j) >= output_prob) {
                        output_class_id = j;
                        output_prob = tmap(i, j);
                    }
                }
                if (output_class_id){
                    m_preTargets[seq_temp[i]].id = output_class_id;
                    m_targets.emplace_back(m_preTargets[seq_temp[i]]);
                }

            }
            m_is.addClassifiedTargets("After Classify", m_targets);
            DEBUG("m_classify end")
        }
        /**
         * @name m_match
         * @func 击打策略函数
         */ 
        emSendStatusA m_match()
        {
            /* 更新下相对帧编号 */
            for (auto iter = s_historyTargets.begin(); iter != s_historyTargets.end(); iter++)
            {
                iter->rTick++;
                /* 历史值数量大于30便删除末尾记录
                装甲板在被击打时会有短暂的一个时间不发光无法识别需要在不发光时保证瞄准状态不改变，数值/fps应在0.15s以上*/
                if (iter->rTick > 20)
                {
                    s_historyTargets.erase(iter, s_historyTargets.end());
                    break;
                }
            }
            /* 选择本次打击目标 */
            if (s_historyTargets.empty())
            {
                /* case A: 之前没选择过打击目标 */
                /* 选择数组中距离最近的目标作为击打目标 */
                auto minTarElement = std::min_element(
                    m_targets.begin(), m_targets.end(), [](Target &a_, Target &b_) -> bool {
                        return cv::norm(a_.ptsInGimbal) < cv::norm(b_.ptsInGimbal);     //找到离得最近的装甲板
                    });                                   //找到含最小元素的目标位置
                if (minTarElement != m_targets.end())
                {
                    s_historyTargets.emplace_front(*minTarElement);
                    PRINT_INFO("++++++++++++++++ 发现目标: 选择最近的 ++++++++++++++++++++\n");
                    return SEND_STATUS_AUTO_AIM;          //瞄准
                }
                else
                {
                    return SEND_STATUS_AUTO_NOT_FOUND;    //未找到
                }
            } // end case A
            else
            {
                /* case B: 之前选过打击目标了, 得找到一样的目标 */
                // PRINT_INFO("++++++++++++++++ 开始寻找上一次目标 ++++++++++++++++++++\n");
                double distance = 0xffffffff;

                int closestElementIndex = -1;
                for (size_t i = 0; i < m_targets.size(); ++i)
                {
                    /* 进行轮廓匹配，所得为0～1，数值越小越好*/
                    double distanceA = cv::matchShapes(m_targets[i].pixelPts2f, s_historyTargets[0].pixelPts2f,
                                                       3, 0.0);
                    /* 获取图像矩 */                                
                    cv::Moments m_1 = cv::moments(m_targets[i].pixelPts2f);
                    cv::Moments m_2 = cv::moments(s_historyTargets[0].pixelPts2f);   
                    // PRINT_WARN("distanceA = %f\n", distanceA);
                    /* 进行matchShaoes的阈值限定，并保证归一化中心矩同号
                    加了这一块有时候会把装甲板过滤掉，我也没整明白图像矩的原理，所以删掉了 */ 
                    // if (distanceA > 0.5 ||
                    //     (m_1.nu11 + m_1.nu30 + m_1.nu12) * (m_2.nu11 + m_2.nu30 + m_2.nu12) < 0)
                    //     continue;

                    double distanceB;
                    if (m_isEnablePredict)
                    {
                        /* 用绝对坐标距离计算 两次位置之差 */
                        distanceB = cv::norm(m_targets[i].ptsInWorld - s_historyTargets[0].ptsInWorld) / 2000.0;
                        // PRINT_WARN("distanceB = %f\n", distanceB);
                        /* 进行阈值判定 */
                        if (distanceB > 0.5)
                            continue;
                    }
                    else
                    {
                        /* 用云台坐标系距离计算 两次位置之差 */
                        distanceB = cv::norm(m_targets[i].ptsInGimbal - s_historyTargets[0].ptsInGimbal) / 3400.0;
                        // PRINT_WARN("distanceB = %f\n", distanceB);
                        /* 进行阈值判定 */
                        if (distanceB > 0.8)
                            continue;
                    }
                    double _distanceTemp = distanceA + distanceB / 2; 
                    /* 参数更正，保证当前图片存在 */
                    if (distance > _distanceTemp)
                    {
                        distance = _distanceTemp;
                        closestElementIndex = i;
                    }
                }
                if (closestElementIndex != -1)
                {
                    /* 找到了 */
                    s_historyTargets.emplace_front(m_targets[closestElementIndex]);
                    PRINT_INFO("++++++++++++++++ 找到上一次目标 ++++++++++++++++++++\n");
                    return SEND_STATUS_AUTO_AIM;           //瞄准
                }
                else
                {
                    PRINT_INFO("++++++++++++++++ 没找到上一次目标, 按上一次的来 ++++++++++++++++++++\n");
                    return SEND_STATUS_AUTO_AIM_FORMER;    //瞄准上一帧
                }
            } // end case B
            PRINT_ERROR("Something is NOT Handled in function m_match \n");
        }

    public:
        /**
         * @name enablePredict
         * @param enable = true: 开启
         * @func 设置是否开启预测
         */
        void enablePredict(bool enable = true)
        {
            // m_communicator.enableReceiveGlobalAngle(enable);
            m_isEnablePredict = enable;
        }

        /**
         * @name getBoundingRect
         * @param tar 上一个检测到的装甲
         * @param rect 截的图
         * @param size 采集图像参数
         * @param extendFlag 是否扩展
         * @func 图像扩展ROI
         */
        void getBoundingRect(Target &tar, cv::Rect &rect, cv::Size &size, bool extendFlag = false)
        {
            rect = cv::boundingRect(s_historyTargets[0].pixelPts2f_Ex);

            if (extendFlag)
            {
                rect.x -= int(rect.width * 4);
                rect.y -= rect.height * 3;
                rect.width *= 9;
                rect.height *= 7;

                rect.width = rect.width >= size.width ? size.width - 1 : rect.width;
                rect.height = rect.height >= size.height ? size.height - 1 : rect.height;

                rect.width = rect.width < 80 ? 80 : rect.width;
                rect.height = rect.height < 50 ? 50 : rect.height;

                rect.x = rect.x < 1 ? 1 : rect.x;
                rect.y = rect.y < 1 ? 1 : rect.y;
                int rect_yy=rect.y+rect.height;
                rect.width = rect.x + rect.width >= size.width ? size.width - 1 - rect.x : rect.width;
                // rect.height = rect.y + rect.height >= size.height ? size.height - 1 - rect.y : rect.height;

                int temp=rect.y;
                //因为ICRA比赛机器人只在水平面内运行，所以只检测图像下半部分
                rect.y =rect.y>210?rect.y:210;
                rect.height=rect_yy-rect.y;
                rect.height = rect.y + rect.height >= size.height ? size.height - 1 - rect.y : rect.height;
            }
        }

        void gimbal_excute(ros::Publisher& gimbalPub,double pitch,double yaw)
        {
            //云台执行函数，已经弃用，云台控制统一由决策节点完成
            roborts_msgs::GimbalAngle gimbalAngle;
            gimbalAngle.yaw_mode = 1;
            gimbalAngle.pitch_mode = 0;
            gimbalAngle.yaw_angle = yaw;
            gimbalAngle.pitch_angle = pitch;
            gimbalPub.publish(gimbalAngle);
        }

        //获取机器人云台和底盘夹角
        void get_gimbal(double& pitch,double& yaw)
        {
            
            tf::StampedTransform transform;
            try{
            listener->lookupTransform("/base_link", "/gimbal",
                                    ros::Time(0), transform);
            double roll;
            //四元数转欧拉角
            tf::Matrix3x3(transform.getRotation()).getRPY(roll,pitch,yaw);
            yaw = yaw/3.14159265*180.0;
            pitch = pitch/3.14159265*180.0;
            }
            catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            return ;
            }
        }

    void Matrix2Euler(Target& tar)
    {
        double roll = atan2(tar.rvMat.at<double>(2,1),tar.rvMat.at<double>(2,2));
        double pitch = atan2(-tar.rvMat.at<double>(2,0),sqrt(pow(tar.rvMat.at<double>(2,2),2)+pow(tar.rvMat.at<double>(2,1),2)));
        double yaw = atan2(tar.rvMat.at<double>(1,0),tar.rvMat.at<double>(0,0));
        tar.relativeYaw = pitch;
        // std::cout<<"roll:"<<roll<<"   "<<"pitch:"<<pitch<<"yaw:"<<yaw<<std::endl;
    }

    //卡尔曼滤波切换目标条件
    bool ChangeCondition(std::deque<Target>& targets)
    {
        if(targets.size()<2)
        {
            return true;
        }
        else
        {   //在两次装甲板的夹角大于pi/4时证明瞄准的不是同一块装甲板而是机器人旋转漏出了另一块装甲板
            if(cv::abs(targets[0].relativeYaw - targets[1].relativeYaw)>M_PI/4)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
    }


        /**
         * @name run
         * @param src 彩图
         * @param timeStamp 时间戳
         * @param gYaw 从电控获得yaw
         * @param gPitch 从电控获得pitch
         * @param pmode 处理模式，0为只使用opencv，1为使用卷积神经网络
         * @func 主运行函数
         * @return true
         */
        bool run(cv::Mat &src,const ros::Time& image_timeStamp, double gYaw, double gPitch,image_transport::Publisher& resultPub,ros::Publisher& gimbalPub, ros::Publisher& messpub,int pmode = 1)
        {
            /* 1.初始化参数，判断是否启用ROI */
            m_bgr_raw = src;
            m_bgr = src;
            int64_t timeStamp = image_timeStamp.toSec()*secmult;   //时间戳是机器人采集到图像时生成的时间戳
            m_currentTimeStamp = timeStamp;
            m_targets.clear();      //m_targets只储存本次监测到的对象，因为每调用一次，就会清零
            m_preTargets.clear();
            m_startPt = cv::Point(0, 0);
            if (s_historyTargets.size() >= 2 && s_historyTargets[0].rTick <= 10)
            {
                cv::Rect latestShootRect;
                getBoundingRect(s_historyTargets[0], latestShootRect, stFrameInfo.size, true);
                m_is.addEvent("Bounding Rect", latestShootRect);
                m_bgr = m_bgr(latestShootRect);
                m_startPt = latestShootRect.tl(); //tl返回左上角顶点的坐标
                // cv::namedWindow("m_bgr");
                // cv::imshow("m_bgr",m_bgr);
                // cv::waitKey(5);
            }

            /* 2.检测+分类 */
            /* 若为模式0，则只使用opencv进行处理 */
            ros::Time pre = ros::Time::now();
            if(pmode == 0)
            {
                m_is.clock("m_classify");
                m_preDetect(pmode);
                m_is.clock("m_classify");
            }
            else if(pmode == 1)
            {
                m_preDetect(pmode);
                m_is.clock("m_classify");
                m_classify_single_tensor(1); 
                m_is.clock("m_classify");
            }


            s_latestTimeStamp.exchange(timeStamp);
            double rYaw = 0.0;
            double rPitch = 0.0;
            /* 获得云台全局欧拉角 */
            // m_communicator.getGlobalAngle(&gYaw, &gPitch);
            /* 计算世界坐标参数，转换到世界坐标系 */
            for (auto &tar : m_targets)
            {
                tar.calcWorldParams();
                tar.convert2WorldPts(-gYaw, gPitch);
            }

            /* 3.目标匹配 */
            emSendStatusA statusA = m_match();
            DEBUG("m_match end")
            if (!s_historyTargets.empty())
            {
                m_is.addFinalTargets("selected", s_historyTargets[0]);
                Matrix2Euler(s_historyTargets[0]);
                /* 5.预测部分 */
                // ros::Time pretime2=ros::Time::now();
                if (m_isEnablePredict)
                {
                    if (statusA == SEND_STATUS_AUTO_AIM)
                    {   /* 获取世界坐标点 */
                        /* 转换为云台坐标点 */
                        get_gimbal(gPitch,gYaw);
                        // m_communicator.getGlobalAngle(&gYaw, &gPitch);
                        s_historyTargets[0].convert2WorldPts(-gYaw, gPitch);
                        // cout << "s_historyTargets[0].ptsInWorld : " << s_historyTargets[0].ptsInWorld << endl;
                        /* 卡尔曼滤波初始化/参数修正 */
                        if (s_historyTargets.size() == 1||ChangeCondition(s_historyTargets))
                        {
                            std::cout<<"RESET"<<std::endl;
                            /*由于在扭腰的时候机器人的底盘坐标系一直在变化，所以如果想要做卡尔曼滤波需要保证将所有的目标统一到一个坐标系之下
                            因此将卡尔曼滤波初始化时候的坐标系设为稳定的坐标系，其他每一时刻的观测结果都转换到这个坐标系下*/
                            odom_buff->reset_standard();
                            odom_buff->pose_judge(s_historyTargets[0].ptsInWorld, image_timeStamp);
                            kalman.clear_and_init(s_historyTargets[0].ptsInWorld, timeStamp);
                        }
                        else
                        {
                            
                            odom_buff->pose_judge(s_historyTargets[0].ptsInWorld, image_timeStamp);
                            kalman.correct(s_historyTargets[0].ptsInWorld, timeStamp);
                        }
                    }
                    m_is.addText(cv::format("inWorld.x %.0f", s_historyTargets[0].ptsInWorld.x));
                    m_is.addText(cv::format("inWorld.y %.0f", s_historyTargets[0].ptsInWorld.y));
                    m_is.addText(cv::format("inWorld.z %.0f", s_historyTargets[0].ptsInWorld.z));
                    // fs<<ros::Time::now()<<","<<s_historyTargets[0].ptsInWorld.x<<","<<s_historyTargets[0].ptsInWorld.y
                    // <<","<<s_historyTargets[0].ptsInWorld.z<<std::endl;
                    /* 进行预测和坐标修正 */
                    if (s_historyTargets.size() > 1)
                    {
                        kalman.predict(0.1, s_historyTargets[0].ptsInWorld_Predict);
                        //得到了预测结果之后将坐标再转换回原来的坐标系
                        odom_buff->transback(s_historyTargets[0].ptsInWorld_Predict);
                        odom_buff->transback(kalman.velocity);
                        /* 转换为云台坐标点 */
                        s_historyTargets[0].convert2GimbalPts(kalman.velocity);
                        //不知道神奇的卡尔曼滤波输出的速度单位是分米/s，所以需要做转换
                        double vx = s_historyTargets[0].vInGimbal3d.x*100;   //mm/s
                        m_is.addText(cv::format("vx %4.0f", s_historyTargets[0].vInGimbal3d.x*100));
                        m_is.addText(cv::format("vy %4.0f", cv::abs(s_historyTargets[0].vInGimbal3d.y*100)));
                        m_is.addText(cv::format("vz %4.0f", cv::abs(s_historyTargets[0].vInGimbal3d.z*100)));
                        if (cv::abs(s_historyTargets[0].vInGimbal3d.x) > 6)   //0.6m/s
                        {
                            //0.1s好像是不太够，ICRA2020的车在值为0.2的时候能对移动目标进行比较稳定的打击
                            double deltaX = vx*0.1;    //dm/s->mm/s->m(delay=0.1)
                            deltaX = deltaX > 200 ? 200 : deltaX;
                            s_historyTargets[0].ptsInGimbal.x += deltaX;

                        }
                    }
                }

                if(statusA == SEND_STATUS_AUTO_AIM)
                {
                    roborts_msgs::visual_detection vision_data;
                    vision_data.if_enemy = true;
                    s_historyTargets[0].convert2WorldPts(-gYaw, gPitch);
                    // m_is.addText(cv::format("inWorld.x %.0f", s_historyTargets[0].ptsInWorld.x));
                    // m_is.addText(cv::format("inWorld.y %.0f", s_historyTargets[0].ptsInWorld.y));
                    // m_is.addText(cv::format("inWorld.z %.0f", s_historyTargets[0].ptsInWorld.z));
                    /* 5.修正弹道并计算欧拉角 */
                    s_historyTargets[0].correctTrajectory_and_calcEuler();
                    rYaw = s_historyTargets[0].rYaw;
                    rYaw=-rYaw;
                    rPitch = s_historyTargets[0].rPitch;
                    /* 6.射击策略 */
                    if (s_historyTargets.size() >= 3 &&
                        cv::abs(s_historyTargets[0].ptsInGimbal.x) < 100.0)
                    {
                        vision_data.if_shoot=true;   //因为相机固定在yaw轴上，因此仅考虑x方向的瞄准情况
                    }
                    m_is.addText(cv::format("ptsInGimbal: %2.3f %2.3f %2.3f",
                                                    s_historyTargets[0].ptsInGimbal.x / 1000.0,
                                                    s_historyTargets[0].ptsInGimbal.y / 1000.0,
                                                    s_historyTargets[0].ptsInGimbal.z / 1000.0));
                            m_is.addText(cv::format("rPitch %.3f", rPitch));
                            m_is.addText(cv::format("rYaw   %.3f", rYaw* M_PI / (180.0)));
                            m_is.addText(cv::format("gYaw   %.3f", gYaw* M_PI / (180.0)));
                            m_is.addText(cv::format("rYaw + gYaw   %.3f", (rYaw + gYaw)* M_PI / (180.0)));

                    rYaw=rYaw* M_PI / (180.0);
                    gYaw=gYaw* M_PI / (180.0);

                    //在获取绝对角度的时候应该考虑的是采集图像那一刻机器云台所处的角度，因此需要回看
                    double gYaw_pred = imu_buff->getPredYaw(timeStamp);
                    double latestYaw = imu_buff->getLatestYaw();
                    float send_Yaw =gYaw_pred+rYaw;

                    // if(cv::abs(newYaw - gYaw)>0.1)

                    vision_data.G_angle.yaw_angle=send_Yaw;
                    vision_data.G_angle.pitch_angle=rPitch;
                    std::vector<roborts_msgs::single_car> temp;

                    roborts_msgs::single_car mycar;     //第一个目标要射的
                    mycar.pose.pose.orientation.x=0;
                    mycar.pose.pose.orientation.y=0;
                    mycar.pose.pose.orientation.z=0;
                    mycar.pose.pose.orientation.w=1;
                    mycar.pose.header.seq=cur_frame++;
                    mycar.pose.header.stamp=image_timeStamp;
                    mycar.pose.header.frame_id = "/base_link";
                    mycar.pose.pose.position.x=s_historyTargets[0].ptsInWorld.x;
                    mycar.pose.pose.position.y=s_historyTargets[0].ptsInWorld.y;
                    mycar.pose.pose.position.z=s_historyTargets[0].ptsInWorld.z;
                    mycar.id = s_historyTargets[0].id;
                    mycar.color = mode+1;    //红蓝模式,yaml文件中读出
                    temp.emplace_back(mycar);
                    //先检查本次target中有无与已瞄id不同的id
                    Target* minother = nullptr;
                    for(auto& _tar : m_targets){
                        if(_tar.id != mycar.id){
                            if(minother == nullptr){
                                minother = &_tar;
                            }
                            else{
                                if(cv::norm(_tar.ptsInGimbal) < cv::norm(minother->ptsInGimbal))
                                    minother = &_tar;
                            }
                        }
                    }
                    if(minother != nullptr){
                        //其他都不变，只需要改一下位姿和id即可
                        mycar.pose.pose.position.x=minother->ptsInWorld.x;
                        mycar.pose.pose.position.y=minother->ptsInWorld.y;
                        mycar.pose.pose.position.z=minother->ptsInWorld.z;
                        mycar.id = minother->id;
                        temp.emplace_back(mycar);
                        }
                    vision_data.multicar = temp;
                    last_vision_data = vision_data;
                 }
                 else{
                     // 在一个时间段内保持之前的打击状态
                 }
            }
            else{
                roborts_msgs::visual_detection vision_data;
                vision_data.if_shoot = false;
                vision_data.if_enemy = false;
                last_vision_data = vision_data;
            }

            //把图像发出来
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", m_is.getFrame()).toImageMsg();
            resultPub.publish(*msg);
            messpub.publish(last_vision_data);
            return true;
        }
    };
} 
#endif
