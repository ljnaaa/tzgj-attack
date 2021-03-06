#include "attack.hpp"
#include "imageshow.hpp"
#include "base.hpp"
#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <vector>
#include <boost/array.hpp>
class rosDetect
{
    public:
        rosDetect()
        {
            // /* 开图像显示辅助程序 */

            armor::PID pid;
            pid.init(armor::stConfig.get<double>("auto.kp"),
                    armor::stConfig.get<double>("auto.ki"),
                    armor::stConfig.get<double>("auto.kd"));

            pidPtr = &pid;
            
            // armor::ImageShowClient isClient = isServer.getClient(0);
            isClientPtr = new armor::ImageShowClient();
            isClientPtr->setClientId(0);
            armor::ImageShowClient isClient = *isClientPtr;

            armor::stFrameInfo.size = cv::Size(640, 512);
            armor::stFrameInfo.offset = cv::Point2i(0, 0);

            armor::Attack attack(*isClientPtr,pid);
            attack.enablePredict(armor::stConfig.get<bool>("auto.enable-predict"));//是否进行预测
            attack.setMode(armor::stConfig.get<std::string>("attack.attack-color") == "red");//击打颜色为红色，具体见配置文件
            attackPtr = &attack;
            ros::NodeHandle n;
            startTime = ros::Time::now();
            ros::Subscriber image_sub = n.subscribe("/cam0/image_raw",1,&rosDetect::imageCB,this);
            ros::Subscriber cameraInfo_sub = n.subscribe("/cam0/camera_info",1,&rosDetect::cameraInfoCB,this);
            resultPub = n.advertise<sensor_msgs::Image>("/cam0/result",1);
            gimbalPub = n.advertise<roborts_msgs::GimbalAngle>("/cmd_gimbal_angle",1);
            listener = new tf::TransformListener;
            ros::spin();
        }
 

        void imageCB(const sensor_msgs::Image::ConstPtr& msg)
        {
            ros::Time start = ros::Time::now();
            cv_bridge::CvImagePtr Image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
            int64_t timestamp = ros::Time::now().toSec()*pow(10,6);

            isClientPtr->update(Image->image, int(timestamp / 1000));
            double pitch,yaw;
            get_gimbal(pitch,yaw);

            attackPtr->run(Image->image,timestamp,yaw,pitch,resultPub,gimbalPub);
        }

        void get_gimbal(double& pitch,double& yaw)
        {
            
            tf::StampedTransform transform;
            try{
            listener->lookupTransform("/base_link", "/gimbal",
                                    ros::Time(0), transform);
            double roll;

            tf::Matrix3x3(transform.getRotation()).getRPY(roll,pitch,yaw);
            yaw = yaw/3.14159265*180.0;
            pitch = pitch/3.14159265*180.0;
            }

            catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            return ;
            }
        }


        void cameraInfoCB(const sensor_msgs::CameraInfo::ConstPtr& msg)
        {
            // boost::array<double,9> listmsgk;
            // for(int i=0;i<9;i++){
            //     listmsgk[i]=msg->K[i];
            // }
            // for(int i=0;i<9;i++){
            //     std::cout<<listmsgk[i]<<' ';
            // }
            // std::cout<<std::endl;
            armor::stCamera.SetCameraInfo(msg->K,msg->D);
        }





    
    private:
        armor::ImageShowClient* isClientPtr;
        armor::Attack* attackPtr;
        armor::PID* pidPtr;
        ros::Time startTime;
        ros::Publisher resultPub;
        ros::Publisher gimbalPub;
        tf::TransformListener* listener;

};







int main(int argc,char **argv)
{
    ros::init(argc,argv,"detect");
    std::cout << "Using OpenCV " << CV_VERSION << std::endl;
    rosDetect a = rosDetect();
    return 0;







}