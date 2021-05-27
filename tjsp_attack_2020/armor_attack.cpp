#include "attack.hpp"
#include "imageshow.hpp"
#include "base.hpp"
#include "ros/ros.h"
#include <image_transport/image_transport.h>

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
            attack.setMode(armor::stConfig.get<std::string>("attack.attack-color") == "blue");//击打颜色为红色，具体见配置文件
            attackPtr = &attack;
            ros::NodeHandle n;
            ros::NodeHandle messnode;
            image_transport::ImageTransport it(n);
            startTime = ros::Time::now();
            ros::Subscriber image_sub = n.subscribe("/cam1/image_raw",1,&rosDetect::imageCB,this);  //收到摄像头的消息，就回调该函数
                                                                                                    //感觉迷你PC算力真是强大
            ros::Subscriber cameraInfo_sub = n.subscribe("/cam1/camera_info",1,&rosDetect::cameraInfoCB,this);
            resultPub = it.advertise("detection",1);
            gimbalPub = n.advertise<roborts_msgs::GimbalAngle>("/cmd_gimbal_angle",1);
            //messpub = messnode.advertise<roborts_msgs::test>("roborts_all",1);
            messpub = messnode.advertise<roborts_msgs::visual_detection>("roborts_all",1);//定义
            ros::service::waitForService("/classify");
            img_client = n.serviceClient<tjsp_attack_2020::img>("/classify");
            listener = new tf::TransformListener;
            ros::spin();
        }
 

        void imageCB(const sensor_msgs::Image::ConstPtr& msg)
        {
            ros::Time start = ros::Time::now();
            cv_bridge::CvImagePtr Image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
            int64_t timestamp = start.toSec()*pow(10,6);
            isClientPtr->update(Image->image, int(timestamp / 1000));
            double pitch,yaw;
            get_gimbal(pitch,yaw);
            if(!armor::stCamera.cameraInfo_set){
                return;
            }
            attackPtr->run(Image->image,msg->header.stamp,yaw,pitch,resultPub,gimbalPub,messpub,img_client,1);
                // std::cout<<"use_time:"<<ros::Time::now()-start<<std::endl;
        }

        void get_gimbal(double& pitch,double& yaw)
        {
            
            tf::StampedTransform transform;
            try{
            listener->lookupTransform("/base_link", "/gimbal",
                                    ros::Time(0), transform);
            double roll;

            tf::Matrix3x3(transform.getRotation()).getRPY(roll,pitch,yaw);
            yaw = yaw/3.14159265*180.0;//弧度转角度
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
        image_transport::Publisher resultPub;
        ros::Publisher gimbalPub;
        ros::Publisher messpub;//声明
        tf::TransformListener* listener;
        ros::ServiceClient img_client;
};







int main(int argc,char **argv)
{
    ros::init(argc,argv,"detect");
    std::cout << "Using OpenCV " << CV_VERSION << std::endl;
    rosDetect a = rosDetect();
    return 0;
}
