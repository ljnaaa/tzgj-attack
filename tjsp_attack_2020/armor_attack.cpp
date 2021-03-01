#include "attack.hpp"
#include "imageshow.hpp"
#include "base.hpp"
#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/Image.h"

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
            ros::Subscriber sub = n.subscribe("/cam0/image_raw",1,&rosDetect::imageCB,this);
            resultPub = n.advertise<sensor_msgs::Image>("/cam0/result",1);

            ros::spin();
        }

        void imageCB(const sensor_msgs::Image::ConstPtr& msg)
        {
            ros::Time start = ros::Time::now();
            cv_bridge::CvImagePtr Image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
            int64_t timestamp = ros::Time::now().toSec()*pow(10,6);
            isClientPtr->update(Image->image, int(timestamp / 1000));
            attackPtr->run(Image->image,timestamp,0.0,0.0,resultPub);
            std::cout<<"Using:"<<ros::Time::now()-start<<std::endl;
        }
    
    private:
        
        armor::ImageShowClient* isClientPtr;
        armor::Attack* attackPtr;
        armor::PID* pidPtr;
        ros::Time startTime;
        ros::Publisher resultPub;


};







int main(int argc,char **argv)
{
    ros::init(argc,argv,"detect");
    std::cout << "Using OpenCV " << CV_VERSION << std::endl;
    rosDetect a = rosDetect();
    return 0;







}