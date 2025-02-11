#include <opencv2/opencv.hpp>
#include "RTSPcam/RTSPcam.h"
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

using namespace std::chrono_literals;

static const rmw_qos_profile_t custom_rmw_qos_profile_sensor_data =
{
  RMW_QOS_POLICY_HISTORY_KEEP_LAST,
  30,
  RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false
};

rmw_qos_profile_t qos_profile_sensor = custom_rmw_qos_profile_sensor_data;

auto qos_sensor = rclcpp::QoS(
    rclcpp::QoSInitialization(
      qos_profile_sensor.history,
      qos_profile_sensor.depth
    ),  
    qos_profile_sensor);

bool camopen_flag = false;
bool frame_availability = false;
const static std::string ANAFI_STREAM_WINDOW = "Anafi_RTSP_Video";
const static std::string rtsp_url = "rtsp://10.202.0.1:554/live"; //"rtsp://192.168.53.1/live"
cv::Mat frame;
cv::Mat backup_frame;
RTSPcam cam;
std::shared_ptr<sensor_msgs::msg::Image> rtsp_frame;

class Anafi_RTSP_Stream_Video : public rclcpp::Node
{
    public:
    Anafi_RTSP_Stream_Video()
    : Node("anafi_rtsp_stream_video")
        {
            RCLCPP_INFO(this->get_logger(), "\n>>>>>> NODE anafi_rtsp_stream_video initiated <<<<<<\n");
            rtspvideopub_ = this->create_publisher<sensor_msgs::msg::Image>("anafi_rtsp_video", qos_sensor);
            rtspvideotimer_ = this->create_wall_timer(0.03333s, std::bind(&Anafi_RTSP_Stream_Video::rstp_video_stream, this));
        }
    
    private:
        void rstp_video_stream()
        {
            if (camopen_flag == false)
            {
                cv::namedWindow(ANAFI_STREAM_WINDOW, cv::WINDOW_AUTOSIZE);
                //you can dump anything OpenCV eats. (cv::CAP_ANY) BTW,OpenCV first tries FFmpeg
                RCLCPP_INFO(this->get_logger(), "Connecting to: %s", rtsp_url.c_str());
                cam.Open(rtsp_url); 
                camopen_flag = true;
            }

            else {}
            
            if(!cam.GetLatestFrame(frame))
            {
                if (frame_availability == true)
                {
                    RCLCPP_WARN(this->get_logger(), "Frame lost, utilizing last backed up frame");
                    rtsp_frame = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", backup_frame).toImageMsg();
                    rtspvideopub_->publish(*rtsp_frame.get());
                    cv::imshow(ANAFI_STREAM_WINDOW, backup_frame);
                }

                else 
                {
                    RCLCPP_ERROR(this->get_logger(), "Capture read error"); 
                }
            }

            else
            {
                backup_frame = frame.clone();
                rtsp_frame = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
                rtspvideopub_->publish(*rtsp_frame.get());
                cv::imshow(ANAFI_STREAM_WINDOW, frame);
                frame_availability = true;
            }

            cv::waitKey(1);
        }

        rclcpp::TimerBase::SharedPtr rtspvideotimer_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rtspvideopub_;
};

int main(int argc, char * argv[])
{
    try
    {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<Anafi_RTSP_Stream_Video>());
        rclcpp::shutdown();
    }

    catch (const std::exception &e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Exception %s", e.what());
        return EXIT_FAILURE;
    }
    
    cv::destroyAllWindows();
    return EXIT_SUCCESS;
}