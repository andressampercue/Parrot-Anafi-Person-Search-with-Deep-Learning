#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include "cv_bridge/cv_bridge.h"
#include "yolov8/cmd_line_util.h"
#include "yolov8/yolov8.h"
#include <opencv2/cudaimgproc.hpp>
#include "anafi_person_search_interfaces/srv/face_key_points.hpp"
#include "anafi_ros_interfaces/srv/fp_start.hpp"
#include "anafi_ros_interfaces/srv/fp_pause_stop.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace std::this_thread;

static const rmw_qos_profile_t custom_rmw_qos_profile_default =
{
  RMW_QOS_POLICY_HISTORY_KEEP_LAST,
  10,
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false
};

rmw_qos_profile_t qos_profile_default = custom_rmw_qos_profile_default;

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

static const std::string OPENCV_WINDOW = "Anafi_Camera";

YoloV8Config config;
std::string onnxModelPath = "/home/carlos/ros2_ws/src/anafi_person_search/anafi_person_search_nodes/models/yolov8x-pose-p6.onnx";

// Create the YoloV8 engine
YoloV8 yoloV8(onnxModelPath, config);

std::map<int, std::string> landmarks_names =
{
    {0, "Nose"},
    {1, "Left-eye"},
    {2, "Right-eye"},
    {3, "Left-ear"},
    {4, "Right-ear"},
    {5, "Left-shoulder"},
    {6, "Right-shoulder"},
    {7, "Left-elbow"},
    {8, "Right-elbow"},
    {9, "Left-wrist"},
    {10, "Right-wrist"},
    {11, "Left-hip"},
    {12, "Right-hip"},
    {13, "Left-knee"},
    {14, "Right-knee"},
    {15, "Left-ankle"},
    {16, "Right-ankle"}
};

std::map<int, std::string> fp_states = 
{
    {0, "PLAYING"}, 
    {1, "STOPPED"}, 
    {2, "PAUSED"}, 
    {3, "LOADED"}, 
    {4, "TIMED_OUT"}, 
    {5, "NO_STATE"}
};

std::map<int, std::string> fp_states_log = 
{
    {0, "PLAYING: Mavlink file is playing (0)"},
    {1, "STOPPED: Mavlink file is stopped (arg filepath and type are useless in this state) (1)"},
    {2, "PAUSED: Mavlink file is paused (2)"},
    {3, "LOADED: Mavlink file is loaded (it will be played at take-off) (3)"},
    {4, "TIMED_OUT: Request for pausing Mavlink file timed out (4)"},
	{5, "NO_STATE: Mavlink file with no state (5)"}
};

std::map<int, std::string> mavlink_types = 
{
    {0, "FLIGHTPLAN"}, 
    {1, "MAPMYHOUSE"}, 
    {2, "FLIGHTPLANV2"}, 
    {3, "UNKNOWN"}
};

std::map<int, std::string> mavlink_types_log = 
{
    {0, "FLIGHTPLAN: Mavlink file for FlightPlan (0)"},
    {1, "MAPMYHOUSE: Mavlink file for MapMyHouse (1)"},
    {2, "FLIGHTPLANV2: Mavlink file for FlightPlan V2 (better follow the standard) (2)"},
    {3, "UNKNOWN: Mavlink file type unknown (3)"}
};

bool image_metadata = false;
bool person_detected = false;
bool nose_available = false;
bool extrighthip_available = false;
bool extlefthip_available = false;
bool fppause_executed = false;
bool stream_latency_trigger = false;
int frame_counter = 0;
int firstperdetected_frame = 0;

class Anafi_Person_Detect : public rclcpp::Node
{
    public:
        Anafi_Person_Detect()
        : Node("anafi_person_detect")
        {
            RCLCPP_INFO(this->get_logger(), "\033[32m\n>>>>>> NODE anafi_person_detect initiated <<<<<<\n");

            anafiimg_ = this->create_subscription<sensor_msgs::msg::Image>("/anafi/camera/image", qos_sensor, std::bind(&Anafi_Person_Detect::camera_subs, this, _1));
            
            facepointscli_ = this->create_client<anafi_person_search_interfaces::srv::FaceKeyPoints>("key_face_points", qos_profile_default);
            
            clifppause_ = this->create_client<anafi_ros_interfaces::srv::FpPauseStop>("/anafi/flightplan/pause", qos_profile_default);
            
            clipfcontinue_ = this->create_client<anafi_ros_interfaces::srv::FpStart>("/anafi/flightplan/start", qos_profile_default);

            cv::namedWindow(OPENCV_WINDOW, cv::WINDOW_AUTOSIZE);
            std::map<std::string, uint> yolov8_pose_estimation(cv::Mat &img_Mat);
        }
    
    private:
        void camera_subs(const sensor_msgs::msg::Image::SharedPtr msg) 
        {
            cv_bridge::CvImagePtr cv_ptr;
            //std::cout << "Image width: " << msg->width << ", height: " << msg->height << std::endl;
            //std::cout << "Encoding: " << msg->encoding << std::endl;
            try
            {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
                if (cv_ptr->image.empty()) 
                {
                    std::cerr << "Error: Image is empty." << std::endl;
                    return;
                }
                
                if (image_metadata == false)
                {
                    RCLCPP_INFO(this->get_logger(), "\033[32mImage width: %d, height: %d\n", cv_ptr->image.cols, cv_ptr->image.rows);
                    image_metadata = true;
                }

                else {}
                
                cv::Mat img_Mat = cv_ptr->image;
                int brightness_offset = 20;
                img_Mat.convertTo(img_Mat, -1, 1, -brightness_offset);
                cv::Mat kernel = (cv::Mat_<float>(3, 3) << 0, -1, 0, 
                                                          -1, 5, -1, 
                                                           0, -1, 0);
                cv::filter2D(img_Mat, img_Mat, img_Mat.depth(), kernel);
                //Resize frame 
                //cv::resize(img, img, cv::Size(1280, 720), 0, 0, cv::INTER_AREA);
                std::map<std::string, uint> face_kps = yolov8_pose_estimation(img_Mat);
                
                if (stream_latency_trigger == true && (frame_counter > (firstperdetected_frame + 90)))
                {
                    if ((face_kps["nx"] != 0 || face_kps["ny"] != 0 || face_kps["cx"] != 0 || face_kps["cy"]) &&
                    (fppause_executed == true && nose_available == true && extlefthip_available == true && extlefthip_available == true))
                    {
                        auto request_face_kps = std::make_shared<anafi_person_search_interfaces::srv::FaceKeyPoints::Request>();
                        request_face_kps->nx = face_kps["nx"];
                        request_face_kps->ny = face_kps["ny"];
                        request_face_kps->cx = face_kps["cx"];
                        request_face_kps->cy = face_kps["cy"];
                        auto response_callback = [this](rclcpp::Client<anafi_person_search_interfaces::srv::FaceKeyPoints>::SharedFuture future) 
                        {
                            try 
                            {
                                RCLCPP_INFO(this->get_logger(), "\033[32mFace Kps Callback invoked");
                                auto response = future.get();
                                if (response == nullptr) 
                                {
                                    RCLCPP_ERROR(this->get_logger(), "Received null response");
                                } 
                                
                                else 
                                {
                                    RCLCPP_INFO(this->get_logger(), "\033[32m[NODE] -> anafi_face_reubication said: %s", response->reception_state.c_str());
                                }
                            }

                            catch (const std::exception &e) 
                            {
                                RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
                            }
                        };

                        while (!facepointscli_->wait_for_service(1s))
                        {
                            if (!rclcpp::ok())
                            {
                                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service. Exiting.");
                                exit(EXIT_FAILURE);
                            }
                            RCLCPP_INFO(this->get_logger(), "\033[32mService not available, waiting again...");
                        }
                        
                        RCLCPP_INFO(this->get_logger(), "\033[32mSending async request to service for facepointscli_");
                        facepointscli_->async_send_request(request_face_kps, response_callback);
                        fppause_executed = false;
                        nose_available = false;
                        extlefthip_available = false;
                        extrighthip_available = false;
                    }

                    else {}

                    // writing the image to a defined location as JPEG
                    bool check = cv::imwrite("/home/carlos/2nd_person_frame.jpg", img_Mat);

                    // if the image is not saved
                    if (check == false) 
                    {
                        RCLCPP_ERROR(this->get_logger(), "Saving the image FAILED");
                    }

                    else 
                    {
                        RCLCPP_INFO(this->get_logger(), "\033[32mSuccessfully saved 2nd_person_frame.jpg");
                    }
                }

                else 
                {
                    RCLCPP_INFO(this->get_logger(), "\033[32mManaging stream latency");
                }
                
                cv::imshow(OPENCV_WINDOW, img_Mat);
                cv::waitKey(30);
                RCLCPP_INFO(this->get_logger(), "\033[32mFrame #: %d", frame_counter);
                frame_counter += 1;
            }

            catch (cv_bridge::Exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                rclcpp::shutdown();
                return;
            }
        }

        std::map<std::string, uint> yolov8_pose_estimation(cv::Mat &img_Mat)
        {
            uint person_counter = 0;
            uint pose_landmarks = 17;
            uint lm_vect_pos = 0;
            uint lefthip_kpx = 0;
            uint lefthip_kpy = 0;
            uint righthip_kpx = 0;
            uint righthip_kpy = 0;
            std::map<std::string, uint> face_kps;

            // Run inference
            const auto objects = yoloV8.detectObjects(img_Mat);

            for (size_t obj = 0; obj < objects.size(); obj++)
            {
                if (objects[obj].label == 0)
                {
                    if (person_detected == false)
                    {
                        RCLCPP_INFO(this->get_logger(), "\033[32m--- Person Idx: %d", person_counter);
                        RCLCPP_INFO(this->get_logger(), "\033[32mProbability: %lf", objects[obj].probability);
                        RCLCPP_INFO(this->get_logger(), "\033[32mx0: %d", int(objects[obj].rect.x));
                        RCLCPP_INFO(this->get_logger(), "\033[32my0: %d", int(objects[obj].rect.y)); 
                        RCLCPP_INFO(this->get_logger(), "\033[32mx1: %d", int((objects[obj].rect.width + objects[obj].rect.x))); 
                        RCLCPP_INFO(this->get_logger(), "\033[32my1: %d\n", int((objects[obj].rect.height + objects[obj].rect.y)));  
                        for (uint landmark = 0; landmark < pose_landmarks; landmark++)
                        {
                            RCLCPP_INFO(this->get_logger(), "\033[32m%s -> x: %d, y: %d", landmarks_names[landmark].c_str(), int(round(objects[obj].kps[lm_vect_pos])), 
                                        int(round(objects[obj].kps[lm_vect_pos+1])));
                            if (landmarks_names[landmark] == "Nose")
                            {
                                face_kps["nx"] = round(objects[obj].kps[lm_vect_pos]);
                                face_kps["ny"] = round(objects[obj].kps[lm_vect_pos+1]);
                                nose_available = true;
                            }

                            else if (landmarks_names[landmark] == "Left-hip")
                            {
                                lefthip_kpx = round(objects[obj].kps[lm_vect_pos]);
                                lefthip_kpy = round(objects[obj].kps[lm_vect_pos+1]);
                                extlefthip_available = true;
                            }

                            else if (landmarks_names[landmark] == "Right-hip")
                            {
                                righthip_kpx = round(objects[obj].kps[lm_vect_pos]);
                                righthip_kpy = round(objects[obj].kps[lm_vect_pos+1]);
                                extrighthip_available = true;
                            }

                            lm_vect_pos += 3; 
                        }

                        if (extlefthip_available == true && extrighthip_available == true)
                        {
                            face_kps["cx"] = round((lefthip_kpx + righthip_kpx)/2);
                            face_kps["cy"] = round((lefthip_kpy + righthip_kpy)/2);
                        }

                        else {}

                        if (fppause_executed == false && nose_available == true && extlefthip_available == true && extrighthip_available == true)
                        {
                            auto response_clifppause_callback = [this](rclcpp::Client<anafi_ros_interfaces::srv::FpPauseStop>::SharedFuture future)
                            {
                                try
                                {
                                    RCLCPP_INFO(this->get_logger(), "\033[32mCallback invoked");
                                    auto response = future.get();
                                    if (response == nullptr)
                                    {
                                        RCLCPP_ERROR(this->get_logger(), "\033[32mReceived null response");
                                    }

                                    else
                                    {
                                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[32m[NODE] -> anafi_flightplan_action state: %s", fp_states_log[response->state].c_str());
                                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[32m[NODE] -> anafi_flightplan_action file path: %s", (response->filepath).c_str());
                                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[32m[NODE] -> anafi_flightplan_action type: %s", mavlink_types_log[response->type].c_str());
                                        if (fp_states[response->state] == "PAUSED")
                                        {
                                            fppause_executed = true;
                                            nose_available = false;
                                            extlefthip_available = false;
                                            extrighthip_available = false;
                                            person_detected = false;
                                            stream_latency_trigger = true;
                                            firstperdetected_frame = frame_counter;
                                        }

                                        else
                                        {
                                            RCLCPP_ERROR(this->get_logger(), "FP was not paused, impossible to continue with rest of protocol");
                                        }
                                    }
                                }

                                catch (const std::exception &e)
                                {
                                    RCLCPP_ERROR(this->get_logger(), "Failed call failed: %s", e.what());
                                }
                            };
                            
                            auto request_clifppause = std::make_shared<anafi_ros_interfaces::srv::FpPauseStop::Request>();
                            RCLCPP_INFO(this->get_logger(), "\033[32mSending async request to servfppause_");
                            clifppause_->async_send_request(request_clifppause, response_clifppause_callback);
                            // writing the image to a defined location as JPEG
                            bool check = cv::imwrite("/home/carlos/1st_person_frame.jpg", img_Mat);

                            // if the image is not saved
                            if (check == false) 
                            {
                                RCLCPP_ERROR(this->get_logger(), "Saving the image FAILED");
                            }

                            else 
                            {
                                RCLCPP_INFO(this->get_logger(), "\033[32mSuccessfully saved 1st_person_frame.jpg");
                            }
                        
                        std::cout << "\n" << std::endl;
                        person_counter += 1;
                        if (nose_available == true && extlefthip_available == true && extrighthip_available == true)
                        {
                            person_detected = true;
                        }

                        else {}

                        }
                        
                    }

                    else {}
                }

                else
                {
                    face_kps["nx"] = 0;
                    face_kps["ny"] = 0;
                    face_kps["cx"] = 0;
                    face_kps["cy"] = 0;
                }
            }

            person_counter = 0;

            // Draw the bounding boxes on the image
            yoloV8.drawObjectLabels(img_Mat, objects);

            cv::imshow(OPENCV_WINDOW, img_Mat);
            cv::waitKey(1);

            return face_kps;
        }

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr anafiimg_;
        rclcpp::Client<anafi_person_search_interfaces::srv::FaceKeyPoints>::SharedPtr facepointscli_;
        rclcpp::Client<anafi_ros_interfaces::srv::FpPauseStop>::SharedPtr clifppause_;
        rclcpp::Client<anafi_ros_interfaces::srv::FpStart>::SharedPtr clipfcontinue_;
};

int main(int argc, char * argv[])
{
    try
    {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<Anafi_Person_Detect>());
        rclcpp::shutdown();
    }
    catch (const std::exception &e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Exception: %s", e.what());
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
