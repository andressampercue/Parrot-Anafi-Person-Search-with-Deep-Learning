#define _USE_MATH_DEFINES
#include <cmath>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "anafi_person_search_interfaces/srv/face_key_points.hpp"
#include "anafi_ros_interfaces/srv/move_by_srv.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

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

//res: 1920x1080 -> dd/dpix = 74/1920[cm/pix]
//res: 1280x720 -> dd/dpix = 10.36/720[m/pix] @ 15 m alt

uint cam_w = 1280;
uint cam_h = 720;
float dd_dpix = 10.36/cam_h; //2.94 [m/pix]
float ref_alt = 15.0;
std::map<int, std::string> moveby_error_status =
{
    {0, "OK"}, 
    {1, "UNKNOWN"}, 
    {2, "BUSY"}, 
    {3, "NOTAVAILABLE"}, 
    {4, "INTERRUPTED"}
};

bool coords_log_flag = false;
int moveby_counter = 2;
bool moveby1_flag = false;
bool moveby2_flag = false;
bool moveby3_flag = false;
bool moveby4_flag = false;
bool moveby5_flag = false;
bool moveby6_flag = false;
bool moveby7_flag = false;
bool moveby8_flag = false;
bool moveby9_flag = false;

auto request_moveby = std::make_shared<anafi_ros_interfaces::srv::MoveBySrv::Request>();
std::vector<int> cam_mid_coords;
std::vector<int> key_coords;
std::vector<int> D_nc;
int nx = 0;
int ny = 0;
int cx = 0;
int cy = 0;
float alpha = 0.0;
float hipo_p = 0.0;
float hipo_p_m = 0.0;
double epsilon = 0.0;
double betha = 0.0;

class Anafi_Face_Reubication : public rclcpp::Node
{
    public:
    Anafi_Face_Reubication()
    : Node("anafi_person_reubication")
    {
        RCLCPP_INFO(this->get_logger(), "\033[32m\n>>>>>> NODE anafi_face_reubication initiated <<<<<<\n");
        facepointssrv_ = this->create_service<anafi_person_search_interfaces::srv::FaceKeyPoints>("key_face_points", 
                        std::bind(&Anafi_Face_Reubication::face_points, this, _1, _2), custom_rmw_qos_profile_default);
        movebysrv_ = this->create_client<anafi_ros_interfaces::srv::MoveBySrv>("/anafi/drone/moveby", qos_profile_default);

        void face_reubication_action();
        void moveby_srv_calling(std::shared_ptr<anafi_ros_interfaces::srv::MoveBySrv::Request> request_moveby);
    }

    private:
        void face_points(const std::shared_ptr<anafi_person_search_interfaces::srv::FaceKeyPoints::Request> request, 
                        std::shared_ptr<anafi_person_search_interfaces::srv::FaceKeyPoints::Response> response)
        {
            if (coords_log_flag == false)
            {
                response->reception_state = "Nose and central key points received with success";
                nx = request->nx;
                ny = request->ny;
                cx = request->cx;
                cy = request->cy;
                RCLCPP_INFO(this->get_logger(), "\033[32mnx: %d", nx);
                RCLCPP_INFO(this->get_logger(), "\033[32mny: %d", ny);
                RCLCPP_INFO(this->get_logger(), "\033[32mcx: %d", cx);
                RCLCPP_INFO(this->get_logger(), "\033[32mcy: %d", cy);
                coords_log_flag = true;
            }

            else {}

            face_reubication_action();
        }

        void face_reubication_action()
        {
            if (moveby1_flag == false)
            {
                cam_mid_coords.push_back(cam_w/2);
                cam_mid_coords.push_back(cam_h/2);
                key_coords.push_back(nx-cam_mid_coords[0]);
                key_coords.push_back(ny-cam_mid_coords[1]);
                //calcular primero distancias de catetos por separado en [m]
                //hipo_p = pow(pow(key_coords[0], 2) + pow(key_coords[1], 2), 0.5);
                hipo_p_m = pow(pow(key_coords[0]*dd_dpix, 2) + pow(key_coords[1]*dd_dpix, 2), 0.5);
                //hipo_p_m = hipo_p * dd_dpix;
                if (key_coords[0] != 0)
                {
                    RCLCPP_INFO(this->get_logger(), "\033[32mnose key_coords[1]: %d", key_coords[1]);
                    RCLCPP_INFO(this->get_logger(), "\033[32mnose key_coords[0]: %d", key_coords[0]);
                    epsilon = static_cast<double>(key_coords[1])/key_coords[0];
                    betha = abs(atan(epsilon)*(180.0/M_PI));
                    RCLCPP_INFO(this->get_logger(), "\033[32mnose epsilon: %f", epsilon);
                    RCLCPP_INFO(this->get_logger(), "\033[32mnose betha: %f", betha);
                    if (epsilon < 0)
                    {
                        if (nx > cam_mid_coords[0])
                        {
                            alpha = 90.0 - betha;
                        }
                    
                        else
                        {
                            alpha = -betha - 90.0;
                        }
                    }

                    else if (epsilon > 0)
                    {
                        if (nx > cam_mid_coords[0])
                        {
                            alpha = betha + 90.0;
                        }

                        else
                        {
                            alpha = -90.0 + betha;
                        }
                    }
                }

                else
                {
                    if (key_coords[1] > cam_mid_coords[1])
                    {
                        alpha = 180.0;
                    }

                    else
                    {
                        alpha = 0.0;
                    }
                }

                RCLCPP_INFO(this->get_logger(), "\033[32mnose hipo_p_m: %f", hipo_p_m);
                RCLCPP_INFO(this->get_logger(), "\033[32mnose alpha: %f", alpha);
                request_moveby->dx = 0.0;
                request_moveby->dy = 0.0;
                request_moveby->dz = 0.0;
                request_moveby->dyaw = alpha;
                RCLCPP_INFO(this->get_logger(), "\033[32mTurning in direction of nose key point");
                moveby_srv_calling(request_moveby);
                moveby1_flag = true;
            }
            
            if (moveby2_flag == true)
            {
                RCLCPP_INFO(this->get_logger(), "\033[32mAngle reached for nose key point");
                request_moveby->dx = hipo_p_m;
                request_moveby->dy = 0.0;
                request_moveby->dz = 0.0;
                request_moveby->dyaw = 0.0;
                RCLCPP_INFO(this->get_logger(), "\033[32mMoving by nose key point");
                moveby_srv_calling(request_moveby);
                moveby2_flag = false;
            }

            if (moveby3_flag == true)
            {
                //RCLCPP_INFO(this->get_logger(), "\033[32mAngle reached for nose key point");
                request_moveby->dx = 0.0;
                request_moveby->dy = 0.0;
                request_moveby->dz = 0.0;
                request_moveby->dyaw = -alpha;
                //RCLCPP_INFO(this->get_logger(), "\033[32mMoving by nose key point");
                moveby_srv_calling(request_moveby);
                moveby3_flag = false;
            }

            if (moveby4_flag == true)
            {
                RCLCPP_INFO(this->get_logger(), "\033[32mDrone positioned above of nose key point");
                D_nc.push_back(cx-nx);
                D_nc.push_back(cy-ny);
                epsilon = static_cast<double>(D_nc[1])/D_nc[0];
                betha = abs(atan(epsilon)*(180.0/M_PI));
                RCLCPP_INFO(this->get_logger(), "\033[32mCentral north epsilon: %f", epsilon);
                RCLCPP_INFO(this->get_logger(), "\033[32mCentral north betha: %f", betha);
                if (D_nc[0] != 0 && D_nc[1] != 0)
                {
                    if ((D_nc[0] * D_nc[1]) > 0)
                    {
                        if (D_nc[0] < 0)
                        {
                            alpha = 90.0 + betha;
                        }

                        else
                        {
                            
                            alpha = -(90.0 - betha);
                        }
                    }

                    else
                    {
                        if (D_nc[0] < 0)
                        {
                            alpha = 90.0 - betha;
                        }

                        else
                        {
                            
                            alpha = -(90.0 + betha);
                        }
                    }
                }

                else
                {
                    if (D_nc[0] == 0)
                    {
                        if (cy > ny)
                        {
                            alpha = 180.0;
                        }

                        else
                        {
                            alpha = 0.0;
                        }
                    }

                    else if (D_nc[1] == 0)
                    {
                        if (cx > nx)
                        {
                            alpha = 90.0;
                        }

                        else
                        {
                            alpha = -90.0;
                        }
                    }
                }

                RCLCPP_INFO(this->get_logger(), "\033[32mCentral north alpha: %f", alpha);
                request_moveby->dx = 0.0;
                request_moveby->dy = 0.0;
                request_moveby->dz = 0.0;
                request_moveby->dyaw = alpha;
                moveby_srv_calling(request_moveby);
                moveby4_flag = false;
            }

            if (moveby5_flag == true)
            {
                RCLCPP_INFO(this->get_logger(), "\033[32mAngle reached for face central north key point");
                RCLCPP_INFO(this->get_logger(), "\033[32mFirst Ascending to face ROI");
                request_moveby->dx = 0.0;
                request_moveby->dy = 0.0;
                request_moveby->dz = 9.0;
                request_moveby->dyaw = 0.0;
                moveby_srv_calling(request_moveby);
                moveby5_flag = false;
            }

            if (moveby6_flag == true)
            {
                RCLCPP_INFO(this->get_logger(), "\033[32mRe-centralizing drone position on face key point along <x> axis");
                request_moveby->dx = -0.39;
                request_moveby->dy = 0.0;
                request_moveby->dz = 0.0;
                request_moveby->dyaw = 0.0;
                moveby_srv_calling(request_moveby);
                moveby6_flag = false;
            }

            if (moveby7_flag == true)
            {
                RCLCPP_INFO(this->get_logger(), "\033[32mRe-centralizing drone position on face key point along <y> axis");
                request_moveby->dx = 0.0;
                request_moveby->dy = 0.49;
                request_moveby->dz = 0.0;
                request_moveby->dyaw = 0.0;
                moveby_srv_calling(request_moveby);
                moveby7_flag = false;
            }

            if (moveby8_flag == true)
            {
                RCLCPP_INFO(this->get_logger(), "\033[32mFinal Ascending to face ROI");
                request_moveby->dx = 0.0;
                request_moveby->dy = 0.0;
                request_moveby->dz = 2.5;
                request_moveby->dyaw = 0.0;
                moveby_srv_calling(request_moveby);
                moveby8_flag = false;
            }

            if (moveby9_flag == true)
            {
                request_moveby->dx = 0.0;
                request_moveby->dy = 0.0;
                request_moveby->dz = 0.0;
                request_moveby->dyaw = 0.0;
                moveby_srv_calling(request_moveby);
                moveby9_flag = false;
            }
        }

        void moveby_srv_calling(std::shared_ptr<anafi_ros_interfaces::srv::MoveBySrv::Request> request_moveby)
        {
            auto response_callback = [this](rclcpp::Client<anafi_ros_interfaces::srv::MoveBySrv>::SharedFuture future)
            {
                try
                {
                    RCLCPP_INFO(this->get_logger(), "\033[32mCallback invoked");
                    auto response = future.get();
                    if (response == nullptr)
                    {
                        RCLCPP_ERROR(this->get_logger(), "Received null response");
                    }
                    
                    else
                    {
                        RCLCPP_INFO(this->get_logger(), "\033[32m[NODE] -> anafi moveby service said: %s", moveby_error_status[response->error].c_str());
                        RCLCPP_INFO(this->get_logger(), "\033[32mdX: %f", response->dx);
                        RCLCPP_INFO(this->get_logger(), "\033[32mdY: %f", response->dy);
                        RCLCPP_INFO(this->get_logger(), "\033[32mdZ: %f", response->dz);
                        RCLCPP_INFO(this->get_logger(), "\033[32mdyaw: %f", response->dpsi);
                        if (moveby_counter == 2)
                        {
                            moveby2_flag = true;
                            moveby_counter += 1;
                            face_reubication_action();
                        }

                        else if (moveby_counter == 3) 
                        {
                            moveby3_flag = true;
                            moveby_counter += 1;
                            face_reubication_action();
                        }

                        else if (moveby_counter == 4)
                        {
                            moveby4_flag = true;
                            moveby_counter += 1;
                            face_reubication_action();
                        }

                        else if (moveby_counter == 5)
                        {
                            moveby5_flag = true;
                            moveby_counter += 1;
                            face_reubication_action();
                        }

                        else if (moveby_counter == 6)
                        {
                            moveby6_flag = true;
                            moveby_counter += 1;
                            face_reubication_action();
                        }

                        else if (moveby_counter == 7)
                        {
                            moveby7_flag = true;
                            moveby_counter += 1;
                            face_reubication_action();
                        }

                        else if (moveby_counter == 8)
                        {
                            moveby8_flag = true;
                            moveby_counter += 1;
                            face_reubication_action();
                        }

                        else if (moveby_counter == 9)
                        {
                            moveby9_flag = true;
                            moveby_counter += 1;
                            face_reubication_action();
                        }

                        else {}
                    }
                }

                catch (const std::exception &e)
                {
                    RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
                }
            };

            RCLCPP_INFO(this->get_logger(), "\033[32mSending asyn request to movebysrv_");
            movebysrv_->async_send_request(request_moveby, response_callback);
        }

        rclcpp::Service<anafi_person_search_interfaces::srv::FaceKeyPoints>::SharedPtr facepointssrv_;
        rclcpp::Client<anafi_ros_interfaces::srv::MoveBySrv>::SharedPtr movebysrv_;
};

int main(int argc, char * argv[])
{
    try
    {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<Anafi_Face_Reubication>());
        rclcpp::shutdown();
    }

    catch(const std::exception &e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Exception: %s", e.what());
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}