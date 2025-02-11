#include <iostream>
#include <chrono>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "anafi_ros_interfaces/srv/gimbal_command_srv.hpp"
#include "anafi_ros_interfaces/srv/fp_upload.hpp"
#include "anafi_ros_interfaces/srv/fp_start.hpp"
#include "anafi_ros_interfaces/srv/fp_pause_stop.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/u_int8.hpp"

using namespace std::this_thread;
using namespace std::chrono_literals;
using std::placeholders::_1;

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

auto qos_default = rclcpp::QoS(
    rclcpp::QoSInitialization(
      qos_profile_default.history,
      qos_profile_default.depth
    ),  
    qos_profile_default);

static const std::string mavlink_fp_path = "/home/carlos/andres_stuff/flightplans/plan_zigzag_ai.mavlink";
std::string fp_uid = "";

std::map<int, std::string> fp_states_log = 
{
    {0, "PLAYING: Mavlink file is playing (0)"},
    {1, "STOPPED: Mavlink file is stopped (arg filepath and type are useless in this state) (1)"},
    {2, "PAUSED: Mavlink file is paused (2)"},
    {3, "LOADED: Mavlink file is loaded (it will be played at take-off) (3)"}
};

std::map<int, std::string> mavlink_types_log = 
{
    {0, "FLIGHTPLAN: Mavlink file for FlightPlan (0)"},
    {1, "MAPMYHOUSE: Mavlink file for MapMyHouse (1)"},
    {2, "FLIGHTPLANV2: Mavlink file for FlightPlan V2 (better follow the standard) (2)"}
};
    
class Anafi_Flightplan_Action : public rclcpp::Node
{
    public:
        Anafi_Flightplan_Action()
        : Node("anafi_flightplan_action")
        {
            RCLCPP_INFO(this->get_logger(), "\033[32m\n>>>>>> NODE anafi_flightplan_action initiated <<<<<<\n");
            fpitemsub_ = this->create_subscription<std_msgs::msg::UInt8>("/anafi/flightplan/item", qos_default, 
                std::bind(&Anafi_Flightplan_Action::fp_item_sub_callback, this, _1));
            //pausefpcli_ = this->create_client<std_srvs::srv::Trigger>("/anafi/flightplan/pause", qos_profile_default);
            //stopfpcli_ = this->create_client<std_srvs::srv::Trigger>("/anafi/flightplan/stop", qos_profile_default);
            //fp_action();
        }

    private:
        void fp_item_sub_callback(const std_msgs::msg::UInt8::SharedPtr item)
        {
            RCLCPP_INFO(this->get_logger(), "\033[32mFP Mission Item %d executed", item->data);
        }

        void fp_action()
        {
            
        }

        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr fpitemsub_;
        //rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr pausefpcli_;
        //rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stopfpcli_;

};

int main(int argc, char * argv[])
{
    try
    {
        rclcpp::init(argc, argv);
        std::shared_ptr<rclcpp::Node> node = std::make_shared<Anafi_Flightplan_Action>();

        rclcpp::Client<anafi_ros_interfaces::srv::FpUpload>::SharedPtr uploadfpcli_ = 
            node->create_client<anafi_ros_interfaces::srv::FpUpload>("/anafi/flightplan/upload", qos_profile_default);
        while (!uploadfpcli_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting");
                return EXIT_SUCCESS;
            }

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[32mService not available, waiting again...");
        }

        auto request_uploadfp = std::make_shared<anafi_ros_interfaces::srv::FpUpload::Request>();
        request_uploadfp->file = mavlink_fp_path;
        auto response_uploadfp = uploadfpcli_->async_send_request(request_uploadfp);
        if (rclcpp::spin_until_future_complete(node, response_uploadfp) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = response_uploadfp.get();
            fp_uid = response->uid;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[32m[NODE] -> anafi uploaded fp successfully with uid: %s", 
                (response->uid).c_str());
        }

        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service for uploadfpcli_");
        }

        rclcpp::Client<anafi_ros_interfaces::srv::FpStart>::SharedPtr fpstartcli_ = 
            node->create_client<anafi_ros_interfaces::srv::FpStart>("/anafi/flightplan/start", qos_profile_default);
        while (!fpstartcli_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting");
                return EXIT_SUCCESS;
            }

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[32mService not available, waiting again...");
        }

        auto request_fpstart = std::make_shared<anafi_ros_interfaces::srv::FpStart::Request>();
        request_fpstart->uid = fp_uid;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[32mPress ENTER to start the Flight Plan");
        while (true)
        {
            char start_key = std::getchar();
            if (start_key != '\n')
            {
                RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "A wrong key was pressed, please hit ENTER to start the Flight Plan");
            }

            else
            {
                break;
            }
        }
        
        auto response_fpstart = fpstartcli_->async_send_request(request_fpstart);
        if (rclcpp::spin_until_future_complete(node, response_fpstart) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = response_fpstart.get();
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[32m[NODE] -> anafi FP state: %s", fp_states_log[response->state].c_str());
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[32m[NODE] -> anafi FP file path: %s", (response->filepath).c_str());
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[32m[NODE] -> anafi FP type: %s", mavlink_types_log[response->type].c_str());
        }

        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service for startfpcli_");
        }

        rclcpp::Client<anafi_ros_interfaces::srv::GimbalCommandSrv>::SharedPtr gimbalcommandcli_ = 
            node->create_client<anafi_ros_interfaces::srv::GimbalCommandSrv>("/anafi/gimbal/command", qos_profile_default);
        while (!gimbalcommandcli_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting");
                return EXIT_SUCCESS;
            }

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[32mService not available, waiting again...");
        }

        auto request_gimbalcommand = std::make_shared<anafi_ros_interfaces::srv::GimbalCommandSrv::Request>();
        request_gimbalcommand->pitch = 90.0;
        auto response_gimbalcommand = gimbalcommandcli_->async_send_request(request_gimbalcommand);
        if (rclcpp::spin_until_future_complete(node, response_gimbalcommand) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = response_gimbalcommand.get();
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[32m[NODE] -> anafi Gimbal command_status: %s", 
                (response->command_status).c_str());
        }

        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service for gimbalcommandcli_");
        }

        rclcpp::Client<anafi_ros_interfaces::srv::FpPauseStop>::SharedPtr fppausecli_ = 
            node->create_client<anafi_ros_interfaces::srv::FpPauseStop>("/anafi/flightplan/pause", qos_profile_default);
        
        while (!fppausecli_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting");
                return EXIT_SUCCESS;
            }

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[32mService not available, waiting again...");
        }

        auto request_fppause = std::make_shared<anafi_ros_interfaces::srv::FpPauseStop::Request>();
        for (int time_count = 4; time_count > 0; time_count--)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[32m%d seconds left before FP is paused", time_count);
            sleep_for(1s);
        }

        auto response_fppause = fppausecli_->async_send_request(request_fppause);
        if (rclcpp::spin_until_future_complete(node, response_fppause) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = response_fppause.get();
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[32m[NODE] -> anafi FP state: %s", fp_states_log[response->state].c_str());
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[32m[NODE] -> anafi FP file path: %s", (response->filepath).c_str());
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[32m[NODE] -> anafi FP type: %s", mavlink_types_log[response->type].c_str());
        }

        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service for fppausecli_");
        }

        for (int time_count = 4; time_count > 0; time_count--)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[32m%d second left before FP is continued", time_count);
            sleep_for(1s);
        }

        request_fpstart->uid = fp_uid;
        response_fpstart = fpstartcli_->async_send_request(request_fpstart);
        if (rclcpp::spin_until_future_complete(node, response_fpstart) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = response_fpstart.get();
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[32m[NODE] -> anafi FP state: %s", fp_states_log[response->state].c_str());
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[32m[NODE] -> anafi FP file path: %s", (response->filepath).c_str());
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[32m[NODE] -> anafi FP type: %s", mavlink_types_log[response->type].c_str());
        }

        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service for fpstartcli_");
        }

        rclcpp::spin(node);
        rclcpp::shutdown();
    }

    catch (const std::exception &e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Exception: %s", e.what());
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}