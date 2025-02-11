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

auto qos_default = rclcpp::QoS(
    rclcpp::QoSInitialization(
      qos_profile_default.history,
      qos_profile_default.depth
    ),  
    qos_profile_default);

static const std::string mavlink_fp_path = "/home/carlos/andres_stuff/flightplans/plan_zigzag_ai.mavlink";
std::string fp_uid = "";

std::map<std::string, int> fp_states = 
{
    {"PLAYING", 0}, 
    {"STOPPED", 1}, 
    {"PAUSED", 2}, 
    {"LOADED", 3}, 
    {"TIMED_OUT", 4}, 
    {"NO_STATE", 5}
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

std::map<std::string, int> mavlink_types = 
{
    {"FLIGHTPLAN", 0}, 
    {"MAPMYHOUSE", 1}, 
    {"FLIGHTPLANV2", 2}, 
    {"UNKNOWN", 3}
};

std::map<int, std::string> mavlink_types_log = 
{
    {0, "FLIGHTPLAN: Mavlink file for FlightPlan (0)"},
    {1, "MAPMYHOUSE: Mavlink file for MapMyHouse (1)"},
    {2, "FLIGHTPLANV2: Mavlink file for FlightPlan V2 (better follow the standard) (2)"},
    {3, "UNKNOWN: Mavlink file type unknown (3)"}
};

uint state = 0;
std::string filepath = "";
uint type = 0;
    
class Anafi_Flightplan_Action : public rclcpp::Node
{
    public:
        Anafi_Flightplan_Action()
        : Node("anafi_flightplan_action")
        {
            RCLCPP_INFO(this->get_logger(), "\033[32m\n>>>>>> NODE anafi_flightplan_action initiated <<<<<<\n");
            fpitemsub_ = this->create_subscription<std_msgs::msg::UInt8>("/anafi/flightplan/item", qos_default, 
                std::bind(&Anafi_Flightplan_Action::fp_item_sub_callback, this, _1));
            
            fpcontinueserver_ = this->create_service<anafi_ros_interfaces::srv::FpStart>("flightplan_continue", 
                std::bind(&Anafi_Flightplan_Action::fp_continue, this, _1, _2), qos_profile_default);

            anafifpcontinuecli_ = this->create_client<anafi_ros_interfaces::srv::FpStart>("/anafi/flightplan/start", qos_profile_default);
        }

    private:
        void fp_item_sub_callback(const std_msgs::msg::UInt8::SharedPtr item)
        {
            RCLCPP_INFO(this->get_logger(), "\033[32mFP Mission Item %d executed", item->data);
        }

        void fp_continue(const std::shared_ptr<anafi_ros_interfaces::srv::FpStart::Request> request, 
            std::shared_ptr<anafi_ros_interfaces::srv::FpStart::Response> response)
        {
            (void)request;
            while (!anafifpcontinuecli_->wait_for_service(1s))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting");
                }

                RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
            }

            RCLCPP_WARN(this->get_logger(), "Continuing current flightplan");
            auto response_anafifpcontinuecli_callback = [this](rclcpp::Client<anafi_ros_interfaces::srv::FpStart>::SharedFuture future)
            {
                try
                {
                    RCLCPP_INFO(this->get_logger(), "Callback invoked");
                    auto response_anafi = future.get();
                    if (response_anafi == nullptr)
                    {
                        RCLCPP_ERROR(this->get_logger(), "Received null response");
                        state = fp_states["NO_STATE"];
			            filepath = "UNREACHABLE";
			            type = mavlink_types["UNKNOWN"];
                    }

                    else
                    {
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[NODE] -> anafi FP state: %s", fp_states_log[response_anafi->state].c_str());
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[NODE] -> anafi FP file path: %s", (response_anafi->filepath).c_str());
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[NODE] -> anafi FP type: %s", mavlink_types_log[response_anafi->type].c_str());
                        state = response_anafi->state;
                        filepath = response_anafi->filepath;
                        type = response_anafi->type;
                    }
                }
                
                catch (const std::exception &e)
                {
                    RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
                    state = fp_states["TIMED_OUT"];
                    filepath = "UNREACHABLE";
                    type = mavlink_types["UNKNOWN"];
                }
            };

            RCLCPP_INFO(this->get_logger(), "Sending async request to anafifpcontinuecli_");
            auto request_anafifpcontinuecli = std::make_shared<anafi_ros_interfaces::srv::FpStart::Request>();
            request_anafifpcontinuecli->uid = fp_uid;
            anafifpcontinuecli_->async_send_request(request_anafifpcontinuecli, response_anafifpcontinuecli_callback);
            response->state = state;
            response->filepath = filepath;
            response->type = type;
        }

        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr fpitemsub_;
        rclcpp::Service<anafi_ros_interfaces::srv::FpPauseStop>::SharedPtr fppauseserver_;
        rclcpp::Client<anafi_ros_interfaces::srv::FpPauseStop>::SharedPtr anafifppausecli_;
        rclcpp::Service<anafi_ros_interfaces::srv::FpStart>::SharedPtr fpcontinueserver_;
        rclcpp::Client<anafi_ros_interfaces::srv::FpStart>::SharedPtr anafifpcontinuecli_;
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
            auto param_fpuid_desc = rcl_interfaces::msg::ParameterDescriptor{};
            param_fpuid_desc.description = "Current flightplan uid uploaded in drone";
            node->declare_parameter("fp_uid", fp_uid, param_fpuid_desc);
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