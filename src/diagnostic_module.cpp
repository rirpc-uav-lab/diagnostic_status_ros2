#include <chrono>
#include <memory>
#include <string>
// #include <telemetry.h>
// #include <mavsdk/plugins/telemetry/telemetry.h>
// #include <mavsdk/mavsdk.h>
#include <vector>
#include <iostream>
#include "yaml-cpp/yaml.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
// #include <mavsdk.h>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include <map>

using namespace std::chrono_literals;
// using mavsdk::Telemetry, mavsdk::Mavsdk;
using std::placeholders::_1, std::placeholders::_2;

class Diagnostics : public rclcpp::Node
{
public:
    Diagnostics()
    : Node("diagnostics")
    {
        std::string default_path_config = "/home/max/uav_ws/src/diagnostic_status_ros2/config/config.yml";
        std::string path_config = default_path_config;
        this->declare_parameter("path", default_path_config);
        this->get_parameter_or("path", path_config, default_path_config);

        config = YAML::LoadFile(path_config);

        // auto timer_callback = [this]() -> void 
        //   {
        //     std::string path_config = "/home/max/uav_ws/src/diagnostic_status_ros2/config/config.yml";
        //     config = YAML::LoadFile(path_config);
            
        //     RCLCPP_ERROR_STREAM(this->get_logger(), "gps_info: analyzed_variables " << config["gps_info"]["analyzed_variables"].as<std::string>());
        //     RCLCPP_ERROR_STREAM(this->get_logger(), "gps_info: fallback_protocol " << config["gps_info"]["fallback_protocol"].as<std::string>());
        //     RCLCPP_ERROR_STREAM(this->get_logger(), "gps_info: importance " << config["gps_info"]["importance"].as<std::string>());

        //     RCLCPP_ERROR_STREAM(this->get_logger(), "fcu_health: analyzed_variables " << config["fcu_health"]["analyzed_variables"].as<std::string>());
        //     RCLCPP_ERROR_STREAM(this->get_logger(), "fcu_health: fallback_protocol " << config["fcu_health"]["fallback_protocol"].as<std::string>());
        //     RCLCPP_ERROR_STREAM(this->get_logger(), "fcu_health: importance " << config["fcu_health"]["importance"].as<std::string>());

        //   };

        timer_ = this->create_wall_timer(500ms, std::bind(&Diagnostics::timer_callback, this));

        diagnostics_sub = this->create_subscription<diagnostic_msgs::msg::DiagnosticStatus>("diagnostics", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile(), std::bind(&Diagnostics::diagnostic_cb, this, _1));
        diagnostic_array_pub = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostic_info_output", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile());
        diagnostics_pub = this->create_publisher<std_msgs::msg::String>("diagnostics/emergency_command", 10);
    }

private:
    std::map<std::string, std::map<std::string, diagnostic_msgs::msg::DiagnosticStatus>> modules_diagnostics;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diagnostics_sub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics_pub;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_array_pub;


    YAML::Node config;
//   rclcpp::SubscriptionOptions sub_options;


    void timer_callback()
    {
        diagnostic_msgs::msg::DiagnosticArray array_msg;
        array_msg.header.stamp = this->get_clock()->now();
        
        for (auto module_iter = modules_diagnostics.begin(); module_iter != modules_diagnostics.end(); module_iter++)
        {
            for (auto submodule_iter = module_iter->second.begin(); submodule_iter != module_iter->second.end(); submodule_iter++)
            {
                array_msg.status.push_back(submodule_iter->second);
            }
        }

        diagnostic_array_pub->publish(array_msg);
    }


    void diagnostic_cb(diagnostic_msgs::msg::DiagnosticStatus diagnostic_status)
    {
        if (config[diagnostic_status.name]["analyzed_variables"].as<std::string>() == "null")
        {
            // Чекаем статус
            if (config[diagnostic_status.name]["importance"].as<std::string>() == "executive")
            {
                std::string total_status = "OK";
                for (const auto & [key,value]: diagnostic_status.values)
                {
                    if (value == "ERROR")
                    {
                        total_status = "ERROR";
                    }
                }
                if (total_status == "ERROR")
                {
                    //паблишим команду из config[diagnostic_status.name]["fallback_protocol"]
                    std_msgs::msg::String pub = std_msgs::msg::String(); 
                    pub.data = config[diagnostic_status.name]["fallback_protocol"].as<std::string>();
                    this->diagnostics_pub->publish(pub);
                }
            }
        }

        else 
        {
            // std::map<std::string, std::string> analyzed_variables = config[diagnostic_status.name]["analyzed_variables"].as<std::map<std::string, std::string>>();
            // Чекаем переменные в analyzed_variables
            std::string total_status = "OK";
            for (const auto & [key,value]: diagnostic_status.values)
            {
                try
                {
                    if (config[diagnostic_status.name]["analyzed_variables"][key][value].as<std::string>() == "ERROR")
                    {
                        total_status = "ERROR";
                        diagnostic_status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
                    }
                }
                catch (const std::exception& e)
                {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Configuration error: " << e.what());
                }
            }
            if (total_status == "ERROR")
            {
                //паблишим команду из config[diagnostic_status.name]["fallback_protocol"]
                std_msgs::msg::String pub = std_msgs::msg::String(); 
                pub.data = config[diagnostic_status.name]["fallback_protocol"].as<std::string>();
                this->diagnostics_pub->publish(pub);
            }
        }

        for (auto module_iter = modules_diagnostics.begin(); module_iter != modules_diagnostics.end(); module_iter++)
        {
            if (diagnostic_status.hardware_id == module_iter->first)
            {
                for (auto submodule_iter = module_iter->second.begin(); submodule_iter != module_iter->second.end(); submodule_iter++)
                {
                    if (submodule_iter->first == diagnostic_status.name)
                    {
                            submodule_iter->second = diagnostic_status;
                            return;
                    }
                }
                std::pair<std::string, diagnostic_msgs::msg::DiagnosticStatus> data = std::make_pair(diagnostic_status.name, diagnostic_status);
                module_iter->second.insert(data);
                return;
            }
        }
        std::pair<std::string, diagnostic_msgs::msg::DiagnosticStatus> data = std::make_pair(diagnostic_status.name, diagnostic_status);
        std::map<std::string, diagnostic_msgs::msg::DiagnosticStatus> submodule_map;
        submodule_map.insert(data);
        modules_diagnostics.insert(std::pair(diagnostic_status.hardware_id, submodule_map));
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Diagnostics>());
    rclcpp::shutdown();
    return 0;
}