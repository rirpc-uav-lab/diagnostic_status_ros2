#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include "yaml-cpp/yaml.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include <map>

using namespace std::chrono_literals;
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

        float default_maxim_wait_time = 3.0f;
        this->declare_parameter("maxim_wait_time", default_maxim_wait_time);
        this->get_parameter_or("maxim_wait_time", maxim_wait_time, default_maxim_wait_time);

        try {
            config = YAML::LoadFile(path_config);
        } catch (const YAML::BadFile& e) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to load YAML file: " << e.what());
            return;
        }

        timer_ = this->create_wall_timer(500ms, std::bind(&Diagnostics::timer_callback, this));

        diagnostics_sub = this->create_subscription<diagnostic_msgs::msg::DiagnosticStatus>("diagnostics", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile(), std::bind(&Diagnostics::diagnostic_cb, this, _1));
        diagnostic_array_pub = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostic_info_output", 10);
        diagnostics_pub = this->create_publisher<std_msgs::msg::String>("diagnostics/emergency_command", 10);
        unknown_diagnostics_sub = this->create_publisher<std_msgs::msg::String>("diagnostics/unknown_keys", 10);
    }

private:

    float maxim_wait_time = 0.0f;
    /**
     * std::map<
     * std::string, - ключ, hardware_id из диагностик статуса. Нужен, чтобы собрать данные по разным частям одного модуля (программного или аппаратного) в одной переменной 
     * std::map<std::string, diagnostic_msgs::msg::DiagnosticStatus> - значение, пояснение далее
     * >
     * 
     * std::map<
     * std::string, - ключ, name из диагностик статуса. В этом мапе собираются все данные по составным частям модуля, которым является hardware_id 
     * diagnostic_msgs::msg::DiagnosticStatus - сам диагностик статус
     * >
     */
    std::map<std::string, std::map<std::string, std::pair<rclcpp::Time, diagnostic_msgs::msg::DiagnosticStatus>>> modules_diagnostics;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diagnostics_sub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr unknown_diagnostics_sub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics_pub;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_array_pub;

    YAML::Node config;

    void timer_callback()
    {
        diagnostic_msgs::msg::DiagnosticArray array_msg;
        diagnostic_msgs::msg::DiagnosticStatus error_msg;

        array_msg.header.stamp = this->get_clock()->now();


        float dt = 0.0f;
        float now = 0.0f;
        float status_time = 0.0f;
        bool flag = 0;
        for (const auto& key : config)
        {
            flag = 0;
            std::string module_name = key.first.as<std::string>();
            for (const auto& [hardware_id, map_name_and_status] : modules_diagnostics)
            {
                for (const auto& [name, status] : map_name_and_status)
                {
                    if (name == module_name)
                    {
                        RCLCPP_INFO(this->get_logger(), "Для модуля %s", module_name);
                        now = this->get_clock()->now().seconds() + this->get_clock()->now().nanoseconds()/float(1e9);
                        RCLCPP_INFO(this->get_logger(), "now = %f", now);

                        status_time = status.first.seconds() + status.first.nanoseconds()/float(1e9);
                        RCLCPP_INFO(this->get_logger(), "status_time = %f", status_time);
                        
                        dt = now - status_time;
                        RCLCPP_INFO(this->get_logger(), "dt = %f", dt);
                        if (abs(dt) < maxim_wait_time)
                        {
                            flag = 1;
                        } 
                    }
                }
            }

            if (flag == 0)
            {
                error_msg.hardware_id = config[module_name]["hardware_id"].as<std::string>();
                error_msg.message = "no information about " + module_name;
                error_msg.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
                array_msg.status.push_back(error_msg);

            }
        }

        for (auto module_iter = modules_diagnostics.begin(); module_iter != modules_diagnostics.end(); module_iter++)
        {
            for (auto submodule_iter = module_iter->second.begin(); submodule_iter != module_iter->second.end(); submodule_iter++)
            {
                array_msg.status.push_back(submodule_iter->second.second);

            }
        }

        diagnostic_array_pub->publish(array_msg);
    }

    std::string safe_get_string(const YAML::Node& node, const std::string& key, std::string module_name)
    {
        if (node[key]) {
            return node[key].as<std::string>();
        } else {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Missing key: " << key);
            std_msgs::msg::String msg;
            msg.data = module_name;
            unknown_diagnostics_sub->publish(msg);
            return "";
        }
    }

    void diagnostic_cb(diagnostic_msgs::msg::DiagnosticStatus diagnostic_status)
    {

        std::string analyzed_variables = safe_get_string(config[diagnostic_status.name], "analyzed_variables", diagnostic_status.name);
        std::string importance = safe_get_string(config[diagnostic_status.name], "importance", diagnostic_status.name);
        std::string fallback_protocol = safe_get_string(config[diagnostic_status.name], "fallback_protocol", diagnostic_status.name);

        if (analyzed_variables == "None")
        {
            if (importance == "executive")
            {
                std::string total_status = "OK";

                if (diagnostic_status.level == diagnostic_msgs::msg::DiagnosticStatus::ERROR)
                {
                    total_status = "ERROR";
                }
                
                if (total_status == "ERROR")
                {
                    std_msgs::msg::String pub = std_msgs::msg::String();
                    pub.data = fallback_protocol;
                    this->diagnostics_pub->publish(pub);
                }
            }
        }
        else
        {
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
                    // RCLCPP_INFO_STREAM(this->get_logger(), "Configuration error: " << e.what());
                }
            }
            if (total_status == "ERROR")
            {
                std_msgs::msg::String pub = std_msgs::msg::String();
                pub.data = fallback_protocol;
                this->diagnostics_pub->publish(pub);
            }
        }

        for (auto module_iter = modules_diagnostics.begin(); module_iter != modules_diagnostics.end(); module_iter++) // for для поиска модуля по hardware_id в большом мапе
        {
            if (diagnostic_status.hardware_id == module_iter->first) // Проверка, нашли (true) или нет (false)
            {
                // Если нашли, то:
                for (auto submodule_iter = module_iter->second.begin(); submodule_iter != module_iter->second.end(); submodule_iter++) // Ищем name в маленьком мапе большого мапа
                {
                    if (submodule_iter->first == diagnostic_status.name) // Проверка, нашли (true) или нет (false)
                    {
                            // Если нашли, то он уже существует. Обновляем данные:
                            submodule_iter->second = std::make_pair(this->get_clock()->now(), diagnostic_status);
                            return; // И возвращаем (выходим) из функции. Дальше код не пойдет
                    }
                }
                // Если мы сюда попали, значит return не сработал, т.е. name в маленьком мапе не найден. Создаем новый элемент в маленьком мапе большого мапа
                auto time_status_pair = std::make_pair(this->get_clock()->now(), diagnostic_status);
                std::pair<std::string, std::pair<rclcpp::Time, diagnostic_msgs::msg::DiagnosticStatus>> data = std::make_pair(diagnostic_status.name, time_status_pair);
                module_iter->second.insert(data);
                return; // И выходим из функции
            }
        }
        auto time_status_pair = std::make_pair(this->get_clock()->now(), diagnostic_status);
        std::pair<std::string, std::pair<rclcpp::Time, diagnostic_msgs::msg::DiagnosticStatus>> data = std::make_pair(diagnostic_status.name, time_status_pair);
        std::map<std::string, std::pair<rclcpp::Time, diagnostic_msgs::msg::DiagnosticStatus>> submodule_map;
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
