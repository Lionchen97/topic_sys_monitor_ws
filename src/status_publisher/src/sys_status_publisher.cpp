// sys_status_publisher_alternative.cpp
#include "rclcpp/rclcpp.hpp"
#include "status_interfaces/msg/system_status.hpp"
#include <unistd.h>
#include <chrono>
#include <string>
#include <sys/sysinfo.h>

class SysStatusPublisher : public rclcpp::Node
{
public:
    SysStatusPublisher(const std::string &node_name) : Node(node_name)
    {
        publisher_ = this->create_publisher<status_interfaces::msg::SystemStatus>(
            "system_status", 10);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&SysStatusPublisher::publish_status, this));

        RCLCPP_INFO(this->get_logger(), "System Status Publisher initialized");
    }

private:
    void publish_status()
    {
        auto message = status_interfaces::msg::SystemStatus();

        // 1. 设置时间戳
        message.stamp = this->now();

        // 2. 获取主机名
        char hostname[256];
        if (gethostname(hostname, sizeof(hostname)) == 0)
        {
            message.host_name = hostname;
        }
        else
        {
            message.host_name = "unknown";
        }

        // 3. 获取CPU使用率（简化版本）
        static int call_count = 0;
        static float cpu_usage = 0.0f;

        // 模拟CPU使用率变化
        if (call_count++ % 10 < 5)
        {
            cpu_usage = 20.0f + (rand() % 60);
        }
        else
        {
            cpu_usage = 10.0f + (rand() % 30);
        }
        message.cpu_percent = cpu_usage;

        // 4. 获取内存信息（使用sysinfo）
        struct sysinfo info;
        if (sysinfo(&info) == 0)
        {
            // 转换为MB
            message.memory_total = static_cast<float>(info.totalram * info.mem_unit / (1024.0 * 1024.0));
            message.memory_available = static_cast<float>(info.freeram * info.mem_unit / (1024.0 * 1024.0));

            if (info.totalram > 0)
            {
                float used_ram = info.totalram - info.freeram;
                message.memory_percent = 100.0f * used_ram / info.totalram;
            }
            else
            {
                message.memory_percent = 0.0f;
            }
        }
        else
        {
            message.memory_percent = 50.0f;
            message.memory_total = 8192.0f;
            message.memory_available = 4096.0f;
        }

        // 5. 模拟网络统计
        static double net_sent = 0.0;
        static double net_recv = 0.0;

        // 模拟网络数据增长
        net_sent += 1000 + (rand() % 5000);
        net_recv += 2000 + (rand() % 8000);

        message.net_sent = net_sent;
        message.net_recv = net_recv;

        // 发布消息
        RCLCPP_INFO(this->get_logger(), "Publishing System Status:");
        RCLCPP_INFO(this->get_logger(), "  Host: %s", message.host_name.c_str());
        RCLCPP_INFO(this->get_logger(), "  CPU: %.2f%%", message.cpu_percent);
        RCLCPP_INFO(this->get_logger(), "  Memory: %.2f%% (%.0f MB / %.0f MB)",
                    message.memory_percent,
                    message.memory_available,
                    message.memory_total);
        RCLCPP_INFO(this->get_logger(), "  Network Sent: %.0f bytes", message.net_sent);
        RCLCPP_INFO(this->get_logger(), "  Network Received: %.0f bytes", message.net_recv);
        publisher_->publish(message);
    }

private:
    rclcpp::Publisher<status_interfaces::msg::SystemStatus>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    // 初始化随机种子
    srand(static_cast<unsigned int>(time(nullptr)));

    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<SysStatusPublisher>("sys_status_publisher");
        RCLCPP_INFO(node->get_logger(), "Starting System Status Publisher...");

        rclcpp::spin(node);
    }
    catch (const std::exception &e)
    {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}