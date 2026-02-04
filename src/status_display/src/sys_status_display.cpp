#include <QLabel>
#include <QApplication>
#include <QTimer>
#include <sstream>
#include "status_display/sys_status_display.h"

SysStatusDisplay::SysStatusDisplay() : Node("sys_status_display")
{
    subscription_ = this->create_subscription<SystemStatus>(
        "system_status", 10, [&](const SystemStatus::SharedPtr msg)
        {
            status_text_ = formatStatusMessage(*msg);
            RCLCPP_INFO(this->get_logger(), "Received System Status:\n%s",
             status_text_.toStdString().c_str()); });
}

QString SysStatusDisplay::getStatusText() const
{
    return status_text_;
}

QString SysStatusDisplay::formatStatusMessage(const SystemStatus &msg)
{
    std::stringstream ss;
    ss << "========System Status Display========" << std::endl;
    ss << "Host Name: " << msg.host_name << std::endl;
    ss << "CPU Usage: " << msg.cpu_percent << "%" << std::endl;
    ss << "Memory Usage: " << msg.memory_percent << "%" << std::endl;
    ss << "Total Memory: " << msg.memory_total << " MB" << std::endl;
    ss << "Available Memory: " << msg.memory_available << " MB" << std::endl;
    ss << "Network Sent: " << msg.net_sent << " bytes" << std::endl;
    ss << "Network Received: " << msg.net_recv << " bytes" << std::endl;
    ss << "======================================" << std::endl;
    return QString::fromStdString(ss.str());
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SysStatusDisplay>();
    QApplication app(argc, argv);
    QLabel label;
    std::thread spin_thread([&]()
                            {
    RCLCPP_INFO(node->get_logger(), "ROS 2 spin thread STARTED");
    try { rclcpp::spin(node); }
    catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Spin thread exception: %s", e.what());
    }
    RCLCPP_INFO(node->get_logger(), "ROS 2 spin thread EXITED"); });
    spin_thread.detach();
    QTimer timer;
    QObject::connect(&timer, &QTimer::timeout, [&]()
                     { label.setText(node->getStatusText());
                    std::cout << node->getStatusText().toStdString() << std::endl; });
    timer.start(1000); // Update every second
    label.setWindowTitle("System Status (Real-Time)");
    label.setText(node->getStatusText());
    label.resize(300, 100);
    label.show();

    app.exec();
    rclcpp::shutdown();
    return 0;
}