#ifndef SYS_STATUS_DISPLAY_H
#define SYS_STATUS_DISPLAY_H

#include <QString>
#include <rclcpp/rclcpp.hpp>
#include "status_interfaces/msg/system_status.hpp"

using SystemStatus = status_interfaces::msg::SystemStatus;

class SysStatusDisplay : public rclcpp::Node
{
public:
    SysStatusDisplay();

    QString getStatusText() const;

private:
    QString formatStatusMessage(const SystemStatus &msg);

    rclcpp::Subscription<SystemStatus>::SharedPtr subscription_;
    QString status_text_{"Waiting for system status..."};
};

#endif // SYS_STATUS_DISPLAY_H