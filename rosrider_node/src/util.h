#ifndef __ROSRIDER_UTIL_H
#define __ROSRIDER_UTIL_H

#include <string>
#include <cstdint>
#include <rclcpp/rclcpp.hpp>

inline void print_mtr_status(rclcpp::Logger logger, uint8_t mtr_status) {

    std::string status_str;
    status_str.reserve(128);

    status_str = "MOTOR_STATUS | ";
    if (mtr_status & 0x80) status_str += "FAULT_RIGHT | ";
    if (mtr_status & 0x40) status_str += "FAULT_LEFT | ";

    status_str += (mtr_status & 0x20) ? "FWD_RIGHT | " : "REV_RIGHT | ";
    status_str += (mtr_status & 0x10) ? "FWD_LEFT | " : "REV_LEFT | ";

    if (mtr_status & 0x08) status_str += "MODE2 | ";
    if (mtr_status & 0x04) status_str += "MODE1 | ";

    uint8_t drive_mode = mtr_status & 0x03;
    status_str += "DRIVE: " + std::to_string(drive_mode) + " | ";
    status_str += std::to_string(mtr_status);

    RCLCPP_INFO(logger, "%s", status_str.c_str());
}

inline void print_sys_status(rclcpp::Logger logger, uint8_t sys_status) {

    std::string status_str;
    status_str.reserve(64);

    status_str = "SYS_STATUS | ";
    if (sys_status & 0x80) status_str += "EEPROM_INIT_ERROR | ";
    if (sys_status & 0x40) status_str += "RESTART_REQUIRED | ";
    if (sys_status & 0x01) status_str += "EEPROM_WRITE_I2C_ERROR | ";

    status_str += std::to_string(sys_status);

    RCLCPP_INFO(logger, "%s", status_str.c_str());
}

inline void print_pwr_status(rclcpp::Logger logger, uint8_t pwr_status) {

    std::string status_str;
    status_str.reserve(128);

    status_str = "POWER_STATUS | ";
    if (pwr_status & 0x80) status_str += "CMD_TIMEOUT | ";
    if (pwr_status & 0x40) status_str += "POWER_BAD | ";
    if (pwr_status & 0x20) status_str += "RIGHT_AMP_LIMIT | ";
    if (pwr_status & 0x10) status_str += "LEFT_AMP_LIMIT | ";
    if (pwr_status & 0x08) status_str += "MAIN_AMP_LIMIT | ";
    if (pwr_status & 0x04) status_str += "BAT_VOLTS_HIGH | ";
    if (pwr_status & 0x02) status_str += "BAT_VOLTS_LOW | ";
    if (pwr_status & 0x01) status_str += "AUX_CTL_ON | ";

    status_str += std::to_string(pwr_status);

    RCLCPP_INFO(logger, "%s", status_str.c_str());
}

#endif