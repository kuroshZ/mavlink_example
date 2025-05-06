

#pragma once
#include "mavlink.h"
#include "string"
#include <cstdint>

auto get_tracking_status_name(uint8_t status) -> std::string;
auto get_tracking_mode_name(uint8_t mode) -> std::string;
auto print_camera_tracking_image_status(const mavlink_camera_tracking_image_status_t &tracking_status) -> void;
