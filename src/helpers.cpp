#include "helpers.hpp"
#include "fmt/core.h"

// Helper function to get the name of the tracking status
std::string get_tracking_status_name(uint8_t status)
{
    switch (status)
    {
    case CAMERA_TRACKING_STATUS_FLAGS_IDLE:
        return "Idle (Camera is not tracking)";
    case CAMERA_TRACKING_STATUS_FLAGS_ACTIVE:
        return "Active (Camera is tracking)";
    case CAMERA_TRACKING_STATUS_FLAGS_ERROR:
        return "Error (Camera tracking in error state)";
    default:
        return "Unknown";
    }
}

// Helper function to get the name of the tracking mode
std::string get_tracking_mode_name(uint8_t mode)
{
    switch (mode)
    {
    case CAMERA_TRACKING_MODE_NONE:
        return "None (Not tracking)";
    case CAMERA_TRACKING_MODE_POINT:
        return "Point (Target is a point)";
    case CAMERA_TRACKING_MODE_RECTANGLE:
        return "Rectangle (Target is a rectangle)";
    default:
        return "Unknown";
    }
}

void print_camera_tracking_image_status(const mavlink_camera_tracking_image_status_t &tracking_status)
{
    fmt::print("Camera Tracking Image Status:\n");
    fmt::print("  Point X: {:.2f}\n", tracking_status.point_x);
    fmt::print("  Point Y: {:.2f}\n", tracking_status.point_y);
    fmt::print("  Radius: {:.2f}\n", tracking_status.radius);
    fmt::print("  Rectangle Top X: {:.2f}\n", tracking_status.rec_top_x);
    fmt::print("  Rectangle Top Y: {:.2f}\n", tracking_status.rec_top_y);
    fmt::print("  Rectangle Bottom X: {:.2f}\n", tracking_status.rec_bottom_x);
    fmt::print("  Rectangle Bottom Y: {:.2f}\n", tracking_status.rec_bottom_y);
    fmt::print("  Tracking Status: {} ({})\n",
               tracking_status.tracking_status,
               get_tracking_status_name(tracking_status.tracking_status));
    fmt::print("  Tracking Mode: {} ({})\n",
               tracking_status.tracking_mode,
               get_tracking_mode_name(tracking_status.tracking_mode));
    fmt::print("  Target Data: {}\n", tracking_status.target_data);
    fmt::print("  Camera Device ID: {}\n", tracking_status.camera_device_id);
}
