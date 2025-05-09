#pragma once
#include <asio.hpp>
#include <chrono>
#include <cstring>
#include <fmt/core.h>
#include <map>
#include <mavlink.h>
#include <memory>
#include <span>
#include <thread>
#include <type_traits>
#include <variant>

using asio::ip::tcp;


enum class CommPoro
{
    TCP = 0,
    SERIAL
};


auto to_rad(float deg) -> float
{

    constexpr float degrees_to_radians = M_PI / float(180);
    return deg * degrees_to_radians;
}


class MavlinkClient
{
public:
    // uint16_t mavlink_msg_param_ext_set_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_param_ext_set_t* param_ext_set)
    //
    // using MavCmd_TO_EncodeFunc_t = std::map<>
    explicit MavlinkClient(asio::io_context &io_context, const std::string &host, const std::string &port);

    explicit MavlinkClient(asio::io_context &io_context, const std::string &serial_port, unsigned int baud_rate);

    ~MavlinkClient();

    void start(uint8_t own_sys_id, uint8_t own_comp_id, uint8_t target_sys_id, uint8_t target_comp_id);
    bool start(uint8_t own_sys_id, uint8_t own_comp_id, const mavlink_heartbeat_t &heartbeatmsg);
    void disconnect();
    void send_param_ext_set();
    void request_gimbal_info();
    void set_mount_msg_interval();
    // void send_param_ext_read_req_sent();
    void send_zoom_request(float zoom_level);
    void send_param_ext_request_list();
    CommPoro protocol();

    void do_mount_configure();
    void do_mount(float pitch, float yaw);
    void set_gimbal_rate(float pitch, float yaw);
    void set_gimbal_home();
    void set_gimbal_attitude(float pitch, float yaw);
    void set_gimbal_encoder_param();
    void set_gimbal_mode();
    void send_camera_tracking_point(float point_x, float point_y, float radius);
    void send_camera_tracking_rec();
    void request_read_gimbal_mode();

    void send_mavlink_message(uint8_t *buffer, size_t len); // Replace with your send function
    void send_mavlink_message(mavlink_message_t &msg);

private:
    void receive_messages();
    void start_receiver_tcp();
    void start_receiver_serial();
    void handle_message(const mavlink_message_t &msg);
    bool start_connection();

    void send_heartbeat_message(mavlink_heartbeat_t msg);
    /* private member variables */
    std::unique_ptr<tcp::socket> socket_{};
    std::unique_ptr<asio::serial_port> serial_port_{};
    uint8_t recv_buffer_{};
    tcp::resolver::results_type endpoints_{};
    CommPoro protocol_;
    std::mutex write_mutex_;
    uint8_t own_sys_id_{};
    uint8_t own_comp_id_{};
    uint8_t target_sys_id_{};
    uint8_t target_comp_id_{};
    bool set_target_from_recieved_{false};
    bool target_set_{false};
    bool first_heartbeat_received{false};
};
