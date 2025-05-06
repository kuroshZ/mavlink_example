

#include <asio.hpp>
#include <chrono>
#include <cstring>
#include <fmt/core.h>
#include <map>
#include <mavlink.h>
#include <span>
#include <thread>
#include <type_traits>

#include "helpers.hpp"
#include "main.hpp"
#include "mavlink_msg_command_long.h"
#include "mavlink_msg_gimbal_device_attitude_status.h"
using namespace std::chrono_literals;


// uint16_t mavlink_msg_param_ext_set_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_param_ext_set_t* param_ext_set)
//
// using MavCmd_TO_EncodeFunc_t = std::map<>
MavlinkClient::MavlinkClient(asio::io_context &io_context, const std::string &host, const std::string &port)
    : socket_(std::make_unique<tcp::socket>(io_context)), serial_port_(nullptr), recv_buffer_(0)
{
    tcp::resolver resolver(io_context);
    endpoints_ = resolver.resolve(host, port);
    protocol_ = CommPoro::TCP;
}

MavlinkClient::~MavlinkClient()
{
    disconnect();
}

MavlinkClient::MavlinkClient(asio::io_context &io_context, const std::string &serial_port, unsigned int baud_rate)
    : socket_(nullptr), serial_port_(std::make_unique<asio::serial_port>(io_context)), recv_buffer_(0)
{
    serial_port_->open(serial_port);
    serial_port_->set_option(asio::serial_port_base::baud_rate(baud_rate));
    serial_port_->set_option(asio::serial_port_base::character_size(8));
    serial_port_->set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
    serial_port_->set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));
    serial_port_->set_option(asio::serial_port_base::flow_control(asio::serial_port_base::flow_control::none));

    protocol_ = CommPoro::SERIAL;
}

CommPoro MavlinkClient::protocol()
{
    return protocol_;
}

bool MavlinkClient::start(uint8_t own_sys_id, uint8_t own_comp_id, const mavlink_heartbeat_t &heartbeatmsg)
{
    own_sys_id_ = own_sys_id;
    own_comp_id_ = own_comp_id;
    set_target_from_recieved_ = true;
    auto const constexpr tries = 5;
    auto success = start_connection();
    if (success)
    {
        receive_messages();
    }
    send_heartbeat_message(heartbeatmsg);

    uint32_t count = 0;
    while (!first_heartbeat_received && count++ < tries)
    {
        fmt::print("waiting for receiving the heartbeat from the device ...\n");
        std::this_thread::sleep_for(2s);
    }

    return first_heartbeat_received;
}

void MavlinkClient::start(uint8_t own_sys_id, uint8_t own_comp_id, uint8_t target_sys_id, uint8_t target_comp_id)
{
    own_sys_id_ = own_sys_id;
    own_comp_id_ = own_comp_id;
    target_sys_id_ = target_sys_id;
    target_comp_id_ = target_comp_id;
    set_target_from_recieved_ = false;

    auto success = start_connection();
    if (success)
    {
        receive_messages();
    }
}


bool MavlinkClient::start_connection()
{
    if (socket_ != nullptr)
    {
        asio::connect(*socket_, endpoints_);
        fmt::print("Connecting to MAVLink device via LAN ...\n");
        return true;
    }
    else if (serial_port_ != nullptr)
    {
        if (serial_port_->is_open())
        {
            fmt::print("Serial port opened successfully\n");
            return true;
        }
        else
        {
            fmt::print("Failed to open serial port\n");
            return false;
        }
    }
    return false;
}

void MavlinkClient::disconnect()
{
    if (socket_ != nullptr)
    {
        socket_->close();
        fmt::print("Disconnected from MAVLink tcp device\n");
    }
    if (serial_port_ != nullptr)
    {
        serial_port_->close();

        fmt::print("Disconnected from MAVLink serial device\n");
    }
}


void print_buffer(uint8_t *buffer, size_t len)
{
    // Print the message as hex byte
    std::ostringstream oss;
    for (uint16_t i = 0; i < len; ++i)
    {
        oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(buffer[i]);
        if (i < len - 1)
        {
            oss << ' ';
        }
    }

    fmt::print("Message in hex: {}\n", oss.str());
}


void MavlinkClient::request_read_gimbal_mode()
{
    mavlink_param_ext_request_read_t param_request{
        .param_index = std::numeric_limits<int16_t>::quiet_NaN(), // Set to -1 to use param_id instead of index
        .target_system = target_sys_id_,
        .target_component = target_comp_id_,
        .param_id = "GB_MODE", // Parameter ID for gimbal mode
    };

    mavlink_message_t message;
    mavlink_msg_param_ext_request_read_encode(own_sys_id_, own_comp_id_, &message, &param_request);

    fmt::print("Requesting gimbal mode parameter: {}\n", param_request.param_id);
    send_mavlink_message(message);
}


void MavlinkClient::set_gimbal_mode()
{
    mavlink_param_ext_set_t param_ext{
        .target_system = target_sys_id_,
        .target_component = target_comp_id_,
        .param_id = "GB_MODE",
        .param_value = "1", // lock : 1 off: 0 follow: 2 mapping 3 reset 4
        .param_type = MAV_PARAM_EXT_TYPE_UINT8,
    };

    mavlink_message_t message;
    mavlink_msg_param_ext_set_encode(own_sys_id_, own_comp_id_, &message, &param_ext);
    fmt::print("sending gimbal mode: {} \n ", param_ext.param_value);
    send_mavlink_message(message);
}


void MavlinkClient::send_camera_tracking_point(float point_x, float point_y, float radius)
{
    mavlink_message_t msg;
    mavlink_command_long_t cmd;
    cmd.confirmation = 0;
    cmd.target_system = target_sys_id_;
    cmd.target_component = target_comp_id_;
    cmd.command = MAV_CMD_CAMERA_TRACK_POINT;
    cmd.param1 = point_x;
    cmd.param2 = point_y;
    cmd.param3 = radius;
    cmd.param4 = target_comp_id_;
    cmd.param5 = 0.0f;
    cmd.param6 = 0.0f;
    cmd.param7 = 0.0f;

    mavlink_msg_command_long_encode(own_sys_id_, own_comp_id_, &msg, &cmd);
    send_mavlink_message(msg);
}

void MavlinkClient::send_camera_tracking_rec()
{
}


void MavlinkClient::set_gimbal_encoder_param()
{

    mavlink_param_set_t param_set = {.param_value = 1.0F,
                                     .target_system = target_sys_id_,
                                     .target_component = target_comp_id_,
                                     .param_id = "GYRO_LPF",
                                     .param_type = MAVLINK_TYPE_UINT16_T

    };
    mavlink_message_t message = {};
    // mavlink_msg_param_set_encode(_system.sysid, _system.compid, &message, &param_set);
    mavlink_msg_param_set_encode_chan(own_sys_id_, own_comp_id_, 0, &message, &param_set);
    fmt::print("set gimbal param GYRO_LPF \n");
    send_mavlink_message(message);
}

void MavlinkClient::set_gimbal_home()
{
    // auto q = quaternion_from_euler_degrees(0.0F, pitch, yaw);
    mavlink_message_t msg;
    mavlink_gimbal_device_set_attitude_t set_attitude{
        .q = {NAN, NAN, NAN, NAN},
        .angular_velocity_x = 0,
        .angular_velocity_y = 0,
        .angular_velocity_z = 0,
        .flags = GIMBAL_DEVICE_FLAGS_NEUTRAL,
        .target_system = target_sys_id_,
        .target_component = target_comp_id_,
    };
    mavlink_msg_gimbal_device_set_attitude_encode_chan(own_sys_id_, own_comp_id_, 0, &msg, &set_attitude);
    // mavlink_msg_gimbal_device_set_attitude_encode(0, 255, &msg, &set_attitude);
    send_mavlink_message(msg);
};

void MavlinkClient::set_gimbal_rate(float pitch, float yaw)
{
    // auto q = quaternion_from_euler_degrees(0.0F, pitch, yaw);
    mavlink_message_t msg;
    mavlink_gimbal_device_set_attitude_t set_attitude;
    set_attitude.target_system = target_sys_id_;
    set_attitude.target_component = target_comp_id_;
    set_attitude.flags = GIMBAL_DEVICE_FLAGS_ROLL_LOCK | GIMBAL_DEVICE_FLAGS_PITCH_LOCK | GIMBAL_DEVICE_FLAGS_YAW_LOCK;
    set_attitude.q[0] = NAN;
    set_attitude.q[1] = NAN;
    set_attitude.q[2] = NAN;
    set_attitude.q[3] = NAN;
    // std::copy(std::begin(q), std::begin(q) + 4, set_attitude.q);
    set_attitude.angular_velocity_x = 0;
    set_attitude.angular_velocity_y = pitch;
    set_attitude.angular_velocity_z = yaw;

    // mavlink_euler_to_quaternion(to_rad(0.0F), to_rad(pitch), to_rad(yaw), set_attitude.q);

    mavlink_msg_gimbal_device_set_attitude_encode_chan(own_sys_id_, own_comp_id_, 0, &msg, &set_attitude);
    // mavlink_msg_gimbal_device_set_attitude_encode(0, 255, &msg, &set_attitude);
    send_mavlink_message(msg);
}
void MavlinkClient::set_gimbal_attitude(float pitch, float yaw)
{
    mavlink_message_t msg;
    mavlink_gimbal_device_set_attitude_t set_attitude;
    set_attitude.target_system = target_sys_id_;
    set_attitude.target_component = target_comp_id_;
    set_attitude.flags = GIMBAL_DEVICE_FLAGS_ROLL_LOCK | GIMBAL_DEVICE_FLAGS_PITCH_LOCK | GIMBAL_DEVICE_FLAGS_YAW_LOCK |
                         GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME;
    set_attitude.angular_velocity_x = NAN;
    set_attitude.angular_velocity_y = NAN;
    set_attitude.angular_velocity_z = NAN;

    mavlink_euler_to_quaternion(to_rad(0.0F), to_rad(pitch), to_rad(yaw), set_attitude.q);

    mavlink_msg_gimbal_device_set_attitude_encode(own_sys_id_, own_comp_id_, &msg, &set_attitude);
    // mavlink_msg_gimbal_device_set_attitude_encode(0, 255, &msg, &set_attitude);
    send_mavlink_message(msg);
}

void MavlinkClient::do_mount(float pitch, float yaw)
{

    mavlink_message_t msg;
    mavlink_command_long_t cmd;
    cmd.confirmation = 0;
    cmd.target_system = target_sys_id_;
    cmd.target_component = target_comp_id_;
    cmd.command = MAV_CMD_DO_MOUNT_CONTROL;
    cmd.param1 = pitch;
    cmd.param2 = 0.0f;
    cmd.param3 = yaw;
    cmd.param4 = 0.0f;
    cmd.param5 = 0.0f;
    cmd.param6 = 0.0f;
    cmd.param7 = MAV_MOUNT_MODE_MAVLINK_TARGETING;

    mavlink_msg_command_long_encode(own_sys_id_, own_comp_id_, &msg, &cmd);
    send_mavlink_message(msg);
}

void MavlinkClient::send_heartbeat_message(mavlink_heartbeat_t heartbeat)
{
    std::thread([this, heartbeat]() {
        while (true)
        {
            mavlink_message_t msg;
            mavlink_msg_heartbeat_encode(own_sys_id_, own_comp_id_, &msg, &heartbeat);
            send_mavlink_message(msg);

            std::this_thread::sleep_for(.95s);
        }
    }).detach();
}

void MavlinkClient::request_gimbal_info()
{

    mavlink_message_t msg;
    mavlink_msg_command_long_pack(own_sys_id_,
                                  own_comp_id_,
                                  &msg,
                                  target_sys_id_,
                                  target_comp_id_,
                                  MAV_CMD_REQUEST_MESSAGE,
                                  0,
                                  283, // GIMBAL_DEVICE_INFORMATION message ID
                                  0,
                                  0,
                                  0,
                                  0,
                                  0,
                                  0);

    send_mavlink_message(msg);
}


void MavlinkClient::send_mavlink_message(mavlink_message_t &msg)
{
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);

    std::lock_guard<std::mutex> lock(write_mutex_);
    if (protocol() == CommPoro::TCP)
    {
        asio::write(*socket_, asio::buffer(buffer, len));
    }
    else if (protocol() == CommPoro::SERIAL)
    {
        asio::write(*serial_port_, asio::buffer(buffer, len));
    }
}


// void MavlinkClient::send_param_ext_read_req_sent()
// {
//
//
//     mavlink_message_t msg;
//     mavlink_param_ext_request_read_t param_ext;
//     param_ext.target_system = target_sys_id_;
//     param_ext.target_component = target_comp_id_;
//     std::string param_id = "YAW ANGLE";
//     std::copy(param_id.begin(), param_id.end(), param_ext.param_id);
//     param_ext.param_id[param_id.size()] = '\0';
//     param_ext.param_index =
//         -1; //Parameter index. Set to -1 to use the Parameter ID field as identifier (else param_id will be ignored)
//
//
//     mavlink_msg_param_ext_request_read_encode(1, MAV_COMP_ID_ONBOARD_COMPUTER, &msg, &param_ext);
//     send_mavlink_message(msg);
// }

void MavlinkClient::send_zoom_request(float zoom_level)
{
    //cam response with CAMERA_SETTINGS
    mavlink_message_t msg;
    mavlink_command_long_t cmd;
    cmd.target_system = target_sys_id_;
    cmd.target_component = target_comp_id_;
    cmd.confirmation = 0;
    cmd.command = MAV_CMD_SET_CAMERA_ZOOM;
    cmd.param1 = CAMERA_ZOOM_TYPE::ZOOM_TYPE_RANGE;
    cmd.param2 = zoom_level;
    cmd.param3 = 0; //all cameras
    cmd.param4 = 0;
    cmd.param5 = 0;
    cmd.param6 = 0;
    cmd.param7 = 0;

    mavlink_msg_command_long_encode(own_sys_id_, own_comp_id_, &msg, &cmd);
    send_mavlink_message(msg);
}

void MavlinkClient::send_param_ext_request_list()


{
    mavlink_message_t msg;
    mavlink_param_ext_request_list_t param_list;
    param_list.target_system = target_sys_id_;
    param_list.target_component = target_comp_id_;
    mavlink_msg_param_ext_request_list_encode(own_sys_id_, own_comp_id_, &msg, &param_list);
    send_mavlink_message(msg);
}

void MavlinkClient::receive_messages()
{
    if (protocol() == CommPoro::TCP)
    {

        start_receiver_tcp();
    }
    else if (protocol() == CommPoro::SERIAL)
    {
        start_receiver_serial();
    }
}

void MavlinkClient::start_receiver_serial()
{
    serial_port_->async_read_some(asio::buffer(&recv_buffer_, 1), [this](std::error_code ec, std::size_t /*not_used*/) {
        if (!ec)
        {
            mavlink_message_t msg;
            mavlink_status_t status;

            if (mavlink_parse_char(MAVLINK_COMM_0, recv_buffer_, &msg, &status) != 0U)
            {
                if (!first_heartbeat_received)
                {
                    first_heartbeat_received = true;
                }

                if (!target_set_ && set_target_from_recieved_)
                {
                    target_sys_id_ = msg.sysid;
                    target_comp_id_ = msg.compid;
                    target_set_ = true;
                    fmt::print("received target sys and targe comp ids: {} , {} ", target_sys_id_, target_comp_id_);
                }
                handle_message(msg);
            }
            start_receiver_serial();
        }
        else
        {
            fmt::print("read buffer error: {}\n", ec.message());
        }
    });
}

void MavlinkClient::start_receiver_tcp()
{
    socket_->async_read_some(asio::buffer(&recv_buffer_, 1),
                             [this](std::error_code ec, [[maybe_unused]] std::size_t length) {
                                 if (!ec)
                                 {
                                     mavlink_message_t msg;
                                     mavlink_status_t status;
                                     if (mavlink_parse_char(MAVLINK_COMM_0, recv_buffer_, &msg, &status) != 0U)
                                     {
                                         handle_message(msg);
                                     }
                                     start_receiver_tcp();
                                 }
                                 else
                                 {
                                     fmt::print("read buffer error: {}\n", ec.message());
                                 }
                             });
}

void MavlinkClient::handle_message(const mavlink_message_t &msg)
{
    switch (msg.msgid)
    {
    case MAVLINK_MSG_ID_HEARTBEAT:
    {
        mavlink_heartbeat_t heartbeat{};
        mavlink_msg_heartbeat_decode(&msg, &heartbeat);
        // fmt::print("heartbit message received status active: {},  \n system_id:{} , component_id:{} \n ",
        //            heartbeat.system_status == MAV_STATE_ACTIVE,
        //            msg.sysid,
        //            msg.compid);
        break;
    }

    case MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS:
    {
        // mavlink_gimbal_manager_status_t status;
        // mavlink_msg_gimbal_manager_status_decode(&msg, &status);
        // fmt::print("gimbal status, id: {} flags: {} \n", status.gimbal_device_id, status.flags);

        break;
    }
    case MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS:
    {
        //     mavlink_gimbal_device_set_attitude_t attitude;
        //     mavlink_msg_gimbal_device_set_attitude_decode(&msg, &attitude);
        //     fmt::print(
        //         "gimbal status: \n target_system: {}, target_component:{} \n  flag:  {}, \n q:[w:{}, i:{}, j:{} , k:{}] \n",
        //         attitude.target_system,
        //         attitude.target_component,
        //         attitude.flags,
        //         attitude.q[0],
        //         attitude.q[1],
        //         attitude.q[2],
        //         attitude.q[3]);
        //
        break;
    }
        // case MAVLINK_MSG_ID_PARAM_EXT_VALUE:
        // {
        //     mavlink_param_ext_value_t param_ext_value;
        //     mavlink_msg_param_ext_value_decode(&msg, &param_ext_value);
        //     float value{0};
        //     std::memcpy(&value, &param_ext_value.param_value, sizeof(float));
        //
        //     const char *param_value = param_ext_value.param_value;
        //     // auto values = recover_values<float>(std::span<const char>(param_value, 1));
        //     //
        //     // fmt::print("orig: {} recover_values: \n", value);
        //     // for (auto val : values)
        //     // {
        //     //     fmt::print("val: {}", val);
        //     // }
        //
        //     fmt::print("\n Received PARAM_EXT_VALUE: total_count:{},param_idx:{} -> id: {} val: {}\n",
        //                param_ext_value.param_count,
        //                param_ext_value.param_index,
        //                param_ext_value.param_id,
        //                value);
        //
        //     break;
        // }

    case MAVLINK_MSG_ID_PARAM_EXT_ACK:
    {
        mavlink_param_ext_ack_t ack_msg;
        mavlink_msg_param_ext_ack_decode(&msg, &ack_msg);
        float value{0};
        std::memcpy(&value, &ack_msg.param_value, sizeof(float));
        fmt::print("Received ext param ack: id: {} result: {} - val: {} \n",
                   ack_msg.param_id,
                   ack_msg.param_result,
                   value);
        break;
    }

    case MAVLINK_MSG_ID_CAMERA_SETTINGS:
    {
        mavlink_camera_settings_t cam_settings;

        mavlink_msg_camera_settings_decode(&msg, &cam_settings);
        auto zoomLevel = cam_settings.zoomLevel;
        auto focusLevel = cam_settings.focusLevel;
        auto time_boot_ms = cam_settings.time_boot_ms;
        fmt::print("Received cam_settings:\n mode_id: {} \n zoom_level: {} \n focusLevel: {} \n time_boot_ms: {} "
                   "\n device_id: {}",
                   cam_settings.mode_id,
                   zoomLevel,
                   focusLevel,
                   time_boot_ms,
                   cam_settings.camera_device_id);

        //error: cannot bind packed field ‘cam_settings.__mavlink_camera_setting
        // s_t::zoomLevel’ to ‘float&’
        // 190 | cam_settings.zoomLevel,

        break;
    }

    case MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION:
    {
        // Create a structure to hold the parsed message
        mavlink_gimbal_device_information_t gimbal_info;

        // Decode the incoming message into the structure
        mavlink_msg_gimbal_device_information_decode(&msg, &gimbal_info);

        // Print the parsed information using fmt::print
        fmt::print("Gimbal Device Information:\n");
        fmt::print("  Vendor Name: {}\n", gimbal_info.vendor_name);
        fmt::print("  Model Name: {}\n", gimbal_info.model_name);
        fmt::print("  Firmware Version: {}.{}.{}\n",
                   (gimbal_info.firmware_version >> 24) & 0xFF,
                   (gimbal_info.firmware_version >> 16) & 0xFF,
                   (gimbal_info.firmware_version >> 8) & 0xFF);
        fmt::print("  Hardware Version: {}.{}.{}\n",
                   (gimbal_info.hardware_version >> 24) & 0xFF,
                   (gimbal_info.hardware_version >> 16) & 0xFF,
                   (gimbal_info.hardware_version >> 8) & 0xFF);
        fmt::print("  UID: {}\n", gimbal_info.uid);
        fmt::print("gimbal_device_id: {}", gimbal_info.gimbal_device_id);
        fmt::print("ya Min Max Speed: {:.2f} - {:.2f} rad/s\n", gimbal_info.yaw_min, gimbal_info.yaw_max);
        break;
    }

    case MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS:
    {
        mavlink_camera_tracking_image_status_t tracking_status;
        mavlink_msg_camera_tracking_image_status_decode(&msg, &tracking_status);
        // print_camera_tracking_image_status(tracking_status);
        break;
    }
    case MAVLINK_MSG_ID_MOUNT_ORIENTATION:
    case MAVLINK_MSG_ID_SYS_STATUS:
    case MAVLINK_MSG_ID_RAW_IMU:
        break;


    default:
    {
        // fmt::print("Received message ID: {} \n", msg.msgid);
        break;
    }
    }
}


int main()
{

    try
    {
        asio::io_context io_context;
        // MavlinkClient client(io_context, "192.168.144.9", "2000");

        MavlinkClient client(io_context, "/dev/ttyUSB0", 115200);
        uint8_t own_sys_id = 1;
        uint8_t own_comp_id = MAV_TYPE_ONBOARD_CONTROLLER;
        // client.start(own_sys_id, own_comp_id, 1, MAV_COMP_ID_GIMBAL);


        //taregt will set from the first received message

        std::thread t([&io_context]() { io_context.run(); });


        mavlink_heartbeat_t heartbeat;
        heartbeat.type = own_comp_id;
        heartbeat.autopilot = MAV_AUTOPILOT_GENERIC;
        heartbeat.base_mode = 0;
        heartbeat.custom_mode = 0;
        heartbeat.system_status = MAV_STATE_ACTIVE;
        heartbeat.mavlink_version = 2;

        auto started = client.start(own_sys_id, own_comp_id, heartbeat);
        if (!started)
        {
            fmt::print("no heartbeat is received from the device \n");
            std::exit(-1);
        }

        std::this_thread::sleep_for(4s);
        // client.send_param_ext_request_list();
        // std::this_thread::sleep_for(1s);

        // client.set_gimbal_encoder_param();
        fmt::print("requresting gimbal information ... \n");
        // client.request_gimbal_info();

        // std::this_thread::sleep_for(3s);

        // fmt::print("setting gimbal home ... \n");
        // client.set_gimbal_home();

        std::this_thread::sleep_for(3s);


        fmt::print("\n\n \n setting attitudes ...  \n \n \n");
        client.do_mount(0.0F, 120.0F);
        std::this_thread::sleep_for(1s);

        fmt::print("\n\n \n setting tracking ...  \n \n \n");
        client.send_camera_tracking_point(.5, .57, .3);

        // fmt::print("setting gimbal home ... \n");
        // client.set_gimbal_mode();
        // client.set_gimbal_rate(0, 0);
        // std::this_thread::sleep_for(3s);
        // fmt::print("\n\n \n setting attitudes ...  \n \n \n");
        //
        // client.set_gimbal_attitude(-10.0F, 50.0F);
        // std::this_thread::sleep_for(5s);
        //
        //
        // fmt::print("\n\n \n setting attitudes ...  \n \n \n");
        // client.set_gimbal_attitude(0.0F, -60.0F);
        // std::this_thread::sleep_for(8s);
        //
        // fmt::print("\n\n \n setting attitudes ...  \n \n \n");
        // // client.set_gimbal_attitude(0.0F, 0.0F);
        // client.set_gimbal_attitude(0.0F, 0.0F);
        std::this_thread::sleep_for(8000s);
        t.join();
    }
    catch (std::exception &e)
    {
        fmt::print("Exception: {}\n", e.what());
    }

    return 0;
}
