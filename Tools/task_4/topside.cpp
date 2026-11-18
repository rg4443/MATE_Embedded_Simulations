#include <ardupilotmega/mavlink.h>

#include <arpa/inet.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

using Clock = std::chrono::steady_clock;

struct Settings {
    std::string host = "127.0.0.1";
    std::string action = "monitor";
    int port = 14680;
    int hold_ms = 2000;
};

Settings parse_args(int argc, char *argv[])
{
    Settings settings;

    for (int i = 1; i < argc; i += 2) {
        if (i + 1 >= argc) throw std::runtime_error("Missing value for " + std::string(argv[i]));

        const std::string option = argv[i];
        const std::string value = argv[i + 1];

        if (option == "--host") settings.host = value;
        else if (option == "--port") settings.port = std::stoi(value);
        else if (option == "--action") settings.action = value;
        else if (option == "--hold-ms") settings.hold_ms = std::stoi(value);
        else throw std::runtime_error("Unknown option: " + option);
    }

    if (settings.port < 1 || settings.port > 65535) throw std::runtime_error("Port must be from 1 to 65535");
    if (settings.hold_ms < 100 || settings.hold_ms > 10000) throw std::runtime_error("Hold time must be from 100 to 10000 ms");

    const std::array<std::string, 8> actions = {
        "monitor", "arm", "disarm", "forward", "yaw", "neutral", "demo", "endpoint-demo"
    };
    bool valid_action = false;

    for (const auto &action : actions) {
        if (settings.action == action) valid_action = true;
    }

    if (!valid_action) throw std::runtime_error("Action must be monitor, arm, disarm, forward, yaw, neutral, demo, or endpoint-demo");

    return settings;
}

class Link {
public:
    explicit Link(const Settings &settings) : allow_refused_(settings.action == "endpoint-demo")
    {
        fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (fd_ < 0) throw std::runtime_error("Could not open udp socket");

        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_port = htons(static_cast<uint16_t>(settings.port));

        if (inet_pton(AF_INET, settings.host.c_str(), &address.sin_addr) != 1) {
            close(fd_);
            throw std::runtime_error("Host must be a numeric ipv4 address");
        }

        if (connect(fd_, reinterpret_cast<const sockaddr *>(&address), sizeof(address)) < 0) {
            close(fd_);
            throw std::runtime_error("Could not connect udp socket");
        }
    }

    ~Link()
    {
        close(fd_);
    }

    Link(const Link &) = delete;
    Link &operator=(const Link &) = delete;

    bool send_message(const mavlink_message_t &message)
    {
        std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> buffer{};
        const uint16_t length = mavlink_msg_to_send_buffer(buffer.data(), &message);
        const ssize_t sent = send(fd_, buffer.data(), length, 0);

        if (sent == length) return true;
        if (sent < 0 && errno == ECONNREFUSED && allow_refused_) return false;

        throw std::runtime_error("Could not send mavlink message");
    }

    void send_heartbeat()
    {
        mavlink_message_t message{};
        mavlink_msg_heartbeat_pack(255, 190, &message, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
        send_message(message);
        last_heartbeat_ = Clock::now();
    }

    bool send_manual(uint8_t system_id, int16_t forward, int16_t yaw)
    {
        mavlink_message_t message{};
        mavlink_msg_manual_control_pack(255, 190, &message, system_id, forward, 0, 500, yaw,
                                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        return send_message(message);
    }

    void send_command(uint8_t system_id, uint16_t command, float param1, float param2)
    {
        mavlink_message_t message{};
        mavlink_msg_command_long_pack(255, 190, &message, system_id, 1, command, 0,
                                      param1, param2, 0, 0, 0, 0, 0);
        send_message(message);
    }

    void request_telemetry(uint8_t system_id)
    {
        send_command(system_id, MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_ATTITUDE, 100000);
        send_command(system_id, MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 100000);
    }

    bool receive(int timeout_ms, mavlink_message_t &message)
    {
        if (Clock::now() - last_heartbeat_ >= std::chrono::seconds(1)) send_heartbeat();
        if (!pending_.empty()) {
            message = pending_.front();
            pending_.pop_front();
            return true;
        }

        pollfd descriptor{fd_, POLLIN, 0};
        const int result = poll(&descriptor, 1, timeout_ms);

        if (result < 0) throw std::runtime_error("Could not poll udp socket");
        if (result == 0) return false;

        std::array<uint8_t, 2048> buffer{};
        const ssize_t length = recv(fd_, buffer.data(), buffer.size(), 0);

        if (length < 0 && errno == ECONNREFUSED && allow_refused_) return false;
        if (length < 0) throw std::runtime_error("Could not receive udp packet");

        for (ssize_t i = 0; i < length; i++) {
            mavlink_message_t parsed{};
            if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &parsed, &status_)) pending_.push_back(parsed);
        }

        if (!pending_.empty()) {
            message = pending_.front();
            pending_.pop_front();
            return true;
        }

        return false;
    }

private:
    int fd_ = -1;
    bool allow_refused_ = false;
    mavlink_status_t status_{};
    std::deque<mavlink_message_t> pending_;
    Clock::time_point last_heartbeat_{};
};

class Vehicle {
public:
    explicit Vehicle(Link &link) : link_(link) {}

    void discover()
    {
        link_.send_heartbeat();
        const auto deadline = Clock::now() + std::chrono::seconds(8);

        while (Clock::now() < deadline && system_id_ == 0) poll_once(100);
        if (system_id_ == 0) throw std::runtime_error("No ardusub heartbeat through the selected endpoint");

        link_.request_telemetry(system_id_);
        last_telemetry_ = Clock::now();
    }

    void arm(bool armed)
    {
        link_.send_command(system_id_, MAV_CMD_COMPONENT_ARM_DISARM, armed ? 1.0f : 0.0f, 0);
        std::cout << (armed ? "Arm requested" : "Disarm requested") << '\n';

        const auto requested_at = Clock::now();
        const auto deadline = Clock::now() + std::chrono::seconds(8);

        while (Clock::now() < deadline) {
            poll_once(100);
            if (seen_armed_state_ && last_vehicle_heartbeat_ >= requested_at && armed_ == armed) {
                std::cout << (armed ? "Armed heartbeat confirmed" : "Disarmed heartbeat confirmed") << '\n';
                return;
            }
        }

        throw std::runtime_error(armed ? "No armed heartbeat; inspect pre-arm text above" : "No disarmed heartbeat");
    }

    void input(int16_t forward, int16_t yaw, int duration_ms, const std::string &name, bool allow_loss = false)
    {
        std::cout << name << " input: x=" << forward << " r=" << yaw << " z=500 for " << duration_ms << " ms\n";

        const auto end = Clock::now() + std::chrono::milliseconds(duration_ms);
        auto next_send = Clock::now();
        int refused = 0;

        while (Clock::now() < end) {
            if (Clock::now() >= next_send) {
                if (!link_.send_manual(system_id_, forward, yaw)) refused++;
                next_send += std::chrono::milliseconds(100);
            }

            poll_once(20, allow_loss);
        }

        if (allow_loss) std::cout << "Refused udp packets: " << refused << '\n';
    }

    void endpoint_demo(int hold_ms)
    {
        if (!armed_) arm(true);

        input(400, 0, hold_ms, "Forward");
        input(0, 0, 1000, "Neutral");
        input(0, 400, hold_ms, "Yaw");
        input(0, 0, 5000, "Neutral");

        std::cout << "Remove the dedicated endpoint in the blueos web interface now\n";

        auto next_send = Clock::now();

        while (true) {
            if (Clock::now() >= next_send) {
                link_.send_manual(system_id_, 0, 0);
                next_send += std::chrono::milliseconds(100);
            }

            poll_once(20, true);

            if (Clock::now() - last_vehicle_heartbeat_ < std::chrono::milliseconds(1500) ||
                Clock::now() - last_telemetry_ < std::chrono::milliseconds(1500)) continue;
            if (!armed_) throw std::runtime_error("Vehicle disarmed before endpoint loss; repeat the sequence");

            break;
        }

        std::cout << "Vehicle telemetry lost; attempting fresh forward input\n";
        const auto lost_at = Clock::now();

        input(400, 0, hold_ms, "Forward", true);
        input(0, 0, 1000, "Neutral", true);

        if (last_vehicle_heartbeat_ > lost_at || last_telemetry_ > lost_at) {
            throw std::runtime_error("Vehicle telemetry returned during the removed-endpoint input");
        }

        std::cout << "No vehicle telemetry during the removed-endpoint input; check gazebo pose\n";
        std::cout << "Re-create the dedicated endpoint in the blueos web interface now\n";

        const auto restore_deadline = Clock::now() + std::chrono::seconds(120);
        const auto restore_started = Clock::now();

        while (Clock::now() < restore_deadline) {
            poll_once(100, true);

            if (last_vehicle_heartbeat_ > restore_started && last_telemetry_ > restore_started) break;
        }

        if (last_vehicle_heartbeat_ <= restore_started || last_telemetry_ <= restore_started) {
            throw std::runtime_error("No vehicle telemetry after the endpoint was restored");
        }

        std::cout << "Vehicle telemetry restored\n";
        if (!armed_) arm(true);

        input(400, 0, hold_ms, "Forward");
        input(0, 0, 1000, "Neutral");
        input(0, 400, hold_ms, "Yaw");
        input(0, 0, 1000, "Neutral");
        arm(false);
    }

    void monitor()
    {
        std::cout << "Monitoring telemetry; press ctrl+c to stop\n";
        while (true) poll_once(100);
    }

    bool armed() const { return armed_; }

private:
    void poll_once(int timeout_ms, bool allow_loss = false)
    {
        mavlink_message_t message{};
        if (!link_.receive(timeout_ms, message)) {
            if (!allow_loss) check_link();
            return;
        }

        if (message.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
            mavlink_heartbeat_t heartbeat{};
            mavlink_msg_heartbeat_decode(&message, &heartbeat);

            if (system_id_ == 0 && message.compid == 1 && heartbeat.type == MAV_TYPE_SUBMARINE &&
                heartbeat.autopilot == MAV_AUTOPILOT_ARDUPILOTMEGA) {
                system_id_ = message.sysid;
                std::cout << "Ardusub system id: " << static_cast<int>(system_id_) << '\n';
            }

            if (message.sysid == system_id_ && message.compid == 1) {
                armed_ = (heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
                seen_armed_state_ = true;
                last_vehicle_heartbeat_ = Clock::now();
                std::cout << "Vehicle heartbeat: " << (armed_ ? "armed" : "disarmed") << '\n';
            }
        }

        if (message.sysid != system_id_ || system_id_ == 0) return;

        if (message.msgid == MAVLINK_MSG_ID_STATUSTEXT) {
            mavlink_statustext_t status{};
            mavlink_msg_statustext_decode(&message, &status);
            std::cout << "Status: " << std::string(status.text, strnlen(status.text, sizeof(status.text))) << '\n';
        }

        if (message.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
            mavlink_command_ack_t ack{};
            mavlink_msg_command_ack_decode(&message, &ack);
            if (ack.command == MAV_CMD_COMPONENT_ARM_DISARM) {
                std::cout << "Arm command result: " << static_cast<int>(ack.result) << '\n';
            }
        }

        if (message.msgid == MAVLINK_MSG_ID_ATTITUDE) {
            mavlink_attitude_t attitude{};
            mavlink_msg_attitude_decode(&message, &attitude);
            constexpr double radians_to_degrees = 57.29577951308232;
            std::cout << std::fixed << std::setprecision(1)
                      << "Attitude deg: roll=" << attitude.roll * radians_to_degrees
                      << " pitch=" << attitude.pitch * radians_to_degrees
                      << " yaw=" << attitude.yaw * radians_to_degrees << '\n';
            last_telemetry_ = Clock::now();
        }

        if (message.msgid == MAVLINK_MSG_ID_SERVO_OUTPUT_RAW) {
            mavlink_servo_output_raw_t outputs{};
            mavlink_msg_servo_output_raw_decode(&message, &outputs);
            std::cout << "Servo output raw us: " << outputs.servo1_raw << ' ' << outputs.servo2_raw
                      << ' ' << outputs.servo3_raw << ' ' << outputs.servo4_raw << ' '
                      << outputs.servo5_raw << ' ' << outputs.servo6_raw << ' '
                      << outputs.servo7_raw << ' ' << outputs.servo8_raw << '\n';
            last_telemetry_ = Clock::now();
        }

        if (!allow_loss) check_link();
    }

    void check_link() const
    {
        if (last_vehicle_heartbeat_ != Clock::time_point{} &&
            Clock::now() - last_vehicle_heartbeat_ > std::chrono::seconds(3)) {
            throw std::runtime_error("Vehicle heartbeat lost through the selected endpoint");
        }

        if (last_telemetry_ != Clock::time_point{} &&
            Clock::now() - last_telemetry_ > std::chrono::seconds(3)) {
            throw std::runtime_error("Attitude and servo telemetry lost through the selected endpoint");
        }
    }

    Link &link_;
    uint8_t system_id_ = 0;
    bool armed_ = false;
    bool seen_armed_state_ = false;
    Clock::time_point last_vehicle_heartbeat_{};
    Clock::time_point last_telemetry_{};
};

int main(int argc, char *argv[])
{
    try {
        std::cout << std::unitbuf;

        const Settings settings = parse_args(argc, argv);
        Link link(settings);
        Vehicle vehicle(link);
        vehicle.discover();

        if (settings.action == "arm") vehicle.arm(true);
        if (settings.action == "disarm") vehicle.arm(false);
        if (settings.action == "monitor") vehicle.monitor();
        if (settings.action == "endpoint-demo") vehicle.endpoint_demo(settings.hold_ms);

        if (settings.action == "forward" || settings.action == "demo") {
            if (settings.action == "demo" && !vehicle.armed()) vehicle.arm(true);
            if (!vehicle.armed()) throw std::runtime_error("Vehicle is disarmed; run --action arm first");
            vehicle.input(400, 0, settings.hold_ms, "Forward");
            vehicle.input(0, 0, 1000, "Neutral");
        }

        if (settings.action == "yaw" || settings.action == "demo") {
            if (!vehicle.armed()) throw std::runtime_error("Vehicle is disarmed; run --action arm first");
            vehicle.input(0, 400, settings.hold_ms, "Yaw");
            vehicle.input(0, 0, 1000, "Neutral");
        }

        if (settings.action == "neutral") vehicle.input(0, 0, 1000, "Neutral");
        if (settings.action == "demo") vehicle.arm(false);

        return 0;
    } catch (const std::exception &error) {
        std::cerr << "Error: " << error.what() << '\n';
        return 1;
    }
}
