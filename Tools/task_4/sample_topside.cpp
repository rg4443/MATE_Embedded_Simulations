#include <arpa/inet.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <ardupilotmega/mavlink.h>

#include <chrono>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

void send_message(int socket_fd, const mavlink_message_t &message)
{
    uint8_t bytes[MAVLINK_MAX_PACKET_LEN]{};
    const uint16_t length = mavlink_msg_to_send_buffer(bytes, &message);

    if (send(socket_fd, bytes, length, 0) != length) {
        throw std::runtime_error("could not send mavlink message");
    }
}

int main(int argc, char *argv[])
{
    try {
        std::string host;
        std::string action;
        int port = 0;

        for (int i = 1; i < argc; i += 2) {
            if (i + 1 >= argc) throw std::runtime_error("a flag is missing its value");

            const std::string flag = argv[i];
            const std::string value = argv[i + 1];

            if (flag == "--host") host = value;
            else if (flag == "--port") port = std::stoi(value);
            else if (flag == "--action") action = value;
            else throw std::runtime_error("unknown flag: " + flag);
        }

        if (host.empty()) throw std::runtime_error("host is required");
        if (port < 1 || port > 65535) throw std::runtime_error("port must be from 1 to 65535");
        if (action != "neutral") throw std::runtime_error("this example supports only --action neutral");

        const int socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
        if (socket_fd < 0) throw std::runtime_error("could not open udp socket");

        sockaddr_in destination{};
        destination.sin_family = AF_INET;
        destination.sin_port = htons(static_cast<uint16_t>(port));

        if (inet_pton(AF_INET, host.c_str(), &destination.sin_addr) != 1) {
            throw std::runtime_error("host must be a numeric ipv4 address");
        }
        if (connect(socket_fd, reinterpret_cast<sockaddr *>(&destination), sizeof(destination)) < 0) {
            throw std::runtime_error("could not set udp destination");
        }

        mavlink_message_t message{};
        mavlink_msg_heartbeat_pack(255, 190, &message, MAV_TYPE_GCS,
                                   MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
        send_message(socket_fd, message);

        uint8_t rov_system_id = 0;
        mavlink_status_t parse_status{};
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);

        while (std::chrono::steady_clock::now() < deadline && rov_system_id == 0) {
            pollfd ready{socket_fd, POLLIN, 0};
            if (poll(&ready, 1, 500) <= 0) continue;

            uint8_t bytes[2048]{};
            const ssize_t length = recv(socket_fd, bytes, sizeof(bytes), 0);
            if (length < 0) throw std::runtime_error("could not receive udp data");

            for (ssize_t i = 0; i < length; i++) {
                mavlink_message_t received{};
                if (!mavlink_parse_char(MAVLINK_COMM_0, bytes[i], &received, &parse_status)) continue;
                if (received.msgid != MAVLINK_MSG_ID_HEARTBEAT || received.compid != 1) continue;

                mavlink_heartbeat_t heartbeat{};
                mavlink_msg_heartbeat_decode(&received, &heartbeat);
                if (heartbeat.autopilot != MAV_AUTOPILOT_INVALID) rov_system_id = received.sysid;
            }
        }

        if (rov_system_id == 0) throw std::runtime_error("no ardusub heartbeat received");
        std::cout << "rov system id: " << static_cast<int>(rov_system_id) << '\n';

        for (int i = 0; i < 10; i++) {
            mavlink_msg_manual_control_pack(255, 190, &message, rov_system_id,
                                            0, 0, 500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
            send_message(socket_fd, message);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        std::cout << "sent neutral manual control for one second\n";
        close(socket_fd);
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "error: " << error.what() << '\n';
        return 1;
    }
}
