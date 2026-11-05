#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/int32.pb.h>
#include <gz/msgs/int32_v.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace Embedded {

class VirtualESC final : public gz::sim::System,
                         public gz::sim::ISystemConfigure,
                         public gz::sim::ISystemPreUpdate {
public:
    void Configure(const gz::sim::Entity &, const std::shared_ptr<const sdf::Element> &sdf,
                   gz::sim::EntityComponentManager &, gz::sim::EventManager &) override
    {
        const std::string model = sdf->Get<std::string>("namespace");
        const int count = sdf->Get<int>("motors");

        min_pwm = sdf->Get<int>("min_pwm", 1100).first;
        max_pwm = sdf->Get<int>("max_pwm", 1900).first;
        deadband_us = sdf->Get<int>("deadband_us", 0).first;
        timeout_s = sdf->Get<double>("timeout_s", 0.5).first;
        delay_s = sdf->Get<double>("delay_s", 0.0).first;

        drop_input_after_s = sdf->Get<double>("drop_input_after_s", -1.0).first;
        stuck_neutral_motor = sdf->Get<int>("stuck_neutral_motor", 0).first;
        
        if (count < 1 || count > 16 || min_pwm < 1100 || max_pwm > 1900 ||
            min_pwm >= 1500 || max_pwm <= 1500 || deadband_us < 0 ||
            timeout_s <= 0 || delay_s < 0 || stuck_neutral_motor < 0 ||
            stuck_neutral_motor > count) {

            throw std::runtime_error("invalid virtual ESC configuration");
        }

        channels.resize(count);

        const std::function<void(const gz::msgs::Int32 &)> fault_callback = [this, count](const gz::msgs::Int32 &message) {
            if (message.data() < 0 || message.data() > count) return;

            std::lock_guard<std::mutex> guard(mutex);
            stuck_neutral_motor = message.data();
        };

        node.Subscribe("/" + model + "/esc/stuck_neutral_motor", fault_callback);
        
        for (int index = 0; index < count; index++) {
            const std::string prefix = "/" + model + "/motor_" + std::to_string(index + 1);
            const std::function<void(const gz::msgs::Int32 &)> callback = [this, index](const gz::msgs::Int32 &message) {
                std::lock_guard<std::mutex> guard(mutex);
                Channel &channel = channels[index];
                const int pwm = message.data();

                if (pwm < 1100 || pwm > 1900) {
                    channel.invalidate(pwm);
                    channel.pending = false;
                    return;
                }
                
                channel.pending_pwm = pwm;
                channel.pending = true;
            };

            node.Subscribe(prefix + "/pwm_us", callback);
            channels[index].command = node.Advertise<gz::msgs::Int32>(prefix);
            channels[index].validity = node.Advertise<gz::msgs::Boolean>(prefix + "/output_valid");
            channels[index].sample = node.Advertise<gz::msgs::Int32_V>(prefix + "/esc_sample");
        }
    }

    void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &) override
    {
        if (info.paused) return;

        const double now = std::chrono::duration<double>(info.simTime).count();
        
        std::lock_guard<std::mutex> guard(mutex);

        for (size_t index = 0; index < channels.size(); index++) {
            Channel &channel = channels[index];

            if (channel.has_source && (now < channel.last_source_s || now - channel.last_source_s >= timeout_s)) {
                channel.invalidate(0);
            }

            if (channel.pending) {
                if (!channel.has_first_input) {
                    channel.first_input_s = now;
                    channel.has_first_input = true;
                }

                if (drop_input_after_s < 0 || now - channel.first_input_s < drop_input_after_s) {
                    channel.queue.push_back({now + delay_s, channel.pending_pwm});
                    channel.last_source_s = now;
                    channel.has_source = true;
                }

                channel.pending = false;
            }

            while (!channel.queue.empty() && channel.queue.front().first <= now) {
                channel.input_pwm = channel.queue.front().second;
                channel.has_input = true;
                channel.queue.pop_front();
            }

            const bool valid = channel.has_input;

            int pwm = 1500;
            
            if (valid && static_cast<int>(index + 1) != stuck_neutral_motor) {
                pwm = std::clamp(channel.input_pwm, min_pwm, max_pwm);
                
                if (std::abs(pwm - 1500) <= deadband_us) {
                    pwm = 1500;
                }
            }
            
            gz::msgs::Int32 output;

            output.set_data(static_cast<int32_t>(std::lround(255.0 * (pwm - 1100) / 800.0)));
            
            channel.command.Publish(output);

            gz::msgs::Boolean state;
            
            state.set_data(valid);

            channel.validity.Publish(state);

            gz::msgs::Int32_V sample;
            sample.add_data(channel.input_pwm);
            sample.add_data(output.data());
            sample.add_data(valid);
            channel.sample.Publish(sample);
        }
    }

private:
    struct Channel {
        bool pending{false};
        bool has_source{false};
        double last_source_s{0};
        int pending_pwm{0};
        int input_pwm{0};
        bool has_input{false};
        bool has_first_input{false};
        double first_input_s{0};
        std::deque<std::pair<double, int>> queue;
        gz::transport::Node::Publisher command;
        gz::transport::Node::Publisher validity;
        gz::transport::Node::Publisher sample;

        void invalidate(int pwm)
        {
            queue.clear();
            input_pwm = pwm;
            has_input = false;
            has_source = false;
        }
    };

    gz::transport::Node node;

    std::mutex mutex;

    std::vector<Channel> channels;

    int min_pwm{1100};
    int max_pwm{1900};
    int deadband_us{0};
    int stuck_neutral_motor{0};

    double timeout_s{0.5};
    double delay_s{0};
    double drop_input_after_s{-1};
};

}

GZ_ADD_PLUGIN(Embedded::VirtualESC, gz::sim::System,
              Embedded::VirtualESC::ISystemConfigure,
              Embedded::VirtualESC::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(Embedded::VirtualESC, "Embedded::VirtualESC")
