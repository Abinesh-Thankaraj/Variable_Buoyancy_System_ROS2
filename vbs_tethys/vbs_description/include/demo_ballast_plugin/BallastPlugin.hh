#ifndef BLUEROV2_BALLAST_PLUGIN_HH
#define BLUEROV2_BALLAST_PLUGIN_HH

#include <ignition/gazebo/System.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/double.pb.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

namespace ballast_plugin
{
    class BallastPlugin : public ignition::gazebo::System,
                          public ignition::gazebo::ISystemConfigure,
                          public ignition::gazebo::ISystemPreUpdate
    {
    public:
        BallastPlugin() = default;
        ~BallastPlugin() override = default;
        
        void Configure(const ignition::gazebo::Entity &_entity,
                      const std::shared_ptr<const sdf::Element> &_sdf,
                      ignition::gazebo::EntityComponentManager &_ecm,
                      ignition::gazebo::EventManager &_eventMgr) override;
                      
        void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                      ignition::gazebo::EntityComponentManager &_ecm) override;

    private:
        void OnCmd(const std_msgs::msg::Float64::SharedPtr _msg);
        
        ignition::gazebo::Entity entity_;
        double empty_mass_{0.5};
        double max_water_mass_{1.0};
        double current_water_mass_{0.0};
        double fluid_density_{1000.0};
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;
        rclcpp::Node::SharedPtr ros_node_;
        ignition::transport::Node ign_node_;
        std::mutex mutex_;
    };
}

#endif
