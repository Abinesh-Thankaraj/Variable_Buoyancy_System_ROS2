#include "ballast_plugin/BallastPlugin.hh"

#include <ignition/gazbeo/components/Inertial.hh>
#include <ignition/plugin/Register.hh>

using namespace ballast_plugin;

IGNITION_ADD_PLUGIN(
    BallastPlugin,
    ignition::gazebo::System,
    BallastPlugin::ISystemConfigure,
    BallastPlugin::ISystemPreUpdate)

void BallastPlugin::Configure(
    const ignition::gazebo::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    ignition::gazebo::EntityComponentManager &_ecm,
    ignition::gazebo::EventManager &/*_eventMgr*/)
{
    entity_ = _entity;
    
    // Get parameters from SDF
    if (_sdf->HasElement("empty_mass"))
        empty_mass_ = _sdf->Get<double>("empty_mass");
    
    if (_sdf->HasElement("max_water_mass"))
        max_water_mass_ = _sdf->Get<double>("max_water_mass");
    
    if (_sdf->HasElement("fluid_density"))
        fluid_density_ = _sdf->Get<double>("fluid_density");
    
    // Initialize ROS node
    ros_node_ = rclcpp::Node::make_shared("ballast_plugin");
    
    // Create subscriber for ballast commands
    sub_ = ros_node_->create_subscription<std_msgs::msg::Float64>(
        _sdf->Get<std::string>("topic", "ballast/cmd").first,
        10,
        std::bind(&BallastPlugin::OnCmd, this, std::placeholders::_1));
}

void BallastPlugin::PreUpdate(
    const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
    if (!_ecm.EntityHasComponentType(entity_, ignition::gazebo::components::Inertial::typeId))
        return;
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Update mass component
    auto inertial = _ecm.Component<ignition::gazebo::components::Inertial>(entity_);
    if (inertial)
    {
        ignition::math::MassMatrix3d massMatrix = inertial->Data().MassMatrix();
        massMatrix.SetMass(empty_mass_ + current_water_mass_);
        inertial->Data().SetMassMatrix(massMatrix);
    }
}

void BallastPlugin::OnCmd(const std_msgs::msg::Float64::SharedPtr _msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    current_water_mass_ = std::clamp(_msg->data, 0.0, max_water_mass_);
}
