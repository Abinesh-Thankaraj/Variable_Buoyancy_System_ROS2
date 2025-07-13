#include <ignition/gazebo/System.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/Conversions.hh>
#include <ignition/msgs.hh>
#include <ignition/transport/Node.hh>
#include <ignition/math/Vector3.hh>

namespace buoyant_force
{
    class BuoyantForceSystem:
        public ignition::gazebo::System,
        public ignition::gazebo::ISystemConfigure,
        public ignition::gazebo::ISystemPreUpdate
    {
        public: BuoyantForceSystem() = default;
        public: ~BuoyantForceSystem() override = default;

        public: void Configure(const ignition::gazebo::Entity &_entity,
                            const std::shared_ptr<const sdf::Element> &_sdf,
                            ignition::gazebo::EntityComponentManager &_ecm,
                            ignition::gazebo::EventManager &_eventMgr) override
        {
            // Parse SDF parameters
            this->linkEntity = _entity;
            
            if (_sdf->HasElement("topic"))
                this->topic = _sdf->Get<std::string>("topic");
            
            if (_sdf->HasElement("min_force"))
                this->minForce = _sdf->Get<double>("min_force");
            
            if (_sdf->HasElement("max_force"))
                this->maxForce = _sdf->Get<double>("max_force");
            
            if (_sdf->HasElement("local_offset"))
                this->localOffset = _sdf->Get<ignition::math::Vector3d>("local_offset");
            
            // Initialize transport node
            this->node.Subscribe(this->topic, &BuoyantForceSystem::OnForceCmd, this);
        }

        public: void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                            ignition::gazebo::EntityComponentManager &_ecm) override
        {
            // Skip if force is zero or world is resetting
            if (std::abs(this->forceCmd) < 1e-6 || _info.paused)
                return;

            // Get link world pose
            auto pose = ignition::gazebo::worldPose(this->linkEntity, _ecm);
            
            // Calculate force application point in world frame
            auto offsetWorld = pose.Rot().RotateVector(this->localOffset);
            auto forcePoint = pose.Pos() + offsetWorld;
            
            // Apply world force at calculated point
            auto forceWorld = ignition::math::Vector3d(0, 0, this->forceCmd);
            ignition::gazebo::Link(this->linkEntity).AddWorldForce(
                _ecm, forceWorld, forcePoint);
        }

        private: void OnForceCmd(const ignition::msgs::Double &_msg)
        {
            // Clamp force command within limits
            this->forceCmd = std::clamp(_msg.data(), this->minForce, this->maxForce);
        }

        private: ignition::transport::Node node;
        private: ignition::gazebo::Entity linkEntity;
        private: std::string topic{"buoyant_force/cmd"};
        private: double forceCmd{0.0};
        private: double minForce{-40.0};
        private: double maxForce{40.0};
        private: ignition::math::Vector3d localOffset{0, 0, 0};
    };
}

// Register plugin
IGNITION_ADD_PLUGIN(
    buoyant_force::BuoyantForceSystem,
    ignition::gazebo::System,
    buoyant_force::BuoyantForceSystem::ISystemConfigure,
    buoyant_force::BuoyantForceSystem::ISystemPreUpdate
)
