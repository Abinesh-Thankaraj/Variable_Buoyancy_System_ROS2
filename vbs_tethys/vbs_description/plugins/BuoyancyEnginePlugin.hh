#include <ignition/gazebo/System.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/double.pb.h>

namespace tethys
{
class BuoyancyEnginePlugin : public ignition::gazebo::System,
                             public ignition::gazebo::ISystemConfigure,
                             public ignition::gazebo::ISystemPreUpdate
{
public:
  void Configure(const ignition::gazebo::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 ignition::gazebo::EntityComponentManager &_ecm,
                 ignition::gazebo::EventManager &_eventMgr) override;
                 
  void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                 ignition::gazebo::EntityComponentManager &_ecm) override;

private:
  void OnCommand(const ignition::msgs::Double &_msg);

  ignition::transport::Node node;
  ignition::gazebo::Entity linkEntity;
  std::string namespace_;
  double currentVolume{0.0005};
  double targetVolume{0.0005};
  double minVolume{0.00008};
  double maxVolume{0.000955};
  double maxInflationRate{0.000003};
  double fluidDensity{1025};
};
}
