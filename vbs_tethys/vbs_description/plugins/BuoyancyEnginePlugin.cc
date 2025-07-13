#include "BuoyancyEnginePlugin.hh"
#include <ignition/msgs.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/gazebo/components/Gravity.hh>
#include <ignition/math/Vector3.hh>

using namespace tethys;

void BuoyancyEnginePlugin::Configure(
    const ignition::gazebo::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    ignition::gazebo::EntityComponentManager &_ecm,
    ignition::gazebo::EventManager &)
{
  // Get parameters from SDF
  if (_sdf->HasElement("link_name")) {
    std::string linkName = _sdf->Get<std::string>("link_name");
    this->linkEntity = ignition::gazebo::worldEntity(_ecm);
    auto entities = ignition::gazebo::entitiesFromScopedName(linkName, _ecm, this->linkEntity);
    if (!entities.empty()) {
      this->linkEntity = *entities.begin();
    }
  }

  if (_sdf->HasElement("namespace")) {
    this->namespace_ = _sdf->Get<std::string>("namespace");
  }

  if (_sdf->HasElement("fluid_density")) {
    this->fluidDensity = _sdf->Get<double>("fluid_density");
  }

  if (_sdf->HasElement("min_volume")) {
    this->minVolume = _sdf->Get<double>("min_volume");
  }

  if (_sdf->HasElement("max_volume")) {
    this->maxVolume = _sdf->Get<double>("max_volume");
  }

  if (_sdf->HasElement("max_inflation_rate")) {
    this->maxInflationRate = _sdf->Get<double>("max_inflation_rate");
  }

  if (_sdf->HasElement("default_volume")) {
    this->currentVolume = _sdf->Get<double>("default_volume");
    this->targetVolume = this->currentVolume;
  }

  // Subscribe to buoyancy commands
  std::string topic = "/model/" + this->namespace_ + "/buoyancy_engine";
  this->node.Subscribe(topic, &BuoyancyEnginePlugin::OnCommand, this);
}

void BuoyancyEnginePlugin::OnCommand(const ignition::msgs::Double &_msg)
{
  this->targetVolume = std::clamp(_msg.data(), this->minVolume, this->maxVolume);
}

void BuoyancyEnginePlugin::PreUpdate(
    const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  if (_info.paused || this->linkEntity == ignition::gazebo::kNullEntity) {
    return;
  }

  // Gradually adjust volume towards target
  double volumeChange = std::clamp(
      this->targetVolume - this->currentVolume,
      -this->maxInflationRate * _info.dt.count() / 1e9,
      this->maxInflationRate * _info.dt.count() / 1e9
  );
  
  this->currentVolume += volumeChange;

  // Calculate buoyancy force (F = ρ * V * g)
  auto worldEntity = ignition::gazebo::worldEntity(_ecm);
  auto gravityComp = _ecm.Component<ignition::gazebo::components::Gravity>(worldEntity);
  auto gravity = gravityComp ? gravityComp->Data() : ignition::math::Vector3d(0, 0, -9.81);
  auto force = -gravity * this->fluidDensity * this->currentVolume;

  // Apply force at the link
  ignition::gazebo::Link link(this->linkEntity);
  link.AddWorldForce(_ecm, force);
}
