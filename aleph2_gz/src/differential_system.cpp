#include <gz/sim/System.hh>

namespace aleph2_gz
{

class DifferentialSystem
  : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemUpdate
{
public:
  void Configure(
    const gz::sim::Entity & _entity,
    const std::shared_ptr<const sdf::Element> & _sdf,
    gz::sim::EntityComponentManager & _ecm,
    gz::sim::EventManager & _eventMgr) override
  {
    gzmsg << "Configure";
  }

  void Update(
    const gz::sim::UpdateInfo & _info,
    gz::sim::EntityComponentManager & _ecm) override
  {
    if (_info.paused) {return;}
    gzmsg << "Update\n";
  }
};

}

#include <gz/plugin/Register.hh>

GZ_ADD_PLUGIN(
  aleph2_gz::DifferentialSystem,
  gz::sim::System,
  aleph2_gz::DifferentialSystem::ISystemConfigure,
  aleph2_gz::DifferentialSystem::ISystemUpdate)
