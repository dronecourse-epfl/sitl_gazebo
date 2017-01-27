#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo {

class GazeboTruckPlugin : public ModelPlugin {
 public:
  GazeboTruckPlugin()
      : ModelPlugin()
        {}
  ~GazeboTruckPlugin();

  void Publish();

 protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo& /*_info*/);

 private:
  physics::ModelPtr model_;
  physics::WorldPtr world_;
  common::Time last_time_;

  float alpha_;             // trajectory parameter

  float length_ = 10.0f; // half the straight segment of the trajectory
  float radius_ = 10.0f;

  // Pointer to the update event connection
  private: event::ConnectionPtr updateConnection_;
};
}
