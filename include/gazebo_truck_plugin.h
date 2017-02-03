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


  float length_ = 120.0f; // half the straight segment of the trajectory
  float radius_ = 20.0f;

  int state_;            // state of state machine
  float alpha_;
  // Pointer to the update event connection
  private:
    // std::string namespace_;
    event::ConnectionPtr updateConnection_;
    // transport::PublisherPtr pose_pub_;
    // transport::NodePtr node_handle_;

    // vehicle_state_msgs::msgs::Pose pose_message_;
};
}
