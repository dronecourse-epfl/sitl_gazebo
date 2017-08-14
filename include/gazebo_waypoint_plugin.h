#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include "gazebo/msgs/msgs.hh"

namespace gazebo {


class GazeboWaypointPlugin : public ModelPlugin {
 public:
  GazeboWaypointPlugin() :
    ModelPlugin()
    {}
  ~GazeboWaypointPlugin();
  typedef ignition::math::Vector3d Vector;

 protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo& /*_info*/);

 private:
  /** Pointer to the update event connection */
  event::ConnectionPtr _updateConnection;
  /** Pointer to the visual topic publisher */
  transport::PublisherPtr _visual_pub;
  /** Pointer to gazebo transport node */
  transport::NodePtr _node;
  /** Pointer to current model */
  physics::ModelPtr _model;
  /** Flag is set when the drone pass through the waypoint */
  bool _valid = false;
  /** Waypoint number set by <no> element */
  int32_t _no;
  /** Waypoint radius set by <radius> element. Default to 2. */
  double _radius;
  /** Unique name of the visual based on #_no */
  std::string _visual_name;

  /** Change the visual of the waypoint: set its color to the provided
   * color such as "Gazebo/Yellow" and "Gazebo/Green".
   * @param color the new color of the visual
   */
  void UpdateVisual(std::string color);
};
}
