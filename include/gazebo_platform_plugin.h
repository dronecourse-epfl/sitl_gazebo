#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "platform.pb.h"

namespace gazebo {

typedef const boost::shared_ptr<const platform_msgs::msgs::platform> PlatformPtr;
static const std::string kDefaultPlatformSubTopic = "/platform";
class GazeboPlatformPlugin : public ModelPlugin {
 public:
  GazeboPlatformPlugin() :
	ModelPlugin(),
	platform_sub_topic_(kDefaultPlatformSubTopic)
  {}
  ~GazeboPlatformPlugin();
  typedef ignition::math::Vector3d Vector;

 protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void PlatformCallback(PlatformPtr& platform_message);
  void OnUpdate(const common::UpdateInfo& /*_info*/);

 private:
 physics::ModelPtr model_;
 physics::LinkPtr box_;
 physics::CollisionPtr collision_;
 math::Pose initialPose_;
 event::ConnectionPtr updateConnection_;  // Pointer to the update event connection
 
 Vector scale;
 std::string platform_sub_topic_;
 transport::NodePtr node_;
 transport::SubscriberPtr platform_sub_;
 float _h;
};
}
