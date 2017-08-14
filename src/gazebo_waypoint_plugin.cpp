#include <ignition/math/Vector3.hh>
#include <gazebo/physics/physics.hh>
#include "gazebo_waypoint_plugin.h"
#include <gazebo/msgs/msgs.hh>

using namespace std;

namespace gazebo {

GZ_REGISTER_MODEL_PLUGIN(GazeboWaypointPlugin);

GazeboWaypointPlugin::~GazeboWaypointPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->_updateConnection);
}

void GazeboWaypointPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf) {
  if(sdf->HasElement("no")) {
    this->_no = sdf->GetElement("no")->Get<int>();
  } else {
    gzerr << "[waypoint] Please specify a value for parameter \"no\".\n";
    this->_no = -1;
    return;
  }

  if(sdf->HasElement("radius")) {
    this->_radius = sdf->GetElement("radius")->Get<double>();
  } else {
    this->_radius = 2;
  }

  this->_visual_name = "__WAYPOINT__VISUAL__NO__" + this->_no;
  this->_model = model;
  this->_node = gazebo::transport::NodePtr(new gazebo::transport::Node());
  this->_node->Init(_model->GetWorld()->GetName());
  this->_updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboWaypointPlugin::OnUpdate, this, _1));
  this->_visual_pub = this->_node->Advertise<msgs::Visual>("~/visual", 10);
  this->_valid = false;
}

void GazeboWaypointPlugin::OnUpdate(const common::UpdateInfo& /*_info*/) {
  ignition::math::Vector3d pose = _model->GetWorld()->GetModel("iris_dronecourse")
          ->GetWorldPose().Ign().Pos();
  ignition::math::Vector3d wp_pose = _model
          ->GetWorldPose().Ign().Pos();
  double distance = (pose - wp_pose).Length();
  if(this->_valid || distance < this->_radius)  {
    this->UpdateVisual("Gazebo/Green");
    this->_valid = true;
  } else {
    this->UpdateVisual("Gazebo/Yellow");
  }
}

void GazeboWaypointPlugin::UpdateVisual(std::string color) {
  msgs::Visual visualMsg;

  visualMsg.set_name(this->_visual_name);

  visualMsg.set_parent_name(_model->GetScopedName());
  msgs::Geometry *geomMsg = visualMsg.mutable_geometry();
  geomMsg->set_type(msgs::Geometry::SPHERE);
  geomMsg->mutable_sphere()->set_radius(this->_radius);
  visualMsg.mutable_material()->mutable_script()
        ->set_name(color);
  msgs::Set(visualMsg.mutable_pose(),
    _model->GetWorldPose().Ign());
  visualMsg.set_cast_shadows(false);
  visualMsg.set_transparency(0.5);

  this->_visual_pub->Publish(visualMsg);
}
}
