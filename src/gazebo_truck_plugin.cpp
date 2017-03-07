#include "gazebo_truck_plugin.h"
using namespace std;

namespace gazebo {

GZ_REGISTER_MODEL_PLUGIN(GazeboTruckPlugin);

GazeboTruckPlugin::~GazeboTruckPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
}

void GazeboTruckPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model.
  model_ = _model;
  world_ = model_->GetWorld();

  state_ = 0; // reset state machine
  alpha_ = 0.5;

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboTruckPlugin::OnUpdate, this, _1));

  // connection to publish pose over google proto buf
  // node_handle_ = transport::NodePtr(new transport::Node());
  // node_handle_->Init(namespace_);

  // std::string topicName = "~/" + _model->GetName() + "/Position";
  // pose_pub_ = node_handle_->Advertise<vehicle_state_msgs::msgs::Pose>(topicName, 10);
}

// This gets called by the world update start event.
void GazeboTruckPlugin::OnUpdate(const common::UpdateInfo& /*_info*/) {

  common::Time current_time = world_->GetSimTime();
  double dt = (current_time - last_time_).Double();
  last_time_ = current_time;

  float lin_speed = 1.5;
  float rot_speed = lin_speed/radius_;

  math::Vector3 pos;
  math::Quaternion rot;
  float beta;
  // state machine
  switch(state_)
  {
    case 0:   // straight segment

      alpha_ += dt * lin_speed / length_;
      pos.Set(alpha_*length_, -radius_, 0);
      rot.SetFromEuler(0,0,0);    
      break;

    case 1:
      alpha_ += dt * rot_speed / M_PI;
      beta = alpha_ * M_PI;
      pos.Set(length_+sin(beta)*radius_, -cos(beta)*radius_, 0);
      rot.SetFromEuler(0,0,beta);  
      break;

    case 2:
      alpha_ += dt * lin_speed / length_;
      pos.Set((1-alpha_)*length_, radius_, 0);
      rot.SetFromEuler(0,0,M_PI);
      break;

    case 3:
      alpha_ += dt * rot_speed / M_PI;
      beta = alpha_ * M_PI + M_PI;
      pos.Set(sin(beta)*radius_, -cos(beta)*radius_, 0);
      rot.SetFromEuler(0,0,beta);  
      break;    
  }

  if(alpha_ >= 1)
  {
    alpha_ = 0;
    state_++;
    state_ = state_ % 4;
  }
  
  math::Pose pose(pos,rot);
  model_->SetWorldPose(pose);

  // gazebo::msgs::Quaternion* msg_orientation = new gazebo::msgs::Quaternion();
  // msg_orientation->set_x(rot.x);
  // msg_orientation->set_y(rot.y);
  // msg_orientation->set_z(rot.z);
  // msg_orientation->set_w(rot.w);
  // pose_message_.set_allocated_orientation(msg_orientation);
  // gazebo::msgs::Vector3d* msg_position = new gazebo::msgs::Vector3d();
  // msg_position->set_x(pos.x);
  // msg_position->set_y(pos.y);
  // msg_position->set_z(pos.z);
  // pose_message_.set_allocated_position(msg_position);
  // pose_message_.set_time_msec(current_time.Double()*1000.0f);
  // pose_pub_->Publish(pose_message_);
}
}
