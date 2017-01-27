#include "gazebo_truck_plugin.h"

namespace gazebo {

GZ_REGISTER_MODEL_PLUGIN(GazeboTruckPlugin);

GazeboTruckPlugin::~GazeboTruckPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
}

void GazeboTruckPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  alpha_ = 0.0f;

  // Store the pointer to the model.
  model_ = _model;
  world_ = model_->GetWorld();

  const math::Vector3 rot_vel(0,0,1);
  model_->SetAngularVel(rot_vel);


  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboTruckPlugin::OnUpdate, this, _1));
}

// This gets called by the world update start event.
void GazeboTruckPlugin::OnUpdate(const common::UpdateInfo& /*_info*/) {

  common::Time current_time = world_->GetSimTime();
  double dt = (current_time - last_time_).Double();

  float lin_speed = 3;
  float rot_speed = lin_speed/radius_;
  float alpha = current_time.Double()*rot_speed;

  // math::Pose pose = model_->GetWorldPose();
  //float alpha = atan2(pose.pos.x, pose.pos.y);
  math::Vector3 rot_vel(0,0,10);
  //model_->SetAngularVel(rot_vel);

  math::Vector3 lin_vel(cos(alpha)*lin_speed,-sin(alpha)*lin_speed,0);
  //model_->SetLinearVel(lin_vel);  


  math::Vector3 pos(sin(alpha)*radius_, cos(alpha)*radius_, 0);
  math::Vector3 rot(0,0,M_PI-alpha);
  math::Pose pose(pos, rot);
  model_->SetWorldPose(pose);
}
}
