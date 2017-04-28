#include "gazebo_gimbal.hpp"
#include <gazebo/physics/physics.hh>


namespace gazebo {

const std::string Gimbal::GIMBAL_JOINT = "gimbal_joint";


float SmallAngle(float angle)
{

  angle = fmod(angle, 2*M_PI);

  if(angle > M_PI)
  {
    angle -= 2*M_PI;
  } else if(angle <= -M_PI)
  {
    angle += 2*M_PI;
  }

  return angle;
}

Gimbal::Gimbal() :
  gimbal_command_sub_topic_(kDefaultGimbalCommandSubTopic),
  pos_pid_pitch_(PID(20,0.1,0.001,10)),
  pos_pid_roll_(PID(20,0.001,0.001,10)),
  vel_pid_pitch_(PID(50.0f,0.1f,0.1f, 50.0f)),
  vel_pid_roll_(PID(50.0f,0.1f,0.1f, 50.0f)),
  joint_(NULL),
  child_(NULL),
  pitch_(0),
  roll_(0)
{
  SetCmd(-M_PI/2.0f,0.0f);
  is_initialized_ = false;
}


void Gimbal::Load(const sdf::ElementPtr _sdf, const physics::ModelPtr model)
{
  // connection to publish pose over google proto buf
  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init();
  gimbal_command_sub_ = node_handle_->Subscribe("~/" + model->GetName() + gimbal_command_sub_topic_,  &Gimbal::GimbalCommandCallback, this);
  FindJoint(_sdf, model);
}

void Gimbal::FindJoint(const sdf::ElementPtr _sdf, const physics::ModelPtr model)
{
    if(_sdf->HasElement(GIMBAL_JOINT))
    {
      std::string joint_name = _sdf->GetElement(GIMBAL_JOINT)->Get<std::string>();
      joint_ = model->GetJoint(joint_name);
      
      if(joint_ != NULL)
      {
        child_ = joint_->GetChild();
      }
    }
}

void Gimbal::GimbalCommandCallback(GimbalCommandPtr& msg)
{
  float pitch = msg->pitch();
  float roll = msg->roll();
  SetCmd(pitch, roll);
}


void Gimbal::SetCmd(float pitch, float roll)
{
  cmd_pitch_ = pitch;
  cmd_roll_ = roll;
}

void Gimbal::Update(double current_time)
{
  if(joint_ == NULL || child_ == NULL)
  {
    return;
  }

  if(!is_initialized_){
    pos_pid_pitch_.reset(current_time);
    pos_pid_roll_.reset(current_time);
    vel_pid_pitch_.reset(current_time);
    vel_pid_roll_.reset(current_time);
    is_initialized_ = true;
    return;
  }

  pitch_ = joint_->GetAngle(0).Radian();
  math::Quaternion q = math::Quaternion(0,-pitch_,0).GetInverse()*child_->GetRelativePose().rot;
  roll_ = q.GetRoll();
  
  float vel_cmd = pos_pid_pitch_.Update(current_time, SmallAngle(cmd_pitch_ - pitch_));
  float force_cmd = vel_pid_pitch_.Update(current_time, vel_cmd - joint_->GetVelocity(0));
  joint_->SetForce(0,force_cmd*1e-9);

  float vel_roll_ = joint_->GetVelocity(1);
  vel_cmd = -pos_pid_roll_.Update(current_time, SmallAngle(cmd_roll_ - roll_));
  force_cmd = vel_pid_roll_.Update(current_time, (vel_cmd - vel_roll_));
  joint_->SetForce(1,force_cmd*1e-9);
}

}