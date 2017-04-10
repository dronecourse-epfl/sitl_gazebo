#include "gazebo_gimbal.hpp"
#include <gazebo/physics/physics.hh>


namespace gazebo {

float SmallAngle(float angle)
{
  while(angle > M_PI)
  {
    angle -= 2*M_PI;
  }

  while(angle <= -M_PI)
  {
    angle += 2*M_PI;
  }

  return angle;
}

Gimbal::Gimbal() :
  pos_pid_pitch_(PID(10,0.1,0.001,10)),
  pos_pid_roll_(PID(10,0.001,0.001,10)),
  vel_pid_pitch_(PID(50.0f,0.1f,0.1f, 50.0f)),
  vel_pid_roll_(PID(50.0f,0.1f,0.1f, 50.0f)),
  joint_(NULL),
  child_(NULL)
{
  SetCmd(-M_PI/2.0f,0.0f);
  is_initialized_ = false;
}

void Gimbal::FindJoint(const sdf::ElementPtr _sdf, const physics::ModelPtr model,  const std::string& param_name)
{
    if(_sdf->HasElement(param_name))
    {
      std::string joint_name = _sdf->GetElement(param_name)->Get<std::string>();
      joint_ = model->GetJoint(joint_name);
      
      if(joint_ != NULL)
      {
        child_ = joint_->GetChild();
      }
    }
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