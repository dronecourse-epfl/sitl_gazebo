#include <gazebo/gazebo.hh>

namespace gazebo
{

  class PID
  {
  public:
    PID(float kp, float ki, float kd, float limit_out) :
      kp_(kp),
      ki_(ki),
      kd_(kd),
      limit_out_(limit_out),
      error_(0.0f),
      integr_(0.0f),
      last_update_s_(0.0)
    {};

    void reset (double current_time_s) {
      last_update_s_ = current_time_s;
      error_ = 0;
      integr_ = 0;
    }

    float Update(double current_time_s, float err)
    {
      if(last_update_s_ == 0)
      {
       last_update_s_ = current_time_s;
       return 0; 
      }

      float dt = current_time_s - last_update_s_;

      // avoid division by 0
      if(dt < 0.0000000001)
      {
        return 0;
      }

      float derr = (err - error_)/dt;
      integr_ += err * dt;
      float out = kp_*err + ki_*integr_ + kd_ * derr;
      // clip output
      if(out > limit_out_)
      {
        out = limit_out_;
      } else if(out < -limit_out_)
      {
        out = -limit_out_;
      } 

      error_ = err;
      last_update_s_ = current_time_s;
      return out;
    };

  private:
    float kp_;
    float ki_;
    float kd_;
    float limit_out_;
    float error_;
    float integr_;
    double last_update_s_;

  };

  class Gimbal
  {
  public:

    Gimbal();

    void Find(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    void FindJoint(const sdf::ElementPtr _sdf, const physics::ModelPtr model, const std::string& param_name);

    void SetCmd(float pitch, float roll);

    float GetPitch() const {return pitch_;};
    float GetRoll() const {return roll_;};

    void Update(double current_time);

  private:
    physics::JointPtr joint_;
    physics::LinkPtr child_;
    PID pos_pid_pitch_;
    PID pos_pid_roll_;
    PID vel_pid_pitch_;
    PID vel_pid_roll_;
    bool is_initialized_;
    double cmd_pitch_;
    double cmd_roll_;
    double pitch_;
    double roll_;
  };
}