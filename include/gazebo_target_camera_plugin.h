#ifndef _GAZEBO_TARGET_CAMERA_PLUGIN_HH_
#define _GAZEBO_TARGET_CAMERA_PLUGIN_HH_

#include "gazebo/common/Plugin.hh"
#include <gazebo/physics/physics.hh>
#include "gazebo/gazebo.hh"
#include "LandingTarget.pb.h"


namespace gazebo
{

  class TargetCameraPlugin : public ModelPlugin
  {
    // Default camera parameters (values defined after class definition)
    const static float HFOV_DEFAULT_;                // default horizontal field of view [rad]
    const static unsigned int IMAGE_WIDTH_DEFAULT_;  // default image width [pixel]
    const static unsigned int IMAGE_HEIGHT_DEFAULT_; // default image height [pixel]
    

    public:
      typedef target_camera::msgs::LandingTarget TargetMsg;


    	TargetCameraPlugin();
    	
    	virtual ~TargetCameraPlugin(){};

    	virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
      
    	virtual void OnUpdate(const common::UpdateInfo& /*_info*/);

    private:
      const sensors::CameraSensorPtr FindCameraSensor(physics::ModelPtr model);

      physics::EntityPtr camera_link_;

      // camera parameters
      float hfov2_;                 // half horizontal field of view [rad]
      float vfov2_;                 // half vertical field of view [rad]
      unsigned int image_width2_;   // half the image width [pixel]
      unsigned int image_height2_;  // half the image height [pixel]
      float focal_length_;          // focal length of the camera as image_width2_ / tan(hfov2_) [pix]

      std::map<physics::ModelPtr, TargetMsg> message_map_;  // Map containing target models and messages for each model
      event::ConnectionPtr updateConnection_;               // 
      
      std::string namespace_;
      transport::PublisherPtr landing_target_pub_;
      transport::NodePtr node_handle_;
  };

  const float TargetCameraPlugin::HFOV_DEFAULT_ = 1.0f;               // default horizontal field of view [rad]
  const unsigned int TargetCameraPlugin::IMAGE_WIDTH_DEFAULT_  = 640; // default image width [pixel]
  const unsigned int TargetCameraPlugin::IMAGE_HEIGHT_DEFAULT_ = 340; // default image height [pixel]

}
#endif