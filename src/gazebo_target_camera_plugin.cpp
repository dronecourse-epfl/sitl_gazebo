#include "gazebo_target_camera_plugin.h"
#include <gazebo/sensors/sensors.hh>
#include "gazebo/rendering/Camera.hh"




namespace gazebo {

GZ_REGISTER_MODEL_PLUGIN(TargetCameraPlugin);

// debug only
void printPose(const math::Pose& pose, const std::string& name)
{
  const math::Vector3 euler = pose.rot.GetAsEuler();
  std::cout << "pose " << name << " pos ( " << pose.pos.x << " | " << pose.pos.y << " | " << pose.pos.z << ")   rot ( " << euler.x << " | " << euler.y << " | " << euler.z << " ) " << std::endl;
}

TargetCameraPlugin::TargetCameraPlugin() :
  ModelPlugin(),
  image_width2_(IMAGE_WIDTH_DEFAULT_ / 2.0f),
  image_height2_(IMAGE_HEIGHT_DEFAULT_ / 2.0f),
  hfov2_(HFOV_DEFAULT_ / 2.0f)
{
  vfov2_ = hfov2_*image_height2_/((float)image_width2_);
  focal_length_ = image_width2_ / tan(hfov2_);
  period_s_ = 1.0f / UPDATE_RATE_DEFAULT_;
}


void TargetCameraPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  world_ = _model->GetWorld();

  // -----------------------
  // find camera parameters
  // -----------------------
  const sensors::CameraSensorPtr camera_sensor = FindCameraSensor(_model);
  if(camera_sensor != NULL)
  {
    const rendering::CameraPtr camera = camera_sensor->Camera();
    hfov2_ = float(camera->HFOV().Radian())/2.0f;
    vfov2_ = float(camera->VFOV().Radian())/2.0f;
    image_width2_ = camera->ImageWidth()/2.0f;
    image_height2_ = camera->ImageHeight()/2.0f;
    focal_length_ = image_width2_ / tan(hfov2_);
    period_s_ = 1.0f / camera_sensor->UpdateRate();
    // add callback to camera
    newFrameConnection_ = camera->ConnectNewImageFrame(boost::bind(&TargetCameraPlugin::OnNewFrame, this));
  }
  else
  {
    camera_link_ = _model;
    std::cout << "TargetCameraPlugin::Load(..) could not find camera sensor! Using default values;" << std::endl;
    std::cout << "   using model (" << _model->GetName() << ") for camera pose" << std::endl;
    // add callback for world update
    updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&TargetCameraPlugin::OnUpdate, this, _1));
  }


  // TODO create model list from attributes
  std::vector<std::string> model_names;
  model_names.push_back("truck");


  std::cout << "###################### !!!!!!!!!!!!!!!! STARTING ADDING MODELS" << std::endl;

  // create a message for each model
  int target_num = 0;
  std::vector<std::string>::const_iterator it;
  for(it = model_names.begin(); it != model_names.end(); it++)
  {
    physics::ModelPtr model = world_->GetModel(*it);
    if(model)
    {
      message_map_[model].set_target_num(target_num++);
      message_map_[model].set_frame(0);
      std::cout << "###################### !!!!!!!!!!!!!!!! added model: " << *it << std::endl;
    }
  }

  // connection to publish pose over google proto buf
  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init();

  std::string topicName = "~/" + _model->GetName() + "/LandingTarget";
  landing_target_pub_ = node_handle_->Advertise<target_camera::msgs::LandingTarget>(topicName, 10);
}


void TargetCameraPlugin::OnUpdate(const common::UpdateInfo& info)
{
  // Get Delta T
  common::Time current_time = info.simTime;
  double dt = (current_time - last_time_).Double();

  if(dt > period_s_)
  {
    OnNewFrame();
    last_time_ = current_time;
  }
}



void TargetCameraPlugin::OnNewFrame()
{
  const math::Pose& camera_pose = camera_link_->GetWorldPose();
  uint64_t timestamp_us = (uint64_t)(world_->GetSimTime().Double()*1e6);

  // iterate over all targets
  std::map<physics::ModelPtr, TargetMsg>::iterator msg_it; 
  for(msg_it = message_map_.begin(); msg_it != message_map_.end(); msg_it++)
  {
    TargetMsg& msg = msg_it->second;

    // Get bearing of target in camera frame
    const math::Pose& target_pose = msg_it->first->GetWorldPose();
    math::Pose rel_pose = target_pose - camera_pose;

    float pixel_x = round((focal_length_ * rel_pose.pos.y/rel_pose.pos.z) + image_width2_); // column
    float pixel_y = round((focal_length_ * rel_pose.pos.x/rel_pose.pos.z) + image_height2_); // row

    if(pixel_x >= 0 && pixel_x < 2*image_width2_ && pixel_y >= 0 && pixel_y < 2*image_height2_)
    {
      msg.set_time_usec(timestamp_us);
      msg.set_angle_x(pixel_x);
      msg.set_angle_y(pixel_y);
      msg.set_distance(abs(rel_pose.pos.z));
      msg.set_size_x(2*image_width2_);
      msg.set_size_y(2*image_height2_);
      std::cout << "  pixels: ( " << pixel_x << " | " << pixel_y << " )    timestamp_us " << timestamp_us << std::endl;
      landing_target_pub_->Publish(msg);
    }
  }
}


const sensors::CameraSensorPtr TargetCameraPlugin::FindCameraSensor(physics::ModelPtr model)
{
  int model_sensor_count = model->GetSensorCount();
  if(model_sensor_count > 0)
  {
    sensors::SensorManager* sensor_manager = sensors::SensorManager::Instance();
    const physics::Link_V& links = model->GetLinks();
    physics::Link_V::const_iterator it;
    for(it = links.begin(); it != links.end(); it++)
    {
      physics::LinkPtr link = *it;
      int link_sensor_count = link->GetSensorCount();
      for(int i = 0; i < link_sensor_count; i++)
      {
        const sensors::SensorPtr sensor = sensor_manager->GetSensor(link->GetSensorName(i));
        sensors::CameraSensorPtr camera_sensor;
        #if GAZEBO_MAJOR_VERSION >= 7
          camera_sensor = std::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
        #else
          camera_sensor = boost::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
        #endif
        if(camera_sensor != NULL)
        {
          camera_link_ = link;
          return camera_sensor;
        }
      }
    }
  }
  return NULL;
}
}