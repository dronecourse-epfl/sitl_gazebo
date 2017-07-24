#include <gazebo/physics/physics.hh>
#include "gazebo_platform_plugin.h"
#include "common.h"
using namespace std;

namespace gazebo {

GZ_REGISTER_MODEL_PLUGIN(GazeboPlatformPlugin);

GazeboPlatformPlugin::~GazeboPlatformPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
}

void GazeboPlatformPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Store the pointer to the model.
  model_ = _model;
  box_ = _model->GetChildLink("box");
  initialPose_ = model_->GetWorldPose();
  _h = 1;
  
  node_ = transport::NodePtr(new transport::Node());
  node_->Init();
  getSdfParam<std::string>(_sdf, "platformSubTopic", platform_sub_topic_, platform_sub_topic_);
  
  platform_sub_ = node_->Subscribe("~/" + platform_sub_topic_, &GazeboPlatformPlugin::PlatformCallback, this);
  // platform_sub_ = node_->Subscribe("~/platform", &GazeboPlatformPlugin::PlatformCallback, this);

  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboPlatformPlugin::OnUpdate, this, _1));


}

void GazeboPlatformPlugin::OnUpdate(const common::UpdateInfo& /*_info*/) {
	model_->SetScale(Vector(1, 1, _h), true);	
	model_->SetWorldPose(math::Pose(0,10,_h/2,0,0,0), true);
}

// This gets called by the world update start event.
void GazeboPlatformPlugin::PlatformCallback(PlatformPtr& platform_message) {
	_h = platform_message->h();
}
}
