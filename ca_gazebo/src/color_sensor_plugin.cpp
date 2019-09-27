#include "ca_gazebo/color_sensor_plugin.h"

#include <string>

#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include <ros/ros.h>

#include <gazebo/common/Plugin.hh>
#include "gazebo_plugins/gazebo_ros_camera.h"
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/SensorTypes.hh>

namespace gazebo
{
  // Register this plugin with the simulator
  GZ_REGISTER_SENSOR_PLUGIN(GazeboRosColor)

  ////////////////////////////////////////////////////////////////////////////////
  // Constructor
  GazeboRosColor::GazeboRosColor():
    nh_("color_sensor_plugin")
{

}
  ////////////////////////////////////////////////////////////////////////////////
  // Destructor
  GazeboRosColor::~GazeboRosColor()
  {
    ROS_DEBUG_STREAM_NAMED("camera","Unloaded");
  }

  void GazeboRosColor::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
  {
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    CameraPlugin::Load(_parent, _sdf);
    GazeboRosCameraUtils::Load(_parent, _sdf);
    // copying from CameraPlugin into GazeboRosCameraUtils
    this->parentSensor_ = this->parentSensor;
    this->width_ = this->width;
    this->height_ = this->height;
    this->depth_ = this->depth;
    this->format_ = this->format;
    this->camera_ = this->camera;
    this->rvalue_max_ = _sdf->Get<double>("rvalue_max");
    this->rvalue_min_ = _sdf->Get<double>("rvalue_min");
    this->gvalue_max_ = _sdf->Get<double>("gvalue_max");
    this->gvalue_min_ = _sdf->Get<double>("gvalue_min");
    this->bvalue_max_ = _sdf->Get<double>("bvalue_max");
    this->bvalue_min_ = _sdf->Get<double>("bvalue_min");
    this->publisher_name_ = _sdf->Get<std::string>("PublisherName");
    this->sensor_publisher_ = nh_.advertise<std_msgs::Bool>(this->publisher_name_, 1);
    this->color_percentage_=_sdf->Get<double>("ColorPercentage");
    this->parentSensor->SetActive(true);

  }

  ////////////////////////////////////////////////////////////////////////////////
  // Update the controller
  void GazeboRosColor::OnNewFrame(const unsigned char *_image,
    unsigned int _width, unsigned int _height, unsigned int _depth,
    const std::string &_format)
  {

    int R,G,B;

    double desired_color_in_pixel_=0, pixel_quant_=0;

    this->sensor_update_time_ = this->parentSensor_->LastUpdateTime();

    std_msgs::BoolPtr publ(new std_msgs::Bool);
    const common::Time cur_time = this->world_->SimTime();
    if (cur_time - this->last_update_time_ >= this->update_period_)
    {
      this->PutCameraData(_image);
      this->PublishCameraInfo();
      this->last_update_time_ = cur_time;

      bool output_;

      for (int i=0; i<_width*_height*3 ;i=i+3)
      {
        R = _image[i] ;
        G = _image[i + 1];
        B = _image[i + 2];
        pixel_quant_=_width*_height;

        if (R <= this->rvalue_max_ and R > this->rvalue_min_ and G <= this->gvalue_max_ and G > this->gvalue_min_ and B <= this->bvalue_max_ and B > this->bvalue_min_)
        {
          desired_color_in_pixel_++;
        }
      }

      output_ = desired_color_in_pixel_/pixel_quant_ > this->color_percentage_;
      publ->data = output_;
      publ->data = output_;
      this->sensor_publisher_.publish(publ);  

      desired_color_in_pixel_ = 0;
      }
    }
  }
