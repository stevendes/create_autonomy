#include "ca_gazebo/color_sensor_plugin.h"

#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <ros/ros.h>

#include <string>
#include <vector>

#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "gazebo_plugins/gazebo_ros_camera.h"

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
    this->LoadRGBLimits(_sdf);
    this->publisher_name_ = _sdf->Get<std::string>("PublisherName");
    this->sensor_publisher_ = nh_.advertise<std_msgs::Bool>(this->publisher_name_, 1);
    this->color_percentage_=_sdf->Get<double>("ColorPercentage");
    this->parentSensor->SetActive(true);

  }

    void GazeboRosColor::LoadRGBLimits(sdf::ElementPtr _sdf)
    {
    this->rgbmax_[0] = _sdf->Get<double>("rvalue_max");
    this->rgbmax_[1] = _sdf->Get<double>("gvalue_max");
    this->rgbmax_[2] = _sdf->Get<double>("bvalue_max");

    this->rgbmin_[0] =  _sdf->Get<double>("rvalue_min");   
    this->rgbmin_[1] = _sdf->Get<double>("gvalue_min");
    this->rgbmin_[2] = _sdf->Get<double>("bvalue_min");
    }

    bool GazeboRosColor::InRange(std::vector<int> RGB)
    {
      bool R_ok_,G_ok_, B_ok_;
      R_ok_ = RGB[0] <= this->rgbmax_[0] and RGB[0] >= this->rgbmin_[0];
      G_ok_ = RGB[1] <= this-> rgbmax_[1] and RGB[1] >= this->rgbmin_[1];
      B_ok_ = RGB[2] <= this-> rgbmax_[2] and RGB[2] >= this->rgbmin_[2];

      return R_ok_ and G_ok_ and B_ok_;
    }


  ////////////////////////////////////////////////////////////////////////////////
  // Update the controller
  void GazeboRosColor::OnNewFrame(const unsigned char *_image,
    unsigned int _width, unsigned int _height, unsigned int _depth,
    const std::string &_format)
  {

    double goal_color_=0, pixel_quant_=0;

    this->sensor_update_time_ = this->parentSensor_->LastUpdateTime();

    std_msgs::BoolPtr publ(new std_msgs::Bool);
    const common::Time cur_time = this->world_->SimTime();
    if (cur_time - this->last_update_time_ >= this->update_period_)
    {
      this->PutCameraData(_image);
      this->PublishCameraInfo();
      this->last_update_time_ = cur_time;
      std::vector <int> RGB {0 , 0, 0};
      bool color_detected_;

      for (int i=0; i < _width * _height * 3 ; i = i + 3)
      {
       RGB[0] = _image[i];
       RGB[1] = _image[i + 1];
       RGB[2] = _image[i + 2];
       
        pixel_quant_=_width*_height;

        if (this->InRange(RGB))
        {
          goal_color_++;
        }
      }

      color_detected_ = goal_color_/pixel_quant_ > this->color_percentage_;
      publ->data = color_detected_;
      this->sensor_publisher_.publish(publ);  

      goal_color_ = 0;
      }
    }
  }
