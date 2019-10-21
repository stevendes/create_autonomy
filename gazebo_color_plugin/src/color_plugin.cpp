#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <ros/console.h>
#include "gazebo_color_sensor_plugin/color_sensor_plugin.h"

#include "gazebo_plugins/gazebo_ros_camera.h"

#include <string>
#include <stdlib.h>

#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <std_msgs/Bool.h>

#include <sensor_msgs/Illuminance.h>

#define COLOR_TOLERANCE 30
#define PIXEL_THRESHOLD 100 //minimum amount of pixels where the sensor returns true

namespace gazebo
{
  // Register this plugin with the simulator
  GZ_REGISTER_SENSOR_PLUGIN(GazeboRosColor)

  ////////////////////////////////////////////////////////////////////////////////
  // Constructor
  GazeboRosColor::GazeboRosColor():
  _nh("color_sensor_plugin"),
  _fov(6),
  _range(10),
  CameraPlugin(),
  GazeboRosCameraUtils()

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
    // copying from CameraPlugin into GazeboRosCameraUtils
    GazeboRosCameraUtils::Load(_parent, _sdf);
    this->parentSensor_ = this->parentSensor;
    this->width_ = this->width;
    this->height_ = this->height;
    this->depth_ = this->depth;
    this->format_ = this->format;
    this->camera_ = this->camera;
    this->publish_topic_name_ = _sdf-> Get<std::string>("publish_topic");
    this->sensor_color_ = _sdf-> Get<std::string>("sensor_color");
    this->update_rate_ = atof(_sdf-> Get<std::string>("update_rate").c_str());
    _sensorPublisher = _nh.advertise<std_msgs::Bool>(this->publish_topic_name_, 1);

    if (this->sensor_color_=="white")
    {
      this->target_color_[0]=255;
      this->target_color_[1]=255;
      this->target_color_[2]=255;
    }
    if (this->sensor_color_=="yellow")
    {
      this->target_color_[0]=255;
      this->target_color_[1]=255;
      this->target_color_[2]=0;
    }
    this->parentSensor_->SetActive(true);
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Update the controller
  void GazeboRosColor::OnNewFrame(const unsigned char *_image,
    unsigned int _width, unsigned int _height, unsigned int _depth,
    const std::string &_format)
  {
    int count=0;

    std_msgs::Bool msg;
    unsigned char startingPixel=0;
    unsigned char r,g,b;
    common::Time cur_time = this->world_->SimTime();
    this->sensor_update_time_ = this->parentSensor_->LastUpdateTime();
    this->update_period_=common::Time(1/this->update_rate_).Double();
    if (cur_time.Double() - this->last_update_time_.Double() >= this->update_period_){
      this->last_update_time_=cur_time;
      this->PutCameraData(_image);
      this->PublishCameraInfo();


    for (int i=0;i<_width;i++)
    {
      for(int j=0;j<_height;j++)
      {
        r=_image[startingPixel];
        g=_image[startingPixel+(unsigned char) 1];
        b=_image[startingPixel+(unsigned char) 2];
              
        if(abs(r-this->target_color_[0]) <= COLOR_TOLERANCE && abs(g-this->target_color_[1]) <= COLOR_TOLERANCE && abs(b-this->target_color_[2]) <= COLOR_TOLERANCE)
        {
          count++;
        }
      
      startingPixel+=(unsigned char) 3;
      }
    }
    msg.data=count>PIXEL_THRESHOLD;
    _sensorPublisher.publish(msg);
    this->sensor_update_time_ = this->parentSensor_->LastUpdateTime();
    }
    
  }
}
