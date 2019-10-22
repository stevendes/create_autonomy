#include <string> //c++ libraries
#include <stdlib.h> 

 //ros libraries
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Bool.h>

#include <gazebo/common/Plugin.hh> //gazebo libraries
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include "gazebo_plugins/gazebo_ros_camera.h"

#include "ca_gazebo/color_sensor_plugin.h" //own libraries



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
    //add the colours to the std::map
    this->_map_of_colors.insert(std::make_pair("white", std::array<unsigned char,3>{255,255,255}));
    this->_map_of_colors.insert(std::make_pair("yellow", std::array<unsigned char,3>{255,255,0}));
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Destructor
  GazeboRosColor::~GazeboRosColor()
  {
    ROS_DEBUG_STREAM_NAMED("camera","Unloaded");
  }
  
  bool GazeboRosColor::IsColor(std::array<unsigned char, 3>target, unsigned char r, unsigned char g, unsigned char b){
    return (abs(r - target[0])<=COLOR_TOLERANCE and abs(g - target[1]) <= COLOR_TOLERANCE && abs(b - target[2]) <= COLOR_TOLERANCE);
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
    this->update_rate_ = _sdf-> Get<double>("update_rate");
    _sensorPublisher = _nh.advertise<std_msgs::Bool>(this->publish_topic_name_, 1);

    this->update_period_=common::Time(1/this->update_rate_).Double();
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
    unsigned char starting_pixel=0;
    unsigned char r,g,b;
    common::Time cur_time = this->world_->SimTime();

    
    if (cur_time.Double() - this->last_update_time_.Double() >= this->update_period_){
      this->last_update_time_ = cur_time;
      this->PutCameraData(_image);
      this->PublishCameraInfo();


    for (int row = 0; row < _width; row++)
    {
      for(int col=0; col < _height; col++)
      {
        r=_image[starting_pixel];
        g=_image[starting_pixel+(unsigned char) 1];
        b=_image[starting_pixel+(unsigned char) 2];
              
        if(this->IsColor(this->_map_of_colors[this->sensor_color_], r, g, b))
        {
          count++;
        }
      
      starting_pixel += (unsigned char) 3;
      }
    }
    msg.data=count > PIXEL_THRESHOLD;
    _sensorPublisher.publish(msg);
    }
    
  }
}
