#ifndef GAZEBO_ROS_COLOR_SENSOR_HH
#define GAZEBO_ROS_COLOR_SENSOR_HH

#include <string>

// library for processing camera data for gazebo / ros conversions
#include <gazebo/plugins/CameraPlugin.hh>

#include <gazebo_plugins/gazebo_ros_camera_utils.h>

namespace gazebo
{
class GazeboRosColor : public CameraPlugin, GazeboRosCameraUtils
{

public:
  /**
   * @brief Construct a new Gazebo Ros Color object
   * 
   */
  GazeboRosColor();

  /**
   * @brief Destroy the Gazebo Ros Color object
   * 
   */
  ~GazeboRosColor();

  /**
   * @brief Overrides the load virtual function
   * 
   * @param _parent Pointer to parent sensor
   * @param _sdf Pointer to the sdf, in order to get the tags values
   */
  void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);
  /**
   * @brief Returns true if the pixel color matches the target
   * 
   * @param target r g b values
   * @param r 
   * @param g 
   * @param b 
   * @return true 
   * @return false 
   */
  bool IsColor(std::array<unsigned char, 3> target, unsigned char r, unsigned char g, unsigned char b);

protected:
  /**
   * @brief Callback executed when a new frame is captured
   * 
   * @param _image Image array
   * @param int width
   * @param int height
   * @param int depth
   * @param _format   
   */
  virtual void OnNewFrame(const unsigned char *_image,
                          unsigned int _width, unsigned int _height,
                          unsigned int _depth, const std::string &_format);

  ros::NodeHandle _nh;
  ros::Publisher _sensorPublisher;
  std::string sensor_color_;
  std::string publish_topic_name_;
  double _fov;
  double _range;
  std::map<std::string, std::array<unsigned char, 3>> _map_of_colors;
  const int COLOR_TOLERANCE = 30;  //tolerance between target colour and the pixel
  const int PIXEL_THRESHOLD = 100; //minimum amount of pixels where the sensor returns true
};
} // namespace gazebo
#endif
