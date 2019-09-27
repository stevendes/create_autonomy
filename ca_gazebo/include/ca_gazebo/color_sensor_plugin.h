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
        ///Constructor
        GazeboRosColor();    
        ///Destructor
        ~GazeboRosColor();
        ///Load the plugin
        void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf); 
        /*
        param _parent : Pointer to the parent sensor
        param _sdf : Pointer to the sdf
        */

        /// \brief Update the controller
        protected: virtual void OnNewFrame(const unsigned char *_image,
        unsigned int _width, unsigned int _height,
        unsigned int _depth, const std::string &_format);

        private:

        /*
        param rvalue_max : Max value in R, received from SDF file
        param rvalue_min : Min value in R, received from SDF file
        param gvalue_max : Max value in G, received from SDF file
        param gvalue_min : Min value in G, received from SDF file
        param bvalue_max : Max value in B, received from SDF file
        param bvalue_min : Min value in B, received from SDF file
        param publisher_name : Name of the publisher topic, received from SDF file
        param color_percentage_: Max value of number of pixels of the same color divided by total number of pixels
        */

        unsigned int rvalue_max_;
        unsigned int rvalue_min_;
        unsigned int gvalue_max_;
        unsigned int gvalue_min_;
        unsigned int bvalue_max_;
        unsigned int bvalue_min_;
        unsigned int update_rate_; 
        std::string publisher_name_;
        float color_percentage_;


        /// Initialize ROS variables
        ros::NodeHandle nh_;
        ros::Publisher sensor_publisher_;

      };
    }
    #endif