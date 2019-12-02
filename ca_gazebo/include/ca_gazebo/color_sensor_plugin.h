    #ifndef GAZEBO_ROS_COLOR_SENSOR_HH
    #define GAZEBO_ROS_COLOR_SENSOR_HH
    
    #include <string>
    #include <vector>
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

        // Put the RGB limits from the SDF to the rgbmax_ and rgbmin_ vectors
        void LoadRGBLimits(sdf::ElementPtr _sdf);

        // Check if the data received from the sensor is in range with the parameters
        bool InRange(std::vector<int> RGB);

        /// \brief Update the controller
        protected: virtual void OnNewFrame(const unsigned char *_image,
        unsigned int _width, unsigned int _height,
        unsigned int _depth, const std::string &_format);

        private:
        // vector rgbmax_: used to store the maximun values of the RGB range
        std::vector<int> rgbmax_ {0, 0, 0};
        // vector rgbmin_: used to store the minimun values of the RGB range
        std::vector<int> rgbmin_ {0, 0, 0};  
        //param publisher_name : Name of the publisher topic, received from SDF file
        std::string publisher_name_;
        //param publisher_name : Name of the publisher topic, received from SDF file
        float color_percentage_;

        /// Initialize ROS variables
        ros::NodeHandle nh_;
        ros::Publisher sensor_publisher_;

      }; // GazeboRosColor
    } // gazebo
    #endif // GAZEBO_ROS_COLOR_SENSOR_HH
    
