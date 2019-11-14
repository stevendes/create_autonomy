#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include "gazebo/common/Time.hh"



#include <ros/ros.h>
#include "std_msgs/Float64.h"


namespace gazebo
{
  class ModelPush : public ModelPlugin
  {
/*    enum State
    {
	    UP,
	    TIME_UP,
	    DOWN,
	    STOP_DOWN,
    };*/

    ros::NodeHandle nh_;
    ros::Publisher dummy_publisher_=nh_.advertise<std_msgs::Float64>("doomsday", 1);
    double curr_position;
    double output;
    double set_pos=1;


//    bool time_flag= false;
//    State g_state = UP;
   

    public:
    ModelPush(): 
      nh_("dummy_node")
    {

    }
    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      this->jointController=this->model->GetJointController();
      this->joint = this->model->GetJoint("toll_joint");
      this->pid = common::PID(100, 0, 0);
      std::string name = model->GetJoint("toll_joint")->GetScopedName();
      this->jointController->SetVelocityPID(name, this->pid);
      this->jointController->SetVelocityTarget(name, 1.0);

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelPush::OnUpdate, this));
          
    }


    // Called by the world update start event
    public: void OnUpdate()
    {
//      double error;
      // Apply a small linear velocity to the model.
//      this->curr_position = model->GetJoint("toll_joint")->Position();
//      this->joint = model->GetJoint("toll_joint");

      this->jointController->Update();
     
//      this->model->GetJoint("toll_joint")->SetVelocity(0, 0.1);

      
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    /// brief Pointer to the joint.
    private: physics::JointPtr joint;

    /// brief A PID controller for the joint.
    private: common::PID pid;

    private: physics::JointControllerPtr jointController;   
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}