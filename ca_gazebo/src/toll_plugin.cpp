#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>

#include <functional>

#include "gazebo/common/Time.hh"
#include "std_msgs/Float64.h"

enum State
{
	RISING,
  WAITING_UP,
	FALLING,
  WAITING_DOWN
};

State g_state;

namespace gazebo
{
  class ModelPush : public ModelPlugin
  {
    public:
    ModelPush(): 
    {
    }

    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      // Configure the PID controller for the joint
      this->jointController.reset(new physics::JointController(this->model));
      this->jointController->AddJoint(model->GetJoint("toll_joint"));
      this->joint = this->model->GetJoint("toll_joint");
      this->pid = common::PID(2.52 , 1.5, 1);
      this->name = model->GetJoint("toll_joint")->GetScopedName();
      this->jointController->SetPositionPID(this->name, this->pid);
      this->jointController->SetPositionTarget (this->name, 1.57);
      
      // Set the initial state
      g_state = RISING;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelPush::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      // If the state is "RISING" and the position is up enough to let the robot pass, starts a timer that controls the time the toll is up
      if (g_state == RISING)
      {
        if (this->joint->Position() > 1.5 and this->timer_flag == false)
        {
          this->timer.Start();
          this->timer_flag = true;
          g_state = WAITING_UP;
        }
      }

      // Checks the timer, when 30 secs pass, set the new goal for de PID to the starting position, stops the timer and resets it
      if (g_state == WAITING_UP) 
      {
        if (this->timer.GetElapsed().Float() > 30 )
        {
          g_state = FALLING;
          this->timer.Stop();
          this->timer.Reset();
          this->timer_flag = false;
          this->jointController->SetPositionTarget (name, 0.0);
        }
      }

      // If the state is "FALLING" and the position is back at the starting point, it starts the timer again
      if (g_state == FALLING)
      {
        if (this->joint->Position() < 0.5 and this->timer_flag == false)
        {
          this->timer.Start();
          this->timer_flag = true;
          g_state = WAITING_DOWN;
        }        
      }

      // Checks the timer, when 30 secs pass, set the new goal for de PID to the up position, stops the timer and resets it
      if (g_state == WAITING_DOWN) 
      {
        if (this->timer.GetElapsed().Float() > 30 )
        {
          g_state = RISING;
          this->timer.Stop();
          this->timer.Reset();
          this->timer_flag = false;
          this->jointController->SetPositionTarget (name, 1.57);
        }
      }

      //Update the PID output
      this->jointController->Update();

    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    /// brief A PID controller for the joint.
    private: common::PID pid;

    // Pointer to the joint controller
    private: physics::JointControllerPtr jointController;   

    // Pointer to the Joint
    private: physics::JointPtr joint;

    // Timer
    private: common::Timer timer;

    // String to get the name of the joint
    private: std::string name;

    // Flag for the timer
    bool timer_flag = false;

  }; // ModelPush

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
} // gazebo