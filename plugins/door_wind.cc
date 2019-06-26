#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
  class DoorWind : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&DoorWind::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {

  
      gazebo::physics::LinkPtr link=  this->model->GetChildLink("door_wind::lever");
      gazebo::physics::JointPtr joint=  this->model->GetJoint("door_wind::joint_frame_door");
      if (joint->GetAngle(2).Radian()<-0.7){
        link->AddRelativeForce(ignition::math::Vector3d(0, 0, 0.8));
      }
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(DoorWind)
}

