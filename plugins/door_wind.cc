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
          std::bind(&ModelPush::OnUpdate, this));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {

  
    gazebo::physics::LinkPtr link=  this->model->GetChildLink("autoclose_door_wind::lever");
    gazebo::physics::JointPtr joint=  this->model->GetJoint("autoclose_door_wind::joint_door_lever");
    link->AddRelativeForce(ignition::math::Vector3d(0, 0, -3));
    link->AddRelativeTorque(ignition::math::Vector3d(0, 0, -5));
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(DoorWind)
}

