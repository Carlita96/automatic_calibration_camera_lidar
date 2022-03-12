#include <gazebo/gazebo.hh>
#include <ignition/math.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <gazebo/common/Plugin.hh>

namespace gazebo
{
  class AnimatedSphereSetup0 : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

        // create the animation
        gazebo::common::PoseAnimationPtr anim(
              // name the animation "test",
              // make it last 10 seconds,
              // and set it on a repeat loop
              new gazebo::common::PoseAnimation("test", 45.0, true));

        gazebo::common::PoseKeyFrame *key;

        // set starting location of the box
        key = anim->CreateKeyFrame(0);
        key->Translation(ignition::math::Vector3d(-2.5, 0.0, 1.25));
        key->Rotation(ignition::math::Quaterniond(0, 0, 0));
        
        key = anim->CreateKeyFrame(15.0);
        key->Translation(ignition::math::Vector3d(-3.25, 0.75, 2.25));
        key->Rotation(ignition::math::Quaterniond(0, 0, 0));

        key = anim->CreateKeyFrame(22.5);
        key->Translation(ignition::math::Vector3d(-4.25, -0.5, 2.5));
        key->Rotation(ignition::math::Quaterniond(0, 0, 0));

        key = anim->CreateKeyFrame(32.5);
        key->Translation(ignition::math::Vector3d(-3.25, -0.75, 1.75));
        key->Rotation(ignition::math::Quaterniond(0, 0, 0));

        // set final location equal to starting location
        key = anim->CreateKeyFrame(45.0);
        key->Translation(ignition::math::Vector3d(-2.5, 0.0, 1.25));
        key->Rotation(ignition::math::Quaterniond(0, 0, 0));

        // set the animation
        _parent->SetAnimation(anim);
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(AnimatedSphereSetup0)
}