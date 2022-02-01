#ifndef USV_GAZEBO_PLUGINS_THRUST_HH
#define USV_GAZEBO_PLUGINS_THRUST_HH

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/JointState.h>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include "geometry_msgs/Vector3.h"

#include <gazebo/common/CommonTypes.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/physics.hh>

#include <sdf/sdf.hh>

namespace gazebo
{
  // Foward declaration of Tension class
  class Tension;

  /// \brief Boat class
  class Boat 
  {
    /// \brief Constructor
    /// \param[in] _parent Pointer to an SDF element to parse.
    public: Boat() = default; 
    public: explicit Boat(Tension *_parent);

    
    /// TODO:fill out the topic message and data type 
    ///\brief Callback for new maginitude commands.
    /// \param[in] _msg The tension message to process.
    public: void OnTensionMagnitudeCB(const std_msgs::Float32::ConstPtr &_msg);

    /// \brief Callback for new tension direction commands.
    /// \param[in] _msg The directoin message to process.
    public: void OnTensionDirectionCB(const geometry_msgs::Vector3::ConstPtr &_msg);

    /// \brief Maximum abs val of incoming Tension.
    public: double maxMag;

    /// \brief Direction of Tension Vector
    public: geometry_msgs::Vector3 currDirec;

    /// \brief Link where tension force is applied.
    public: physics::LinkPtr link;

    /// \brief Topic name for incoming ROS tension commands.
    public: std::string magTopic;

    /// \brief Subscription to tension magnitude.
    public: ros::Subscriber magSub;

    /// \brief Topic name for incoming ROS tension direction.
    public: std::string direcTopic;

    /// \brief Subscription to tension direction.
    public: ros::Subscriber direcSub;

    /// \brief Current, most recent Magnitude.
    public: double currMag;

    /// \brief Application point of the force in the link at the body frame.
    public: geometry_msgs::Point application_point;
    public: double application_point_x;
    public: double application_point_y;
    public: double application_point_z;

    /// \brief Last time received a command via ROS topic.
    public: common::Time lastCmdTime;

    /// \brief Last time of update
    public: common::Time lastAngleUpdateTime;

    /// \brief Plugin parent pointer - for accessing world, etc.
    protected: Tension *plugin;
    

  };

  /// \brief A plugin to simulate a tension force applied to a model.
  /// This plugin accepts the following SDF parameters.
  /// for more information.
  ///
  ///   <robotNamespace>: Namespace prefix for USV.
  ///   <cmdTimeout>:  Timeout, after which tension is set to zero [s].
  ///
  ///   Required elements:
  ///   <linkName>: Name of the link on which to apply thrust forces.
  ///   <magTopic>: The ROS topic to control the tension magnitude,
  ///   <direcTopic>: The ROS topic to control the direction of the 
  ///                 tension vector
  ///   Optional elements:
  ///   <maxMag>:Maximum (abs val) of tension magnitude,
  ///   defualt is XX Newtowns
  ///
  /// Here is an example:
  ///
  ///    <plugin name="example" filename="libusv_gazebo_thrust_plugin.so">
  ///      <!-- General plugin parameters -->
  ///      <cmdTimeout>1.0</cmdTimeout>
  ///      <publisherRate>1.0</publisherRate>
  ///      <boat>
  ///        <linkName>base_link</linkName>
  ///        <magTopic>tension_mag</magTopic>
  ///        <direcTopic>tension_direction</direcTopic>
  ///        <maxMag>10.0</maxMag>
  ///        <applicationPointX>0</applicationPointX>
  ///        <applicationPointY>0</applicationPointY>
  ///        <applicationPointZ>0</applicationPointZ>
  ///      </boat>
  ///    </plugin>

  class Tension : public ModelPlugin //replacing UsvThrust
  {
    /// \brief Vector of boat instances.
    public: Boat boat;

    /// \brief Constructor.
    public: Tension() = default;

    /// \brief Destructor.
    public: virtual ~Tension() = default;

    // Documentation inherited.
    public: virtual void Load(physics::ModelPtr _parent,
                              sdf::ElementPtr _sdf);

    /// \brief Callback executed at every physics update.
    protected: virtual void Update();

    /// \brief Convenience function for getting SDF parametersi.//
    /// \param[in] _sdfPtr Pointer to an SDF element to parse.
    /// \param[in] _paramName The name of the element to parse.
    /// \param[in] _defaultVal The default value returned if the element
    /// does not exist.
    /// \return The value parsed.
    private: double SdfParamDouble(sdf::ElementPtr _sdfPtr,
                                   const std::string &_paramName,
                                   const double _defaultVal) const;

    /// \brief Takes ROS tension vector direction and mapps it to tension vector
    /// \param[in] _direc ROS direction
    /// \return force of tension vector.
    private: geometry_msgs::Vector3 DirecToTension(const double _mag,
                                   const double _maxMag,
                                   geometry_msgs::Vector3 _currDirec);


    /// \brief A mutex to protect member variables accessed during
    /// OnThustCmd() and Update().
    public: std::mutex mutex;

    /// \brief The ROS node handler used for communications.
    private: std::unique_ptr<ros::NodeHandle> rosnode;

    // comment
    public: geometry_msgs::Vector3 direc_unit;

    /// important
    /// \brief Pointer to the Gazebo world, retrieved when the model is loaded.
    public: physics::WorldPtr world;

    /// \brief Pointer to Gazebo parent model, retrieved when the model is
    /// loaded.
    private: physics::ModelPtr model;

    /// \brief Timeout for receiving Drive commands [s].
    private: double cmdTimeout;

    /// \brief Pointer to the update event connection.
    private: event::ConnectionPtr updateConnection;

    /// \brief Update rate (Hz) of the joint ROS publisher.
    private: double publisherRate = 100.0;

    /// \brief The last time we publish joints.
    private: common::Time prevUpdateTime;

  };
}

#endif
