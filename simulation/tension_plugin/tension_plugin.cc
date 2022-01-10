#include <boost/algorithm/clamp.hpp>
#include <ros/time.h>
#include "geometry_msgs/Vector3.h"
#include <cmath>
#include <functional>

#include "tension_plugin/tension_plugin.hh"

using namespace gazebo;

//////////////////////////////////////////////////
Boat::Boat(Tension *_parent)
{
  // Initialize fields
  this->plugin = _parent;
  this->currMag = 0.0;
  this->application_point = 0.0;

  #if GAZEBO_MAJOR_VERSION >= 8
    this->lastCmdTime = this->plugin->world->SimTime();
  #else
    this->lastCmdTime = this->plugin->world->GetSimTime();
  #endif
}

//////////////////////////////////////////////////
void Thruster::OnTensionMagnitude(const std_msgs::Float32::ConstPtr &_msg)
{
  // When we get a new thrust command!
  ROS_DEBUG_STREAM("New thrust command! " << _msg->data);
  std::lock_guard<std::mutex> lock(this->plugin->mutex);
  #if GAZEBO_MAJOR_VERSION >= 8
    this->lastCmdTime = this->plugin->world->SimTime();
  #else
    this->lastCmdTime = this->plugin->world->GetSimTime();
  #endif
  this->currCmd = _msg->data;
}

//////////////////////////////////////////////////
void Thruster::OnTensionDirection(const std_msgs::Float32::ConstPtr &_msg)
{
  // When we get a new thrust angle!
  ROS_DEBUG_STREAM("New thrust angle! " << _msg->data);
  std::lock_guard<std::mutex> lock(this->plugin->mutex);
  this->desiredAngle = boost::algorithm::clamp(_msg->data, -this->maxAngle,
                                               this->maxAngle);
}

//////////////////////////////////////////////////
double UsvThrust::SdfParamDouble(sdf::ElementPtr _sdfPtr,
  const std::string &_paramName, const double _defaultVal) const
{
  if (!_sdfPtr->HasElement(_paramName))
  {
    ROS_INFO_STREAM("Parameter <" << _paramName << "> not found: "
                    "Using default value of <" << _defaultVal << ">.");
    return _defaultVal;
  }

  double val = _sdfPtr->Get<double>(_paramName);
  ROS_DEBUG_STREAM("Parameter found - setting <" << _paramName <<
                   "> to <" << val << ">.");
  return val;
}

//////////////////////////////////////////////////
void Tension::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  ROS_DEBUG("Loading tension_plugin");
  this->model = _parent;  // important
  this->world = this->model->GetWorld();

  // Get parameters from SDF
  std::string nodeNamespace = "";
  if (_sdf->HasElement("robotNamespace"))
  {
    nodeNamespace = _sdf->Get<std::string>("robotNamespace") + "/";
    ROS_INFO_STREAM("Tension namespace <" << nodeNamespace << ">");
  }

  this->cmdTimeout = this->SdfParamDouble(_sdf, "cmdTimeout", 1.0);

  // Parse joint publisher update rate
  this->publisherRate = this->SdfParamDouble(_sdf, "publisherRate", 100.0);

  ROS_DEBUG_STREAM("Loading model from SDF");

  if (_sdf->HasElement("base_link"))
  {
    sdf::ElementPtr base_linkSDF = _sdf->GetElement("base_link");
    
    // Instatiate
    Boat boat(this);

    std::string linkName = thrusterSDF->Get<std::string>("linkName");
    base_link.link = this->model->GetLink(linkName);
    if (!thruster.link)
    {
      ROS_ERROR_STREAM("Could not find a link by the name <" << linkName
        << "> in the model!");
    }
    else
    {
      ROS_DEBUG_STREAM("Thruster added to link <" << linkName << ">");
    }
    
    // Parse for mag subscription topic
    if (base_linkSDF->HasElement("magTopic"))
    {
      thruster.magTopic = thrusterSDF->Get<std::string>("magTopic");
    }
    else
    {
      ROS_ERROR_STREAM("Please specify a magTopic (for ROS subscription)");
    }

    // Parse for tension direction subscription topic
    if (thrusterSDF->HasElement("direcTopic"))
    {
      thruster.direcTopic = thrusterSDF->Get<std::string>("direcTopic");
    }
    else
    {
      ROS_ERROR_STREAM("Please specify direcTopic (for ROS subscription) ");
    }

    thruster.maxMag = this->SdfParamDouble(thrusterSDF, "maxMag", 100000.0);
    //TODO: change data type to max direc
    thruster.maxAngle = this->SdfParamDouble(thrusterSDF, "maxAngle",
                                             M_PI / 2);

    
  }
  else
  {
    ROS_WARN_STREAM("No 'base_link' tags in description - to what will you apply tension?");
  }

  // Initialize the ROS node
  this->rosnode.reset(new ros::NodeHandle(nodeNamespace));

  #if GAZEBO_MAJOR_VERSION >= 8
    this->prevUpdateTime = this->world->SimTime();
  #else
    this->prevUpdateTime = this->world->GetSimTime();
  #endif

  // Subscribe to commands TODO: add subscriber
  this->boat.magSub = this->rosnode->subscribe(
    this->boat.magTopic, 1, &Thruster::OnTensionMagnitude,
    &this->boat);

  // Subscribe to tension direction.
  this->boat.direcSub = this->rosnode->subscribe(
    this->boat.direcTopic, 1, &Thruster::OnTensionDirection,
    &this->boat);

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&UsvThrust::Update, this));
}

//////////////////////////////////////////////////
geometry_msgs::Vector3 DirecToTension(const double _mag, const double _maxMag,
  const geometry_msgs::Vector3 _Direc) 
{
  geometry_msgs::Vector3 tension(0, 0, 0);  // initialize tension
  direc_unit = Normalize(_Direc)
  if (_mag >= 0.0)
  {
    if (_mag > _maxMag )
    {
      ROS_WARN_STREAM("Magnitude of tension exceeding" << _maxMag << ) 
    }
    tension = std::min(_mag, _maxMag) * direc_unit;
  }
  else
  {
    ROS_ERROR_STREAM("Magnitude of tension cannot be negative") 
  }
  return tension;
}

//////////////////////////////////////////////////
void UsvThrust::Update()
{
  #if GAZEBO_MAJOR_VERSION >= 8
    common::Time now = this->world->SimTime();
  #else
    common::Time now = this->world->GetSimTime();
  #endif
  
  
  std::lock_guard<std::mutex> lock(this->mutex);
  // Enforce command timeout
  double dtc = (now - this->boat.lastCmdTime).Double();
  if (dtc > this->cmdTimeout && this->cmdTimeout > 0.0)
  {
    this->boat[i].currCmd = 0.0;
    ROS_DEBUG_STREAM_THROTTLE(1.0, "[" << i << "] Cmd Timeout");
  }

  // Apply the thrust mapping
  geometry_msgs::Vector3 tforcev(0, 0, 0);
  tforcev = this->DirecToTension(this->boat.currMag, this->boat.maxMag,
                                    this->boat.Direc);
  geometry_msgs::Vector3 application_point(0, 0, 0);

  this->boat.link->AddForceAtRelativePosition(tforcevi, application_point);
  //virtual void gazebo::physics::Link::AddForceAtRelativePosition(const math::Vector3 &_force, const math::Vector3 &_relPos) 	

}


GZ_REGISTER_MODEL_PLUGIN(Tension);
