#include "geometry_msgs/Point.h"
#include <boost/algorithm/clamp.hpp>
#include <cmath>
#include <functional>
#include <ros/time.h>

#include "tension_plugin.hh"

using namespace gazebo;

//////////////////////////////////////////////////
Boat::Boat(Tension *_parent) {
  // Initialize fields
  this->plugin = _parent;
  this->currMag = 0.0;

#if GAZEBO_MAJOR_VERSION >= 8
  lastCmdTime = this->plugin->world->SimTime(); // this->
#else
  lastCmdTime = this->plugin->world->GetSimTime(); // this->
#endif
}

//////////////////////////////////////////////////
void Boat::OnTensionMagnitudeCB(const std_msgs::Float32::ConstPtr &_msg) {
  // When we get a new magnitude!
  ROS_DEBUG_STREAM("New magnitude command from magTopic! "
                  << _msg->data);
  std::lock_guard<std::mutex> lock(this->plugin->mutex);
#if GAZEBO_MAJOR_VERSION >= 8
  this->lastCmdTime = this->plugin->world->SimTime();
#else
  this->lastCmdTime = this->plugin->world->GetSimTime();
#endif
  if (_msg->data > this->maxMag) 
  {
    ROS_WARN_STREAM("Magnitude of " << _msg->data << " higher than max magnitude of: " << this->maxMag ); 
  }
  this->currMag = boost::algorithm::clamp(_msg->data, 0,
                                          this->maxMag); // bounds magnitude
}

//////////////////////////////////////////////////
void Boat::OnTensionDirectionCB(const geometry_msgs::Vector3::ConstPtr &_msg) {
  // Callback when we get a new tension direction
  ROS_DEBUG("New tension direction from direcTopic!");
  ROS_DEBUG_STREAM(
      " this is the x value:  "
      << _msg->x);

  // std::lock_guard<std::mutex> lock(this->plugin->mutex);
  this->currDirec = *_msg;
}

//////////////////////////////////////////////////
double Tension::SdfParamDouble(sdf::ElementPtr _sdfPtr,
                               const std::string &_paramName,
                               const double _defaultVal) const {
  // Parses SDF for double and returns a default value if not found
  if (!_sdfPtr->HasElement(_paramName)) {
    ROS_INFO_STREAM("Parameter <" << _paramName
                                  << "> not found: "
                                     "Using default value of <"
                                  << _defaultVal << ">.");//no publishing rate in SDF
    return _defaultVal;
  }

  double val = _sdfPtr->Get<double>(_paramName);
  return val;
}

//////////////////////////////////////////////////
void Tension::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
  ROS_DEBUG("Loading tension_plugin"); 
  this->model = _parent;              // important
  this->world = this->model->GetWorld();

  // Get parameters from SDF  TODO: investigate why there is no namespace
  std::string nodeNamespace = "";
  if (_sdf->HasElement("robotNamespace")) {
    nodeNamespace = _sdf->Get<std::string>("robotNamespace") + "/";
    ROS_DEBUG_STREAM("Tension namespace <" << nodeNamespace << ">");
  }

  this->cmdTimeout = this->SdfParamDouble(_sdf, "cmdTimeout", 1.0);

  // Parse joint publisher update rate
  this->publisherRate = this->SdfParamDouble(_sdf, "publisherRate", 1.0);

  // Parse SDF for link
  ROS_INFO_STREAM("Loading model from SDF");

  if (_sdf->HasElement("boat")) {
    sdf::ElementPtr boatSDF = _sdf->GetElement("boat");

    // Instatiate
    this->boat = Boat(this);

    if (boatSDF->HasElement("linkName")) {
      std::string linkName = boatSDF->Get<std::string>("linkName");
      boat.link = this->model->GetLink(linkName);

      if (!boat.link) {
        ROS_ERROR_STREAM("Could not find a link by the name <"
                         << linkName << "> in the model!");
      } else {
        ROS_INFO_STREAM("Boat link found:  <" << linkName << ">");
      }
    } else {
      ROS_ERROR_STREAM(
          "there is no linkName tag in SDF of model to apply tension");
    }

    // Parse for mag subscription topic
    if (boatSDF->HasElement("magTopic")) {
      boat.magTopic = boatSDF->Get<std::string>("magTopic");
    } else {
      ROS_ERROR_STREAM(
          "Please specify a Magnitude Topic (for ROS subscription)");
    }

    // Parse for tension direction subscription topic
    if (boatSDF->HasElement("direcTopic")) {
      boat.direcTopic = boatSDF->Get<std::string>("direcTopic");
    } else {
      ROS_ERROR_STREAM(
          "Please specify a tension direction Topic (for ROS subscription) ");
    }
    boat.maxMag = this->SdfParamDouble(boatSDF, "maxMag", 1000000.0); //TODO:change to this->
    boat.application_point_x = this->SdfParamDouble(boatSDF, "applicationPointX", 0); 
    boat.application_point_y = this->SdfParamDouble(boatSDF, "applicationPointY", 0); 
    boat.application_point_z = this->SdfParamDouble(boatSDF, "applicationPointZ", 0); 

    // Initialize the ROS node
    this->rosnode.reset(new ros::NodeHandle(nodeNamespace));

#if GAZEBO_MAJOR_VERSION >= 8
    this->prevUpdateTime = this->world->SimTime();
#else
    this->prevUpdateTime = this->world->GetSimTime();
#endif

    // Subscribe to Magnitude
    boat.magSub = this->rosnode->subscribe(boat.magTopic, 1,
                                           &Boat::OnTensionMagnitudeCB, &boat);

    // Subscribe to tension direction.
    boat.direcSub = this->rosnode->subscribe(
        boat.direcTopic, 1, &Boat::OnTensionDirectionCB, &boat);

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&Tension::Update, this));

    // TODO: set limits on tension direction (ex: not below water)
  } else {
    ROS_WARN_STREAM(
        "No 'boat' tags in description - to what will you apply tension?");
  }
}

//////////////////////////////////////////////////
geometry_msgs::Vector3 Tension::DirecToTension(const double _mag,
                        geometry_msgs::Vector3 _currDirec) {
  // Maps the current direction to the tension vector
  
  geometry_msgs::Vector3 tension; 
  
  double norm = sqrt(pow(_currDirec.x,2)+pow(_currDirec.y,2)+pow(_currDirec.z,2));
  if (norm == 0) //TODO: do this more cleanly with a check for empty vector
  {
    direc_unit.x = 0;
    direc_unit.y = 0;
    direc_unit.z = 0;
  }
  else
  {
    direc_unit.x = _currDirec.x/norm;
    direc_unit.y = _currDirec.y/norm;
    direc_unit.z = _currDirec.z/norm;
  }
  if (_mag >= 0.0)
  {
    tension.x = _mag * direc_unit.x;
    tension.y = _mag * direc_unit.y;
    tension.z = _mag * direc_unit.z;
  //TODO: publish the tension to a topic for visualization purposes
  }
  else
  {
    ROS_ERROR_STREAM("Magnitude of tension cannot be negative");
  }
  return tension;
}

//////////////////////////////////////////////////
void Tension::Update() {
#if GAZEBO_MAJOR_VERSION >= 8
  common::Time now = this->world->SimTime();
#else
  common::Time now = this->world->GetSimTime();
#endif

  // td::lock_guard<std::mutex> lock(this->mutex);
  // Enforce command timeout
  double dtc = (now - this->boat.lastCmdTime).Double();
  if (dtc > this->cmdTimeout && this->cmdTimeout > 0.0) {
    this->boat.currMag = 0.0;
    ROS_DEBUG_STREAM_THROTTLE(1.0, "Magnitude Cmd Timeout");
  }
  
  // Apply the tension mapping
  geometry_msgs::Vector3 tforcev;
  tforcev = this->DirecToTension(this->boat.currMag,
                                 this->boat.currDirec);
  ROS_DEBUG_STREAM("Tension applied: " << tforcev);
  ignition::math::Vector3d application_point_gazebo(this->boat.application_point_x, this->boat.application_point_y, this->boat.application_point_z);

  ignition::math::Vector3d tforcev_gazebo(tforcev.x, tforcev.y, tforcev.z);

  this->boat.link->AddForceAtRelativePosition(tforcev_gazebo,
                                              application_point_gazebo);
}

GZ_REGISTER_MODEL_PLUGIN(Tension);
