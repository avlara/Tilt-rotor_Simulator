#ifndef AERO_H
#define AERO_H

#include <ros/ros.h>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <hector_gazebo_plugins/update_timer.h>
#include "tilt_srv/Sensor.h"
#include <gazebo/math/Vector3.hh>
#include <ros/package.h>
#include <log4cxx/logger.h>
#include <log4cxx/xml/domconfigurator.h>

using namespace log4cxx;
using namespace log4cxx::xml;
using namespace log4cxx::helpers;

namespace gazebo
{
	LoggerPtr loggerMyMain(Logger::getLogger( "main"));

	class Aerodinamica : public ModelPlugin
	{
		
		public: Aerodinamica(); 
  		public:virtual ~Aerodinamica(); 
		public:virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf); 
  		public: virtual void Reset();  
  		protected: virtual void Update(); 
		public: void CallbackVR(tilt_srv::Sensor);
		public: void CallbackVL(tilt_srv::Sensor);		

		private: 
			std::string path;
			ros::NodeHandle node_handle_;
			UpdateTimer updateTimer;
			event::ConnectionPtr updateConnection;
			physics::WorldPtr world;
			physics::LinkPtr linkR;
			physics::LinkPtr linkL;
			double ka;
			double kb;
			double vr;
			double vl;
			std::string topic_VR;
			std::string topic_VL;
			std::string NameOfNode_;
			ros::Subscriber motor_subscriberVL_;
			ros::Subscriber motor_subscriberVR_;
			std::string NameOfLinkDir_;
			std::string NameOfLinkEsq_;	 
	};
}

#endif
