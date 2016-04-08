#include <ros/ros.h>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <hector_gazebo_plugins/update_timer.h>
#include "tilt_srv/States.h"
#include <iostream>
#include <boost/thread.hpp>
#include <ros/callback_queue.h>

// testes
#include <boost/date_time.hpp>
#include "std_msgs/String.h"

#include <ros/package.h>
#include <log4cxx/logger.h>
#include <log4cxx/xml/domconfigurator.h>

using namespace log4cxx;
using namespace log4cxx::xml;
using namespace log4cxx::helpers;

namespace gazebo
{
	//LoggerPtr loggerMyMain(Logger::getLogger( "main"));	

	class AllData : public ModelPlugin
	{
		public: AllData(); 
  		public:virtual ~AllData(); 
		public:virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf); 
  		public: virtual void Reset();  
  		protected: virtual void Update(); 
		

		private:  
			
			std::string NameOfJointR_;
			std::string NameOfJointL_;
			std::string NameOfNode_;
			std::string link_name_;
			physics::LinkPtr link;
			physics::WorldPtr world; 
			physics::JointPtr juntaR;
			physics::JointPtr juntaL;   
			UpdateTimer updateTimer;
  			event::ConnectionPtr updateConnection;
			ros::NodeHandle node_handle_;
			boost::mutex lock;
			double ang;
			double vel_ang;

			tilt_srv::States X;
			ros::Publisher publisher_;	
			

	};
}
