#include <ros/ros.h>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <hector_gazebo_plugins/update_timer.h>
#include "tilt_srv/Sensor.h"
#include "tilt_srv/Sensor_srv.h"
#include "tilt_srv/Ref.h"
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
	LoggerPtr loggerMyMain(Logger::getLogger( "main"));	

	class ServoMotorPlugin : public ModelPlugin
	{
		public: ServoMotorPlugin(); 
  		public:virtual ~ServoMotorPlugin(); 
		public:virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf); 
  		public: virtual void Reset();  
  		protected: virtual void Update(); 
		public: void Controlador();
		public: void CallbackReferencias(tilt_srv::Ref);		
		private: int16_t saturate(float , const float );
		private: float velocity_feedforward(float ); 
		private: float velocity_controller(float, float);
		private: float position_controller(float, float );

		private:  
			std::string path;
			std::string NameOfJoint_;
			std::string NameOfNode_;
			std::string TopicSubscriber_;
			std::string TopicPublisher_;
			std::string Modo_;
			physics::WorldPtr world; 
			physics::JointPtr junta;  
			UpdateTimer updateTimer;
  			event::ConnectionPtr updateConnection;
			ros::NodeHandle node_handle_;
			ros::Publisher motor_publisher_;
			ros::Subscriber motor_subscriber_;
			ros::CallbackQueue queue;
			double refvalue;
			double ang;
			double vel_ang;	
			double Kpx, Kpv;
			double Kix, Kiv;
			double Kdx, Kdv;
			double torque;
			double torquepub;
			tilt_srv::Sensor sensor;
			boost::mutex lock;
			bool ValuesOfMotor(tilt_srv::Sensor_srv::Request &req, tilt_srv::Sensor_srv::Response &res);
			ros::ServiceServer DataMotorService;

	};
}
