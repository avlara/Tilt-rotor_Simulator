#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include <gazebo/physics/World.hh>
#include "std_msgs/String.h"

namespace gazebo
{
class WorldPluginTutorial : public WorldPlugin
{
	public:

	physics::WorldPtr objeto;
	ros::Subscriber sub;
	ros::NodeHandle n;

  	WorldPluginTutorial() : WorldPlugin()
  	{
  	}

	void chatterCallback(const std_msgs::String::ConstPtr& msg)
	{
  		//ROS_INFO("I heard: [%s]", msg->data.c_str());
		objeto->Step(1);
	}

  	void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
  	{
    		// Make sure the ROS node for Gazebo has already been initialized                      

		_world->SetPaused(true);                                                              
    		if (!ros::isInitialized())
    		{
      			ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        			<< "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      			return;
    		}	

		//ros::init(argc, argv, "Teste");
		sub = n.subscribe("Step", 1, &gazebo::WorldPluginTutorial::chatterCallback,this);

		// Create our node for communication
  		//transport::NodePtr node(new gazebo::transport::Node());
  		//node->Init();
		
		//_world->Stop();
		objeto = _world;
		
		//objeto->Step(1);
		//objeto2 = objeto;
		/*while(1)
		{
			objeto.Step(1);		
		}*/
  		// Listen to Gazebo world_stats topic
  		//transport::SubscriberPtr sub = node->Subscribe("~/world_stats", &gazebo::WorldPluginTutorial::cb);
		

    		ROS_INFO("Hello World!");
  	}



};
GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
}
