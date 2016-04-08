#include <hector_gazebo_plugins/AllData.h>

namespace gazebo
{
	AllData::AllData()
	{ 
		/*try
		{
			path = ros::package::getPath("hector_gazebo_plugins");
			DOMConfigurator::configure(path+"/LogConfig/Log4cxxConfig_servo.xml");
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
		torque = 0;*/
	}

	AllData::~AllData()
	{	
		try
		{
			updateTimer.Disconnect(updateConnection);
			ros::shutdown();
		}
		catch(std::exception& e)
		{
			//LOG4CXX_ERROR (loggerMyMain, e.what());
			std::cout << e.what() << std::endl;
		} 
	}

	void AllData::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
	{	
		try
		{
	    		if (!ros::isInitialized())
	    		{
				//LOG4CXX_ERROR (loggerMyMain, "Nao inicializado!");
				ROS_INFO("Nao inicializado!");
	      		        return;
	    		}
			
			if (_sdf->HasElement("NameOfNode")) { 	NameOfNode_ = _sdf->GetElement("NameOfNode")->Get<std::string>();}
			else {  std::cout << "Coloque o nome do No: <NameOfNode> x </NameOfNode>" << std::endl;
				return;}

			ros::NodeHandle cp_node_handle_(NameOfNode_);
			node_handle_ = cp_node_handle_;

			if (_sdf->HasElement("NameOfJointR")) { NameOfJointR_ = _sdf->GetElement("NameOfJointR")->Get<std::string>();}
			else {  std::cout << "Coloque o nome da Junta: <NameOfJointR> x </NameOfJointR>" << std::endl;
				return;}

			if (_sdf->HasElement("NameOfJointL")) { NameOfJointL_ = _sdf->GetElement("NameOfJointR")->Get<std::string>();}
			else {  std::cout << "Coloque o nome da Junta: <NameOfJointL> x </NameOfJointL>" << std::endl;
				return;}

			world = _model->GetWorld();	
			juntaR = _model->GetJoint(NameOfJointR_);
			juntaL = _model->GetJoint(NameOfJointL_);


			if (_sdf->HasElement("bodyName")){link_name_ = _sdf->GetElement("bodyName")->GetValue()->GetAsString();}			
			else { std::cout << "GazeboRosGps plugin error: bodyName: "+ link_name_ +" does not exist" << std::endl;
    				return;	}
			link = _model->GetLink(link_name_);			

	  		Reset();
			updateTimer.Load(world, _sdf);
	  		updateConnection = updateTimer.Connect(boost::bind(&AllData::Update, this));
			publisher_ = node_handle_.advertise<tilt_srv::States>("Teste", 1);
			
		}
		catch(std::exception& e)
		{
			//LOG4CXX_ERROR (loggerMyMain, e.what());
			std::cout << e.what() << std::endl;
		}
	}

	void AllData::Reset()
	{
		try
		{
			updateTimer.Reset();
		}
		catch(std::exception& e)
		{
			//LOG4CXX_ERROR (loggerMyMain, e.what());
			std::cout << e.what() << std::endl;
		}
	}

	void AllData::Update()
	{
		try
		{
			common::Time sim_time = world->GetSimTime();
			boost::mutex::scoped_lock scoped_lock(lock);
			X.header.stamp = ros::Time(sim_time.sec, sim_time.nsec);
			X.aR = juntaR->GetAngle(0).Radian();
			X.daR = juntaR->GetVelocity(0); 
			X.aL = juntaL->GetAngle(0).Radian();
			X.daL = juntaL->GetVelocity(0); 
			math::Pose pose = link->GetWorldPose();
			X.x = pose.pos.x;
			X.y = pose.pos.y;
			X.z = pose.pos.z;
			X.r = pose.rot.GetAsEuler( ).x;
			X.p = pose.rot.GetAsEuler( ).y;
			X.yaw = pose.rot.GetAsEuler( ).z;
			math::Vector3 linear = link->GetWorldLinearVel();
			X.vx = linear.x;
			X.vy = linear.y;
			X.vz = linear.z;
			math::Vector3 angular = link->GetWorldAngularVel( );
			X.dr = angular.x;
			X.dp = angular.y;
			X.dyaw = angular.z;	
			publisher_.publish(X);
			
		
		}
		catch(std::exception& e)
		{
			//LOG4CXX_ERROR (loggerMyMain, e.what());
			std::cout << e.what() << std::endl;
		}
	}
	

	GZ_REGISTER_MODEL_PLUGIN(AllData)
}
