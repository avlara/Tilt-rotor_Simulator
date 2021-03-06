#include <hector_gazebo_plugins/aerodinamica.h>

//using namespace gazebo::math;

namespace gazebo
{

	Aerodinamica::Aerodinamica()
	{
		try
		{
			path = ros::package::getPath("hector_gazebo_plugins");
			DOMConfigurator::configure(path+"/LogConfig/Log4cxxConfig_aerodinamica.xml");
		}
		catch(std::exception& e)
		{
			std::cout << e.what() << std::endl;
		}
	}

	Aerodinamica::~Aerodinamica()
	{	
		try
		{
			updateTimer.Disconnect(updateConnection);
			ros::shutdown(); 
		}
		catch(std::exception& e)
		{
			LOG4CXX_ERROR (loggerMyMain, e.what());
		}
	}

	void Aerodinamica::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
	{	
		try
		{
			std::cout << "Load" << std::endl;
	    		if (!ros::isInitialized())
	    		{
	      			LOG4CXX_ERROR(loggerMyMain,"Nao inicializado!");
	      		        return;
	    		}
	 
			// ler do arquivo xml
			if (_sdf->HasElement("ka")) { 	ka = _sdf->GetElement("ka")->Get<double>();}
			else {  std::cout << "<ka> x </ka>" << std::endl;
				return;}

			if (_sdf->HasElement("kb")) { 	kb = _sdf->GetElement("kb")->Get<double>();}
			else {  std::cout << "<kb> x </kb>" << std::endl;
				return;}

			if (_sdf->HasElement("topic_VR")) { 	topic_VR = _sdf->GetElement("topic_VR")->Get<std::string>();}
			else {  std::cout << "<topic_VR> x </topic_VR>" << std::endl;
				return;}

			if (_sdf->HasElement("topic_VL")) { 	topic_VL = _sdf->GetElement("topic_VL")->Get<std::string>();}
			else {  std::cout << "<topic_VL> x </topic_VL>" << std::endl;
				return;}

			if (_sdf->HasElement("NameOfNode")) { 	NameOfNode_ = _sdf->GetElement("NameOfNode")->Get<std::string>();}
			else {  std::cout << "<NameOfNode> x </NameOfNode>" << std::endl;
				return;}

			if (_sdf->HasElement("LinkDir")) { NameOfLinkDir_ = _sdf->GetElement("LinkDir")->Get<std::string>();}
			else {  std::cout << "Coloque o nome da Junta: <LinkDir> x </LinkDir>" << std::endl;
				return;}

			if (_sdf->HasElement("LinkEsq")) { 	NameOfLinkEsq_ = _sdf->GetElement("LinkEsq")->Get<std::string>();}
			else {  std::cout << "Coloque o nome da Junta: <LinkEsq> x </LinkEsq>" << std::endl;
				return;}

			//std::cout << topic_VR << std::endl;
			//std::cout << topic_VL << std::endl;

			// capitular elementos da simulação
		
			linkR = _model->GetLink(NameOfLinkDir_);
			linkL = _model->GetLink(NameOfLinkEsq_);	

			// update timer
	  		Reset();
			world = _model->GetWorld();
			updateTimer.Load(world, _sdf);
	  		updateConnection = updateTimer.Connect(boost::bind(&Aerodinamica::Update, this));

			// subscriber e publisher
			ros::NodeHandle cp_node_handle_(NameOfNode_);
			node_handle_ = cp_node_handle_;
			// subscribers
			motor_subscriberVL_ = node_handle_.subscribe(topic_VR, 1, &gazebo::Aerodinamica::CallbackVR, this);
			motor_subscriberVR_ = node_handle_.subscribe(topic_VL, 1, &gazebo::Aerodinamica::CallbackVL, this);
		}
		catch(std::exception& e)
		{
			LOG4CXX_ERROR (loggerMyMain, e.what());
		}
	}

	void Aerodinamica::Reset()
	{
		try
		{
			updateTimer.Reset();
		}
		catch(std::exception& e)
		{
			LOG4CXX_ERROR (loggerMyMain, e.what());
		}
	}

	void Aerodinamica::Update()
	{
		try
		{		
			math::Vector3 forceR(0,0,ka*vr);
			math::Vector3 torqueR(0,0,0);
			math::Vector3 forceL(0,0,ka*vl);
			math::Vector3 torqueL(0,0,0);
					
	  		linkR->AddRelativeForce(forceR);
			linkL->AddRelativeForce(forceL);
		}
		catch(std::exception& e)
		{
			LOG4CXX_ERROR (loggerMyMain, e.what());
		}
	}

	void Aerodinamica::CallbackVR(tilt_srv::Sensor msg)
	{
		try{vr = msg.vel_ang;}
		catch(std::exception& e)
		{
			LOG4CXX_ERROR (loggerMyMain, e.what());
		}
	}
	void Aerodinamica::CallbackVL(tilt_srv::Sensor msg)
	{
		try{vl = msg.vel_ang;}
		catch(std::exception& e)
		{
			LOG4CXX_ERROR (loggerMyMain, e.what());
		}
	}	

	GZ_REGISTER_MODEL_PLUGIN(Aerodinamica)
}
