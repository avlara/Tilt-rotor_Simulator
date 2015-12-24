#include <aerodinamica/aerodinamica.h>

//using namespace gazebo::math;

namespace gazebo
{
	Aerodinamica::Aerodinamica()
	{
		std::cout << "Construtor" << std::endl;	
	}

	Aerodinamica::~Aerodinamica()
	{	
		std::cout << "Destrutor" << std::endl;
		updateTimer.Disconnect(updateConnection);
		ros::shutdown(); 
	}

	void Aerodinamica::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
	{	
		std::cout << "Load" << std::endl;
    		if (!ros::isInitialized())
    		{
      			ROS_INFO("Nao inicializado!");
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

	void Aerodinamica::Reset()
	{
		std::cout << "Reset" << std::endl;
		updateTimer.Reset();
	}

	void Aerodinamica::Update()
	{
		//std::cout << "Vr" << vr << std::endl;
		//std::cout << "Vl" << vl << std::endl;
		math::Vector3 forceR(0,0,ka*vr);
		math::Vector3 torqueR(0,0,0);
		math::Vector3 forceL(0,0,ka*vl);
		math::Vector3 torqueL(0,0,0);
				

		//set force and torque in gazebo
  		linkR->AddRelativeForce(forceR);
		linkL->AddRelativeForce(forceL);
		// função a ser trabalhada
  		//linkR->AddRelativeTorque(torque - link->GetInertial()->GetCoG().Cross(force));
	}

	void Aerodinamica::CallbackVR(servo_motor::Sensor msg){vr = msg.vel_ang;}
	void Aerodinamica::CallbackVL(servo_motor::Sensor msg){vl = msg.vel_ang;}	

	GZ_REGISTER_MODEL_PLUGIN(Aerodinamica)
}
