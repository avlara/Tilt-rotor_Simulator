#include <servo_motor/servo_motor_plug.h>

namespace gazebo
{
        void ServoMotorPlugin::CallbackReferencias(servo_motor::Ref msg){refvalue = msg.ref;}

	float ServoMotorPlugin::position_controller(float r, float y)
	{
		static float e_old = 0, u = 0 ;
		float e = r-y;
		u = Kpx*e-Kiv*e_old;
		//u=u+Kpx*e-Kix*e_old;
		e_old=e;
		return u;
	}

	float ServoMotorPlugin::velocity_controller(float r, float y)
	{
		static float e_old = 0, u = 0, torqueantes = 0;
		float e = r-y;
		u=u+Kpv*e-Kiv*e_old;
		torque = u;
		e_old=e;
		return u;
	}

	float ServoMotorPlugin::velocity_feedforward(float r)
	{
		static float y = 0;
		y=0.6439*y+0.3561*r; 
		return y;
	}
	
	int16_t ServoMotorPlugin::saturate(float x, const float max)
	{
		if (x>max)
			x=max;
		else if (x<(-max))
			x=-max;
		if (x>0) x+=0.5;
		if (x<0) x-=0.5;
	
		return (int16_t)x;
	}
	
	ServoMotorPlugin::ServoMotorPlugin(){ torque = 0;}

	ServoMotorPlugin::~ServoMotorPlugin()
	{	
		updateTimer.Disconnect(updateConnection);
		ros::shutdown(); 
	}

	void ServoMotorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
	{	
    		if (!ros::isInitialized())
    		{
      			ROS_INFO("Nao inicializado!");
      		        return;
    		}

		Kpx = 0.01;
		Kpv = 0.01;
		Kix = 0.01;
		Kiv = 0.01;
		Kdx = 0;
		Kdv = 0;
		Modo_ = "Posição";
		refvalue = 0;
		ang = 0;
		vel_ang = 0;

		if (_sdf->HasElement("NameOfNode")) { 	NameOfNode_ = _sdf->GetElement("NameOfNode")->Get<std::string>();}
		else {  std::cout << "Coloque o nome do No: <NameOfNode> x </NameOfNode>" << std::endl;
			return;}
		
		ros::NodeHandle cp_node_handle_(NameOfNode_);
		node_handle_ = cp_node_handle_;

		if (_sdf->HasElement("NameOfJoint")) { 	NameOfJoint_ = _sdf->GetElement("NameOfJoint")->Get<std::string>();}
		else {  std::cout << "Coloque o nome da Junta: <NameOfJoint> x </NameOfJoint>" << std::endl;
			return;}

		if (_sdf->HasElement("TopicSubscriber")){ TopicSubscriber_ = _sdf->GetElement("TopicSubscriber")->Get<std::string>();}		
		else{	std::cout << "Coloque o nome do tópico a ser lido: <TopicSubscriber> x </TopicSubscriber>" << std::endl;
			return;	}

		if (_sdf->HasElement("TopicPublisher")){ TopicPublisher_ = _sdf->GetElement("TopicPublisher")->Get<std::string>();}		
		else {	std::cout << "Coloque o nome do tópico a ser publicado: <TopicPublisher> x </TopicPublisher>" << std::endl;
			return; }

		if (_sdf->HasElement("Kpx"))     Kpx = _sdf->GetElement("Kpx")->Get<double>();
		if (_sdf->HasElement("Kpv"))     Kpv = _sdf->GetElement("Kpv")->Get<double>();
		if (_sdf->HasElement("Kix"))     Kix = _sdf->GetElement("Kix")->Get<double>();
		if (_sdf->HasElement("Kiv"))     Kiv = _sdf->GetElement("Kiv")->Get<double>();
		if (_sdf->HasElement("Kdx"))     Kdx = _sdf->GetElement("Kdx")->Get<double>();
		if (_sdf->HasElement("Kdv"))     Kdv = _sdf->GetElement("Kdv")->Get<double>();


		if (_sdf->HasElement("Modo"))     Modo_ = _sdf->GetElement("Modo")->Get<std::string>(); 

		world = _model->GetWorld();	
		junta = _model-> GetJoint(NameOfJoint_);

		motor_subscriber_ = node_handle_.subscribe(TopicSubscriber_, 1, &gazebo::ServoMotorPlugin::CallbackReferencias, this);
		motor_publisher_ = node_handle_.advertise<servo_motor::Sensor>(TopicPublisher_, 5);
  		Reset();
		updateTimer.Load(world, _sdf);
  		updateConnection = updateTimer.Connect(boost::bind(&ServoMotorPlugin::Update, this));
	}

	void ServoMotorPlugin::Reset()
	{
		updateTimer.Reset();
	}

	void ServoMotorPlugin::Update()
	{
		servo_motor::Sensor sensor;
		sensor.ang = junta->GetAngle(0).Radian();
		ang = sensor.ang;
		sensor.vel_ang = junta->GetVelocity(0); 
		vel_ang = sensor.vel_ang;
		
		if (Modo_ == "Posição")
		{
			torque = position_controller(refvalue, ang); 
			junta->SetForce(0,torque);
		}
		else if (Modo_ == "Velocidade")
		{
			torque= velocity_controller(refvalue, vel_ang);
			junta->SetForce(0,torque);
		}
		else if (Modo_ == "Torque")
		{
			torque = refvalue;
			junta->SetForce(0,torque);
		}

		sensor.torque = torque;
		motor_publisher_.publish(sensor);

		ros::spinOnce();
	}

	GZ_REGISTER_MODEL_PLUGIN(ServoMotorPlugin)
}
