

namespace gazebo 
{
	class gps2 : public SensorPlugin
	{
		UpdateTimer updateTimer;
  		event::ConnectionPtr updateConnection;

		public:
		gps2()
		{

		}

		~gps2()
		{

		}

		void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
		{		
			world = _model->GetWorld();

			Reset();
			updateTimer.Load(world, _sdf);
	  		updateConnection = updateTimer.Connect(boost::bind(&ServoMotorPlugin::Update, this));
		}

		void Reset()
		{

		}

		void Update()
		{	

		}	

	}
	
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosSonar)

} // namespace gazebo
