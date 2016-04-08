#include <iostream>
#include <csignal>
#include <sys/time.h>
#include "ros/ros.h"
#include "tilt_srv/Imu_srv.h"
#include "tilt_srv/Vector3Stamped_srv.h"
#include "tilt_srv/NavSatFix_srv.h"
#include "tilt_srv/Range_srv.h"
#include "tilt_srv/Sensor_srv.h"
#include "tilt_srv/Ref.h"
#include "tilt_srv/States.h"
#include "std_msgs/String.h"

// variáveis globais
struct sigaction sa;
struct itimerval timer;

// clientes
ros::ServiceClient imu;
ros::ServiceClient gps_fix;
ros::ServiceClient gps_velocity;
ros::ServiceClient magnetic;
ros::ServiceClient sonar;
ros::ServiceClient servo_esq_srv;
ros::ServiceClient servo_dir_srv;
ros::ServiceClient thrust_esq_srv;
ros::ServiceClient thrust_dir_srv;

// estruturas de comunicação
tilt_srv::Imu_srv values_imu;
tilt_srv::NavSatFix_srv values_fix_gps;
tilt_srv::Vector3Stamped_srv values_velocity_gps;
tilt_srv::Vector3Stamped_srv values_magnetic;
tilt_srv::Range_srv values_sonar;
tilt_srv::Sensor_srv values_servo_esq;
tilt_srv::Sensor_srv values_servo_dir;
tilt_srv::Sensor_srv values_thrust_esq;
tilt_srv::Sensor_srv values_thrust_dir;

// mensagens
tilt_srv::Ref thrustdir;
tilt_srv::Ref thrustesq;
tilt_srv::Ref servodir;
tilt_srv::Ref servoesq;


// publicadores
ros::Publisher thrustdir_pub;
ros::Publisher thrustesq_pub;
ros::Publisher servodir_pub;
ros::Publisher servoesq_pub;
ros::Publisher Step_pub;

//subscribers
ros::Subscriber X_sub;

tilt_srv::States msg2;

void X_Callback(tilt_srv::States msg)
{
	//std::cout << msg.header << std::endl;
	msg2 = msg;
}

// temporização
void timer_handler (int signum)
{
	// lendo sensores	
	/*if (!imu.call(values_imu)) ROS_ERROR("Failed to call service IMU");
	if (!gps_fix.call(values_fix_gps)) ROS_ERROR("Failed to call service fix GPS");
	if (!gps_velocity.call(values_velocity_gps)) ROS_ERROR("Failed to call service velocity GPS");
	if (!magnetic.call(values_magnetic)) ROS_ERROR("Failed to call service magnetic");
	if (!sonar.call(values_sonar)) ROS_ERROR("Failed to call service sonar");
	if (!servo_esq_srv.call(values_servo_esq)) ROS_ERROR("Failed to call service servo dir");
	if (!servo_dir_srv.call(values_servo_dir)) ROS_ERROR("Failed to call service servo left");
	if (!thrust_esq_srv.call(values_thrust_esq)) ROS_ERROR("Failed to call service thrust left");
	if (!thrust_dir_srv.call(values_thrust_dir)) ROS_ERROR("Failed to call service thrusr dir");*/
	std::cout << msg2.header << std::endl;
	
	// controlador + filtro
 	/*thrustdir.ref = 40;
	thrustesq.ref = 40;
	servodir.ref = 0;
	servoesq.ref = 0;*/	

	// Atuadores
	/*thrustdir_pub.publish(thrustdir);
	thrustesq_pub.publish(thrustesq);
	servodir_pub.publish(servodir);
	servoesq_pub.publish(servoesq);*/
	std_msgs::String msgpub;
    	std::stringstream ss;
    	ss << "hello world " ;
    	msgpub.data = ss.str();
	Step_pub.publish(msgpub);
	//std::cout << "Entrei" <<  std::endl;	
}

void SetSampleTime(int n) // em ms
{
	memset (&sa, 0, sizeof (sa));
	sa.sa_handler = &timer_handler;
	sigaction (SIGVTALRM, &sa, NULL);

	/* Configure the timer to expire after 250 msec... */
	timer.it_value.tv_sec = 0;
	timer.it_value.tv_usec = n*1000;
	/* ... and every 250 msec after that. */
	timer.it_interval.tv_sec = 0;
	timer.it_interval.tv_usec = n*1000;
	/* Start a virtual timer. It counts down whenever this process is executing. */
	setitimer (ITIMER_VIRTUAL, &timer, NULL);
}

int main (int argc, char **argv)
{
	// declarando nó
	ros::init(argc, argv, "tilt_controller");
	ros::NodeHandle n;
	
	// declarando clientes
	/*imu = n.serviceClient<tilt_srv::Imu_srv>("ValoresIMU");
	gps_fix = n.serviceClient<tilt_srv::NavSatFix_srv>("FixGPS");
	gps_velocity = n.serviceClient<tilt_srv::Vector3Stamped_srv>("VelocityGPS");
	magnetic = n.serviceClient<tilt_srv::Vector3Stamped_srv>("ValoresMagnetometro");
	sonar = n.serviceClient<tilt_srv::Range_srv>("ValoresSonar");
	servo_esq_srv = n.serviceClient<tilt_srv::Sensor_srv>("/Tilt/SensoraL/ValoresMotores");
	servo_dir_srv = n.serviceClient<tilt_srv::Sensor_srv>("/Tilt/SensoraR/ValoresMotores");
	thrust_esq_srv = n.serviceClient<tilt_srv::Sensor_srv>("/Tilt/SensorThrustesq/ValoresMotores");
	thrust_dir_srv = n.serviceClient<tilt_srv::Sensor_srv>("/Tilt/SensorThrustdir/ValoresMotores");*/

	// declarando publicadores
	/*thrustdir_pub = n.advertise<tilt_srv::Ref>("/Tilt/RefThrustdir", 10);
	thrustesq_pub = n.advertise<tilt_srv::Ref>("/Tilt/RefThrustesq", 10);
	servodir_pub = n.advertise<tilt_srv::Ref>("/Tilt/RefaL", 10);
	servoesq_pub = n.advertise<tilt_srv::Ref>("/Tilt/RefaR", 10);*/	
	Step_pub = n.advertise<std_msgs::String>("Step", 1);	

	// declarando subscribers
	//ros::Subscriber sub = n.subscribe("/Tilt/Teste", 1, X_Callback);
	ros::Subscriber sub = n.subscribe("/Tilt/Teste", 1, X_Callback);

	// configurando período de amostragem
	SetSampleTime(250);
	
	// controlador funciona enquanto o usuário não pressionar a tecla ENTER
	while(true)
	{
	    ros::spinOnce();
	}

	return 0;
}
