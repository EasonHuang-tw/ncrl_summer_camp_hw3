#include "ros/ros.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
#include "sensor_msgs/Imu.h"
#include <math.h>
#include <Eigen/Dense>
using namespace Eigen; 


double dt;
double now_t = -1;
double last_t = -1;
Vector3d last_gyrop = Vector3d::Zero();
Matrix3d last_C = Matrix3d::Identity();
Vector3d last_acc_b(0,0,9.81);
Vector3d last_v_g(0,0,0);
Vector3d last_s_g(0,0,0);

const Vector3d g_g(0,0,9.81);

int get_flag = 1;

double get_t(sensor_msgs::Imu msg){
	return msg.header.stamp.sec +  msg.header.stamp.nsec*1e-9;
}
void chatterCallback(const sensor_msgs::Imu msg)
{	
	double td = 0.002;
	
	Vector3d gyrop(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z); 
	Vector3d acc_b(msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z);
	Matrix3d B;
	Matrix3d C;
	Vector3d acc_g;
	Vector3d v_g;
	Vector3d s_g;
	double sigma =0;
	Vector3d gyrop_mid;
	now_t = get_t(msg);
	if (last_t == -1){
		last_t = get_t(msg);
	}
	else {
		td = now_t-last_t;
		last_t = now_t;
	}

	//Get altitude
	gyrop_mid = (gyrop+last_gyrop)/2;
	B << 	0,-gyrop_mid(2),gyrop_mid(1),
		gyrop_mid(2),0,-gyrop_mid(0),
		-gyrop_mid(1),gyrop_mid(0),0;
	B=B*td;
	sigma = (gyrop_mid*td).norm();			//get sigma by norm
	C = last_C*(Matrix3d::Identity() + sin(sigma)/sigma*B + ((1-cos(sigma))/pow(sigma,2))*B*B);		//get C
	
	//Tracking position
	acc_g = last_C*((last_acc_b+acc_b)/2);		//tracking position (42)
	v_g = last_v_g + td*(acc_g-g_g);		//(45)
	s_g = last_s_g + td*(v_g);			//(46)
	std::cout<<s_g <<std::endl<<std::endl;

	last_acc_b = acc_b;
	last_C = C;
	last_gyrop = gyrop;
	last_s_g =s_g;
	last_v_g = v_g;	
	get_flag = 1;
	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "imu_path");
	ros::NodeHandle n;
 
 
	ros::Subscriber sub = n.subscribe("imu/data", 1000, chatterCallback);
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_maker", 100 );
	visualization_msgs::Marker line_strip;
	line_strip.header.frame_id = "map";
	while(ros::ok()){
  		if(get_flag == 1){
			line_strip.header.stamp = ros::Time::now();		//define line strip
			line_strip.ns = "points_and_lines";
			line_strip.action = visualization_msgs::Marker::ADD;
			line_strip.pose.orientation.w = 1.0;
    			line_strip.id = 1;
    			line_strip.type = visualization_msgs::Marker::LINE_STRIP;
			line_strip.scale.x = 0.1;	
   			line_strip.color.b = 1.0;
    			line_strip.color.a = 1.0;
			
		      	geometry_msgs::Point p;					//define point
		      	p.x = last_s_g(0);
		      	p.y = last_s_g(1);
		      	p.z = last_s_g(2);
		
		      	line_strip.points.push_back(p);
    			marker_pub.publish(line_strip);
			get_flag = 0;
		}	
			
		ros::spinOnce();
	}
	
	return 0;
}
