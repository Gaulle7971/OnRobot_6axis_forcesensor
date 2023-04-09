#include <stdio.h>
#include <string>
#include <time.h>
#include <ros/ros.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include "std_msgs/Float32MultiArray.h"
#include <serial/serial.h>

#define BUFFER_MAX_LEN 1024
int count=0;
int time_initialization=500;
float fz_sum;
float error_fz;
int main(int argc, char** argv)
{
	ros::init(argc, argv, "forcesensor_node");
	ros::NodeHandle node_handle;
	ros::Rate loop_rate(1000);
	ros::Publisher pub_force = node_handle.advertise<std_msgs::Float32MultiArray>("/force", 100);
	serial::Serial ser;

	uint8_t rxLen = 0;
	uint8_t rxBuffer[BUFFER_MAX_LEN];
	uint8_t rxFlag = 0;
	uint8_t read_buffer[BUFFER_MAX_LEN];
	
	uint32_t timeLast = 0;

	uint16_t readLen = 0;
	uint16_t tempPtr = 0;

	std::string com_port_str;
	char com_port[30];

    if(node_handle.getParam("force_sensor_com_port",com_port_str)){
        std::cout<<"Connecting to forcesensor "<<com_port_str<<std::endl;
    }
    std::strcpy(com_port,com_port_str.c_str());
	
    serial::Timeout to = serial::Timeout::simpleTimeout(100);    
    ser.setPort(com_port_str);
    ser.setBaudrate(1000000);
    ser.setTimeout(to);
    ser.open();

	float fx,fy,fz,tx,ty,tz,act_fz;
    while(ros::ok()){
		readLen = ser.available();
		if(readLen>0){
			if(readLen>BUFFER_MAX_LEN){
				readLen = BUFFER_MAX_LEN;
			}
			readLen = ser.read(read_buffer,readLen);
			tempPtr = 0;
			
			while(tempPtr!=readLen){
				//std::cout<<(int)((uint8_t)read_buffer[tempPtr])<<" ";
				//std::cout<<(int)rxFlag<<" ";
				switch(rxFlag){
					case 0:
						if(read_buffer[tempPtr]==170){
							rxFlag = 1;
						}
						break;
					case 1:
						if(read_buffer[tempPtr]==7){
							rxFlag = 2;
						}
						break;
					case 2:
						if(read_buffer[tempPtr]==8){
							rxFlag = 3;
						}
						break;
					case 3:
						if(read_buffer[tempPtr]==16){
							rxFlag = 4;
							rxLen = 0;
						}
						break;
					case 4:
						rxBuffer[rxLen++]=read_buffer[tempPtr];
						if(rxLen==18){
							rxFlag = 0;
							// std::cout<<"----------------------------------------"<<std::endl;
							// std::cout<<clock()-timeLast<<"us"<<std::endl;
							// timeLast = clock();
							//TODO:力量程换算待处理
							std_msgs::Float32MultiArray force;
							fx = ((short)(rxBuffer[4]<<8|rxBuffer[5]))/6100.0*150.0;
							fy = ((short)(rxBuffer[6]<<8|rxBuffer[7]))/6100.0*150.0;
							fz = ((short)(rxBuffer[8]<<8|rxBuffer[9]))/12542.05*150.0;
							tx = ((short)(rxBuffer[10]<<8|rxBuffer[11]))/6100.0*150.0;
							ty = ((short)(rxBuffer[12]<<8|rxBuffer[13]))/6100.0*150.0;
							tz = ((short)(rxBuffer[14]<<8|rxBuffer[15]))/6100.0*150.0;
							 //std::cout<<"Fx: "<<fx<<"  Fy: "<<fy<<"  Fz: "<<fz<<std::endl;
							 //std::cout<<"Tx: "<<tx<<"  Ty: "<<ty<<"  Tz: "<<tz<<std::endl;
							//std::cout<<"aFx: "<<(rxBuffer[4]<<8|rxBuffer[5])<<"  aFy: "<<(rxBuffer[6]<<8|rxBuffer[7])<<"  aFz: "<<(rxBuffer[8]<<8|rxBuffer[9])<<std::endl;

							if(count<time_initialization){
								fz_sum=fz+fz_sum;
								count++;
							}
							else{
								error_fz=fz_sum/count;
								act_fz=-(fz-error_fz);
								std::cout<<"act_Fz: "<<act_fz<<" error_fz: "<<error_fz<<std::endl;
							}


							force.data.push_back(fx);
							force.data.push_back(fy);
							force.data.push_back(fz);
							force.data.push_back(tx);
							force.data.push_back(ty);
							force.data.push_back(tz);
							force.data.push_back(act_fz);
							pub_force.publish(force);
						}
						break;
					default:
						rxFlag = 0;
						break;
				}
				tempPtr++;
			}
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}