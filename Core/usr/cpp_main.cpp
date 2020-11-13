/*
 * cpp_main.c
 */
#include "cpp_main.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "ringbuffer.h"
#include "ros.h"

#include <vector>

#include <string>

#include "std_msgs/Byte.h"
#include "std_msgs/Int8.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int32.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/UInt8MultiArray.h"
#include "nbt.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"

#include "sensor_msgs/Range.h"

extern uint8_t RxBuffer[RxBufferSize];

extern uint8_t nh_connected;

extern float gyroX;
extern float gyroY;
extern float gyroZ;
extern float accelX;
extern float accelY;
extern float accelZ;

extern int8_t speedDataRightFrontWheel;
extern int8_t speedDataLeftFrontWheel;
extern int8_t speedDataRightBackWheel;
extern int8_t speedDataLeftBackWheel;

extern int8_t sideDataRightFrontWheel;
extern int8_t sideDataLeftFrontWheel;
extern int8_t sideDataRightBackWheel;
extern int8_t sideDataLeftBackWheel;

extern uint8_t speedRXDataRightFrontWheel;
extern uint8_t sideRXDataRightFrontWheel;
extern uint8_t speedRXDataRightBackWheel;
extern uint8_t sideRXDataRightBackWheel;
extern uint8_t speedRXDataLeftFrontWheel;
extern uint8_t sideRXDataLeftFrontWheel;
extern uint8_t speedRXDataLeftBackWheel;
extern uint8_t sideRXDataLeftBackWheel;

extern uint8_t state_can;

extern uint8_t sensorData1;
extern uint8_t sensorData2;
extern uint8_t sensorData3;
extern uint8_t sensorData4;
extern uint8_t sensorData5;
extern uint8_t sensorData6;

extern uint8_t sensorData7;
extern uint8_t sensorData8;

extern uint8_t diagnostics_data[6];

extern uint32_t leftCount, rightCount;

extern CAN_RxHeaderTypeDef wheel_RxHeader;

struct ringbuffer rb;

ros::NodeHandle nh;

std_msgs::Int8 uint_msg_right_front;
std_msgs::Int8 uint_msg_left_front;
std_msgs::Int8 uint_msg_right_back;
std_msgs::Int8 uint_msg_left_back;

std_msgs::UInt8MultiArray diagnostics_data_array;
std_msgs::UInt8MultiArray sensors_data_array;

geometry_msgs::Vector3 gyro_msg;
geometry_msgs::Vector3 accel_msg;
geometry_msgs::Vector3 sen1_3_msg;
geometry_msgs::Vector3 sen4_6_msg;
geometry_msgs::Vector3 sen7_8_msg;

extern "C" void rpm_rightFront_subCb(const std_msgs::Int8& msg);
extern "C" void rpm_leftFront_subCb(const std_msgs::Int8& msg);
extern "C" void rpm_rightBack_subCb(const std_msgs::Int8& msg);
extern "C" void rpm_leftBack_subCb(const std_msgs::Int8& msg);
extern "C" void init_ROS();

ros::Publisher stm("stm", &gyro_msg);
ros::Publisher gyro("gyro", &gyro_msg);
ros::Publisher accel("accel", &accel_msg);

ros::Publisher rpm_right_front("rpm_right_front", &uint_msg_right_front);
ros::Publisher rpm_left_front("rpm_left_front", &uint_msg_left_front);
ros::Publisher rpm_right_back("rpm_right_back", &uint_msg_right_back);
ros::Publisher rpm_left_back("rpm_left_back", &uint_msg_left_back);

ros::Subscriber<std_msgs::Int8> rpm_rightFront_sub("rpm_rightFront_sub", rpm_rightFront_subCb);
ros::Subscriber<std_msgs::Int8> rpm_leftFront_sub("rpm_leftFront_sub", rpm_leftFront_subCb);
ros::Subscriber<std_msgs::Int8> rpm_rightBack_sub("rpm_rightBack_sub", rpm_rightBack_subCb);
ros::Subscriber<std_msgs::Int8> rpm_leftBack_sub("rpm_leftBack_sub", rpm_leftBack_subCb);

ros::Publisher diagnostic_data("diagnostics_data", &diagnostics_data_array);
ros::Publisher sensa_data("sensors1_3_data", &sen1_3_msg);
ros::Publisher sensb_data("sensors4_6_data", &sen4_6_msg);
ros::Publisher sensc_data("sensors7_8_data", &sen7_8_msg);

static nbt_t gyro_nbt;
static nbt_t accel_nbt;
static nbt_t rpm_right_front_nbt;
static nbt_t rpm_left_front_nbt;
static nbt_t rpm_right_back_nbt;
static nbt_t rpm_left_back_nbt;
static nbt_t ros_nbt;
static nbt_t diagnostics_data_nbt;
static nbt_t sensors1_3_data_nbt;
static nbt_t sensors4_6_data_nbt;
static nbt_t sensors7_8_data_nbt;

extern "C" void rpm_rightFront_subCb(const std_msgs::Int8& msg)
{
	if (msg.data >= 6) {
		speedDataRightFrontWheel = msg.data;
		sideDataRightFrontWheel = 0; //CW
	}
	else if (msg.data <= -6) {
		speedDataRightFrontWheel = -(msg.data);
		sideDataRightFrontWheel = 1; //CCW
	}
	else if (msg.data == 0) {
		speedDataRightFrontWheel = 0;
		sideDataRightFrontWheel = 2;
	}
}

extern "C" void rpm_leftFront_subCb(const std_msgs::Int8& msg)
{
	if (msg.data >= 6) {
		speedDataLeftFrontWheel = msg.data;
		sideDataLeftFrontWheel = 1; //CCW
	}
	else if (msg.data <= -6) {
		speedDataLeftFrontWheel = -(msg.data);
		sideDataLeftFrontWheel = 0; //CW
	}
	else {
		speedDataLeftFrontWheel = 0;
		sideDataLeftFrontWheel = 2;
	}
}

extern "C" void rpm_rightBack_subCb(const std_msgs::Int8& msg)
{
	if (msg.data >= 6) {
		speedDataRightBackWheel = msg.data;
		sideDataRightBackWheel = 0; //CW
	}
	else if (msg.data <= -6) {
		speedDataRightBackWheel = -(msg.data);
		sideDataRightBackWheel = 1; //CCW
	}
	else {
		speedDataRightBackWheel = 0;
		sideDataRightBackWheel = 2;
	}
}

extern "C" void rpm_leftBack_subCb(const std_msgs::Int8& msg)
{
	if (msg.data >= 6) {
		speedDataLeftBackWheel = msg.data;
		sideDataLeftBackWheel = 1; //CCW
	}
	else if (msg.data <= -6) {
		speedDataLeftBackWheel = -(msg.data);
		sideDataLeftBackWheel = 0; //CW
	}
	else {
		speedDataLeftBackWheel = 0;
		sideDataLeftBackWheel = 2;
	}
}

extern "C" void cdc_receive_put(uint8_t value)
	{
		ringbuffer_putchar(&rb, value);
	}
extern "C" void init_ROS(void)
{
	ringbuffer_init(&rb, RxBuffer, RxBufferSize);
	// Initialize ROS
	nh.initNode();

	nh.subscribe(rpm_rightFront_sub);
	nh.subscribe(rpm_leftFront_sub);
	nh.subscribe(rpm_rightBack_sub);
	nh.subscribe(rpm_leftBack_sub);

	nh.advertise(stm);
	nh.advertise(gyro);
	nh.advertise(accel);
	nh.advertise(rpm_left_front);
	nh.advertise(rpm_right_front);
	nh.advertise(rpm_left_back);
	nh.advertise(rpm_right_back);

	//nh.advertise(diagnostic_data);
	nh.advertise(sensa_data);
	nh.advertise(sensb_data);
	nh.advertise(sensc_data);

	NBT_init(&rpm_left_front_nbt, 5);
	NBT_init(&rpm_right_front_nbt, 5);
	NBT_init(&rpm_left_back_nbt, 5);
	NBT_init(&rpm_right_back_nbt, 5);

	NBT_init(&diagnostics_data_nbt, 5);
	NBT_init(&sensors1_3_data_nbt, 5);
	NBT_init(&sensors4_6_data_nbt, 5);
	NBT_init(&sensors7_8_data_nbt, 5);

	NBT_init(&gyro_nbt, 1);
	NBT_init(&accel_nbt, 1);

	NBT_init(&ros_nbt, 1);
}

extern "C" void sensors1_3_data_handler(void)
{
	//if (NBT_handler(&sensors1_3_data_nbt)) {
		sen1_3_msg.x = static_cast<float>(sensorData1);
		sen1_3_msg.y = static_cast<float>(sensorData2);
		sen1_3_msg.z = static_cast<float>(sensorData3);
    	sensa_data.publish(&sen1_3_msg);

	//}
}

extern "C" void sensors4_6_data_handler(void)
{
	//if (NBT_handler(&sensors4_6_data_nbt)) {
		sen4_6_msg.x = static_cast<float>(sensorData4);
		sen4_6_msg.y = static_cast<float>(sensorData5);
		sen4_6_msg.z = static_cast<float>(sensorData6);
    	sensb_data.publish(&sen4_6_msg);
	//}
}

extern "C" void sensors7_8_data_handler(void)
{
	//if (NBT_handler(&sensors7_8_data_nbt)) {
		sen7_8_msg.x = static_cast<float>(sensorData7);
		sen7_8_msg.y = static_cast<float>(sensorData8);
		sen7_8_msg.z = 0.0;
    	sensc_data.publish(&sen7_8_msg);

	//}
}

extern "C" void diagnostics_data_handler(void)
{
	//if (NBT_handler(&diagnostics_data_nbt)) {
		diagnostics_data_array.data_length = 6;
		diagnostics_data_array.data = diagnostics_data;
    	if (nh.connected()) {
    		diagnostic_data.publish(&diagnostics_data_array);
    	}
	//}
}

extern "C" void rpm_right_front_handler(void)
{
	//if (NBT_handler(&rpm_right_front_nbt))
	//{
		if (sideRXDataRightFrontWheel == 2) {
		    uint_msg_right_front.data = speedRXDataRightFrontWheel;
		}
		else if (sideRXDataRightFrontWheel == 1) {
			uint_msg_right_front.data = -speedRXDataRightFrontWheel;
		}
		else {
			uint_msg_right_front.data = 0;
		}
    	if (nh.connected()) {
    		rpm_right_front.publish(&uint_msg_right_front);
    	}
	//}
}

extern "C" void rpm_left_front_handler(void)
{
	//if (NBT_handler(&rpm_left_front_nbt))
	//{
		if (sideRXDataLeftFrontWheel == 1) {
		    uint_msg_left_front.data = speedRXDataLeftFrontWheel;
		}
		else if (sideRXDataLeftFrontWheel == 2) {
		  	uint_msg_left_front.data = -speedRXDataLeftFrontWheel;
		}
		else {
		  	uint_msg_left_front.data = 0;
		}
    	if (nh.connected()) {
    		rpm_left_front.publish(&uint_msg_left_front);
    	}
	//}
}

extern "C" void rpm_right_back_handler(void)
{
	//if (NBT_handler(&rpm_right_back_nbt))
	//{
		if (sideRXDataRightBackWheel == 2) {
		    uint_msg_right_back.data = speedRXDataRightBackWheel;
		}
		else if (sideRXDataRightBackWheel == 1) {
			uint_msg_right_back.data = -speedRXDataRightBackWheel;
		}
		else {
			uint_msg_right_back.data = 0;
		}
    	if (nh.connected()) {
    		rpm_right_back.publish(&uint_msg_right_back);
    	}
	//}
}

extern "C" void rpm_left_back_handler(void)
{
	//if (NBT_handler(&rpm_left_back_nbt))
	//{
		if (sideRXDataLeftBackWheel == 1) {
		  	uint_msg_left_back.data = speedRXDataLeftBackWheel;
		}
		else if (sideRXDataLeftBackWheel == 2) {
			uint_msg_left_back.data = -speedRXDataLeftBackWheel;
		}
		else {
			uint_msg_left_back.data = 0;
		}
		if (nh.connected()) {
			rpm_left_back.publish(&uint_msg_left_back);
		}
	//}
}

extern "C" void gyro_handler(void)
{
    gyro_msg.x = gyroX;
    gyro_msg.y = gyroY;
    gyro_msg.z = gyroZ;
    if (nh.connected()) {
    	//if (NBT_handler(&gyro_nbt))
    	{
    		gyro.publish(&gyro_msg);
    	}
    }
}

extern "C" void accel_handler(void)
{
    accel_msg.x = accelX;
    accel_msg.y = accelY;
    accel_msg.z = accelZ;
    if (nh.connected()) {
    	//if (NBT_handler(&accel_nbt))
    	{
    		accel.publish(&accel_msg);
    	}
    }
}

extern "C" void spinOnce(void)
{
	//if (NBT_handler(&ros_nbt))	{
		nh.spinOnce();
	//}
}

