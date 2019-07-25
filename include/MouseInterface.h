#ifndef __MOUSE_INTERFACE_H__
#define __MOUSE_INTERFACE_H__

#include "ros/ros.h"
#include <linux/input.h>
#include <fcntl.h>
#include <deque>
#include "mouse_perturbation_robot/MouseMsg.h"
#include "mouse_perturbation_robot/mouseInterface_paramsConfig.h"
#include <dynamic_reconfigure/server.h>


class MouseInterface 
{
	private:

		// ROS variables
		ros::NodeHandle _n;
		ros::Rate _loopRate;
		float _dt;

		// Footmouse data publisher declaration
		ros::Publisher _pubMouseData;

		// Footmouse data
		mouse_perturbation_robot::MouseMsg _mouseMessage;

		// Input device variables
		int _fd;									// File descriptor
		struct input_event _ie;		// Input event structure

		// State variables
		bool _synReceived;				// Monitor a SYN event received
		bool _relReceived;				// Monitor a REL event received

		float _filteredRelX;			// Filtered relative x motion
		float _filteredRelY;			// Filtered relative y motion
		float _filteredRelZ;			// Filtered relative z motion

		// Moving average filter variables for cursor data
		std::deque<int> _winX; 		// Window for relative x motion
		std::deque<int> _winY; 		// Window for relative y motion

		// Foot mouse interface configuration variables
		bool _useMovingAverage; 	// Select filtering method for cursor data (moving average or 1D low pass filter) 
		float _alpha; 						// Gain for low pass filter
		int _windowSize;					// Window size for moving average

		// Dynamic reconfigure (server+callback)		
		dynamic_reconfigure::Server<mouse_perturbation_robot::mouseInterface_paramsConfig> _dynRecServer;
		dynamic_reconfigure::Server<mouse_perturbation_robot::mouseInterface_paramsConfig>::CallbackType _dynRecCallback;

	public:

		// Class constructor
		MouseInterface(ros::NodeHandle &n, float frequency);

		// Initialize node
		bool init(std::string eventPath);

		// Run node main loop
		void run();

	private:

		// Read foot mouse data from input device
		void readFootMouse();

		// Publish data to topics
		void publishData();

		// Dynamic reconfigure callback
		void dynamicReconfigureCallback(mouse_perturbation_robot::mouseInterface_paramsConfig &config, uint32_t level);
};


#endif
