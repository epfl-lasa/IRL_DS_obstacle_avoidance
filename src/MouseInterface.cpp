#include "MouseInterface.h"
#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

MouseInterface::MouseInterface(ros::NodeHandle &n, float frequency):
	_n(n),
  _loopRate(frequency),
  _dt(1/frequency)
{

	ROS_INFO_STREAM("The foot mouse interface node is created at: " << _n.getNamespace() << " with freq: " << frequency << "Hz.");
}


bool MouseInterface::init(std::string eventPath) 
{

  // Pulibsher definition
  _pubMouseData = _n.advertise<mouse_perturbation_robot::MouseMsg>("/mouse", 1);

  if((_fd = open(eventPath.c_str(), O_RDONLY)) == -1) 
  {
    return false;
  }

  // Set flags for non blocking read
  int flags = fcntl(_fd, F_GETFL, 0);
  fcntl(_fd, F_SETFL, flags | O_NONBLOCK);

  // Initialize state variables
  _synReceived = false;
  _relReceived = false;

  // Initialize foot mouse message
  _mouseMessage.event = mouse_perturbation_robot::MouseMsg::M_NONE;
  _mouseMessage.buttonState = 0;
  _mouseMessage.relX = 0;
  _mouseMessage.relY = 0;
  _mouseMessage.relZ = 0;
  _mouseMessage.filteredRelX = 0.0f;
  _mouseMessage.filteredRelY = 0.0f;
  _mouseMessage.filteredRelZ = 0.0f;

  // Initialize filtered x,y relative motion 
  _filteredRelX = 0.0f;
  _filteredRelY = 0.0f;
  _filteredRelZ = 0.0f;

  // Dynamic reconfigure definition
  _dynRecCallback = boost::bind(&MouseInterface::dynamicReconfigureCallback, this, _1, _2);
  _dynRecServer.setCallback(_dynRecCallback);


	if (_n.ok())
	{ 
  	// Wait for callback to be called
		ros::spinOnce();
		ROS_INFO("The ros node is ready.");
		return true;
	}
	else 
	{
		ROS_ERROR("The ros node has a problem.");
		return false;
	}
}


void MouseInterface::run() 
{
  while (_n.ok()) 
  {

    // Read foot mouse data;
    // Publish data to topics
    readFootMouse();

    publishData();

    ros::spinOnce();
    _loopRate.sleep();

  }
}


void MouseInterface::publishData()
{
  if(_synReceived)
  {
    _synReceived = false;
    _relReceived = false;

    if(_mouseMessage.event == mouse_perturbation_robot::MouseMsg::M_CURSOR)
    {
      // Update windows for moving average filtering
      if(_winX.size()>_windowSize)
      {
        _winX.pop_front();
      }
      else if(_winX.size() < _windowSize)
      {
        _winX.push_back(_mouseMessage.relX);
      }
      else
      {
        _winX.pop_front();
        _winX.push_back(_mouseMessage.relX);

      }
      
      if(_winY.size()>_windowSize)
      {
        _winY.pop_front();
      }
      else if(_winY.size() < _windowSize)
      {
        _winY.push_back(_mouseMessage.relY);
      }
      else
      {
        _winY.pop_front();
        _winY.push_back(_mouseMessage.relY);

      }
    }

    if(_useMovingAverage) // If moving average is used, compute average relative motion from the windows
    {
      std::cerr << _winX.size() << std::endl;
      if(_winX.size() == 0)
      {
        _filteredRelX = _mouseMessage.relX;
      }
      else
      {
        float temp = 0;
        for(int k = 0; k < _winX.size(); k++)
        {
          temp += (float) _winX[k]; 
        }
        temp/= _winX.size();
        _filteredRelX = temp;
      }

      if(_winY.size() == 0)
      {
        _filteredRelY = _mouseMessage.relX;
      }
      else
      {
        float temp = 0;
        for(int k = 0; k < _winY.size(); k++)
        {
          temp += (float) _winY[k]; 
        }
        temp/= _winY.size();
        _filteredRelY = temp;
      }

      _filteredRelZ = 0.0f;
    }
    else // Use simple 1D low pass filter
    {
      _filteredRelX = _alpha*_filteredRelX+(1.0f-_alpha)*_mouseMessage.relX;
      _filteredRelY = _alpha*_filteredRelY+(1.0f-_alpha)*_mouseMessage.relY;
      _filteredRelZ = _alpha*_filteredRelZ+(1.0f-_alpha)*_mouseMessage.relZ;
    }

    _mouseMessage.filteredRelX = _filteredRelX;
    _mouseMessage.filteredRelY = _filteredRelY;
    _mouseMessage.filteredRelZ = _filteredRelZ;

    std::cerr << "cursor: " << _mouseMessage.relX << " " << _mouseMessage.relY << " " << _mouseMessage.relZ << std::endl;

    // Publish foot mouse message
    _pubMouseData.publish(_mouseMessage);
  }
}

void MouseInterface::readFootMouse() 
{

  fd_set readset;
  FD_ZERO(&readset);
  FD_SET(_fd, &readset);
  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = int(5e4);
  int result = select(_fd+1, &readset, NULL, NULL, &tv);

  if(result > 0 && FD_ISSET(_fd,&readset)) // Check if data available
  {

    // Read data from input device
    result = read(_fd,&_ie,sizeof(struct input_event));

    // Get event type
    switch (_ie.type)
    {
      case EV_KEY:
      {
        std::cerr << (int) _ie.type << std::endl;
        // Get KEY event code
        if(_ie.code == BTN_RIGHT)
        {
          _mouseMessage.event = mouse_perturbation_robot::MouseMsg::M_RIGHT_CLICK;
        }
        else if(_ie.code == BTN_LEFT)
        {
          _mouseMessage.event = mouse_perturbation_robot::MouseMsg::M_LEFT_CLICK;
        }
        else if(_ie.code == KEY_KPMINUS)
        {
          _mouseMessage.event = mouse_perturbation_robot::MouseMsg::M_BTN_B;
        }
        else if(_ie.code == KEY_KPPLUS)
        {
          _mouseMessage.event = mouse_perturbation_robot::MouseMsg::M_BTN_A;
        }

        _mouseMessage.buttonState = _ie.value;
        // std::cerr << (int) _ie.code << " " << (int) _ie.value << std::endl;  

        break;
      }
      case EV_REL:
      {     
        // std::cerr << (int) _ie.code << " " << (int) _ie.value << std::endl;  
        // Get REL event code   
        if(_ie.code == REL_X)
        {
          _mouseMessage.event = mouse_perturbation_robot::MouseMsg::M_CURSOR;
          _mouseMessage.relX = _ie.value;
          _mouseMessage.relY = 0;

          if(!_relReceived)
          {
            _relReceived = true;
          }

        }
        else if(_ie.code == REL_Y)
        {
          _mouseMessage.event = mouse_perturbation_robot::MouseMsg::M_CURSOR;
          _mouseMessage.relY = _ie.value;

          if(!_relReceived)
          {
            _mouseMessage.relX = 0;
            _relReceived = true;
          }
        }
        else if(_ie.code == REL_Z)
        {
          _mouseMessage.event = mouse_perturbation_robot::MouseMsg::M_CURSOR;
          _mouseMessage.relZ = _ie.value;
          if(!_relReceived)
          {
            _mouseMessage.relX = 0;
            _mouseMessage.relY = 0;
            _relReceived = true;
          }
        }
        else if(_ie.code == REL_WHEEL)
        {
          _mouseMessage.event = mouse_perturbation_robot::MouseMsg::M_WHEEL;
          _mouseMessage.relWheel = _ie.value;
        }

        break;
      }
      case EV_SYN:
      {   
        // A SYN event is received between event, it separates them       
        _synReceived = true;
        break;
      }
      default:
      {
        break;
      }
    }
  }
  else // No events happened
  {
    _synReceived = true;
    _mouseMessage.event = mouse_perturbation_robot::MouseMsg::M_NONE;
  }

  // std::cerr << (int) _ie.type << " " << (int) _ie.code << " " << (int) _ie.value << std::endl;
}


void MouseInterface::dynamicReconfigureCallback(mouse_perturbation_robot::mouseInterface_paramsConfig &config, uint32_t level) 
{

  ROS_INFO("Reconfigure request. Updatig the parameters ...");

  _alpha = config.alpha;
  _useMovingAverage = config.useMovingAverage;
  _windowSize = config.windowSize;

}