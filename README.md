# mouse_perturbation_robot

# process explained
	if the mouse is used "if(_mouseInUse or currentTime - _lastMouseTime < _commandLagDuration)", then this trajectory is successed (for now), when the distance is smaller than a tolerance, then it means we arrive the target. -> label sucess and update parameter.

	if the mouse is loose, then failed. wait untill the robot goes back the original position, then label it as failed and update the parameter


	passiveDS?
	TF? listenser?

	cascade

	https://github.com/epfl-lasa/iiwa_ros/tree/custom_controllers

	rosservice list

	rosservice call /iiwa/controller_manager/load_controller '/iiwa/PassiveDS2'

	rosservice call /iiwa/controller_manager/switch_controller "start_controllers:
- '/iiwa/PassiveDS2'
stop_controllers:
- '/iiwa/DSImpedance'
strictness: 0" 

	rosservice call /iiwa/controller_manager/switch_controller "start_controllers:
- '/iiwa/PositionTorqueController'
stop_controllers:
- '/iiwa/PassiveDS2'
strictness: 0" 

	/iiwa/PassiveDS/command



	rosservice call /iiwa/controller_manager/switch_controller "start_controllers:
- '/iiwa/DSImpedance'
stop_controllers:
- '/iiwa/PositionTorqueController'
strictness: 0" 

	why position controller can't be used?

	change the setobstacle part

	eigen vector3f [] difference with ()