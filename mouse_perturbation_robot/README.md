# mouse_perturbation_robot

	passiveDS

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
