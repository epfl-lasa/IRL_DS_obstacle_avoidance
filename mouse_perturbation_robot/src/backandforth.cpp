	void MotionGenerator::backAndForthMotion()
{


	// Update current time
	double currentTime = ros::Time::now().toSec();

	Eigen::Vector3f gains, error;
	gains.setConstant(0.0f);
	error.setConstant(0.0f);

	switch (_state)
	{
		case State::INIT:
		{
			_phaseDuration = _initDuration;
		}
		case State::CLEAN_MOTION:
		{
			// Compute desired target position
			_xd = _x0+_targetOffset.col(_currentTarget);

			// Compute distance to target
			float distance = (_xd-_x).norm();
			if(distance < TARGET_TOLERANCE)
			{
				// Target is reached 
				_trialCount++;
				_previousTarget = _currentTarget;

				// Update target and obstacle
				if (_currentTarget == Target::A)
				{	
					_currentTarget = Target::B;
					if(_useArduino)
					{
						sendValueArduino(4);
					}
				}
				else
				{
					_currentTarget = Target::A;
					if(_useArduino)
					{
						sendValueArduino(2);
					}
				}
				_obs._x0 = _x0 + (_targetOffset.col(_currentTarget)+_targetOffset.col(_previousTarget))/2;
				_obs._x0(2) -= 0.05f;

				// Random change in trajectory parameters
				if (_switchingTrajectories and (float)std::rand()/RAND_MAX>0.5)
				{
					//sendMsgForParameterUpdate(_msg_para_up);
				}

				obsModulator.setObstacle(_obs);
				// Update motion and perturbation direction
				Eigen::Vector3f temp;
				temp << 0.0f,0.0f,1.0f;
				_motionDirection = _targetOffset.col(_currentTarget)-_targetOffset.col(_previousTarget);
				_motionDirection.normalize();
				_perturbationDirection = temp.cross(_motionDirection);
				_perturbationDirection.normalize();

				// Go in pause state
				_reachedTime = ros::Time::now().toSec();
				_state = State::PAUSE;
			}

			// Compute the gain matrix M = B*L*B'
			// B is an orthogonal matrix containing the directions corrected
			// L is a diagonal matrix defining the correction gains along the directions
			Eigen::Matrix3f B,L;
			B.col(0) = _motionDirection;
			B.col(1) = _perturbationDirection;
			B.col(2) << 0.0f,0.0f,1.0f;
			gains << 10.0f, 10.0f, 30.0f;

			// Compute error and desired velocity
			error = _xd-_x;
			L = gains.asDiagonal();
			_vd = B*L*B.transpose()*error;

			_vd = obsModulator.obsModulationEllipsoid(_x, _vd, false);
			// Check for end of clean motion phase
			if(currentTime-_initTime > _phaseDuration and _perturbationFlag)
			{
				// Go to jerky motion phase
				_perturbation = true;
				_state = State::JERKY_MOTION;
				_initTime = ros::Time::now().toSec();
				_phaseDuration = _jerkyMotionDuration;
				if(_useArduino)
				{
					sendValueArduino(255);
				}
			}
			break;
		}
		case State::PAUSE:
		{
			// Send zero velocity
			_vd.setConstant(0.0f);

			// Check for end of pause
			if(currentTime-_reachedTime>_pauseDuration)
			{
				// Go back to clean motion phase
				_state = State::CLEAN_MOTION;
			}
			break;
		}
		case State::JERKY_MOTION:
		{
			_perturbationDirection << 0.0f,0.0f,1.0f;
			// Update perturbation offset based on perturbation velocity + apply saturation
			_perturbationOffset += PERTURBATION_VELOCITY*(-1+2*(float)std::rand()/RAND_MAX)*_dt*_perturbationDirection;
			if(_perturbationOffset.norm()>MAX_PERTURBATION_OFFSET)
			{
				_perturbationOffset *= MAX_PERTURBATION_OFFSET/_perturbationOffset.norm();
			}
			while(_perturbationOffset.norm()< MIN_PERTURBATION_OFFSET)
			{
				_perturbationOffset = MAX_PERTURBATION_OFFSET*(-1+2*(float)std::rand()/RAND_MAX)*_perturbationDirection;
			}

			// Compute desired position by considering perturbation offset
			_xd = _x+_perturbationOffset;

			// Compute the gain matrix M = B*L*B'
			// The gain along the motion direction is set to zero to stay in place 
			// The gain along the z axis is kept to keep the height
			Eigen::Matrix3f B,L;
			B.col(0) = _motionDirection;
			B.col(1) = _perturbationDirection;
			B.col(2) << 1.0f,0.0f,0.0f;
			gains << 0, 30.0f, 10.0f;

			error = _xd-_x;
			L = gains.asDiagonal();
			_vd = B*L*B.transpose()*error;

			// Check for end of jerky motion phase
			if(currentTime-_initTime > _phaseDuration)
			{
				// Update perturbation count + go to clean motion phase
				_perturbationCount++;
				_perturbationOffset.setConstant(0.0f);
				_perturbation = false;
				_state = State::CLEAN_MOTION;
				_initTime = ros::Time::now().toSec();
				_phaseDuration = _minCleanMotionDuration+(_maxCleanMotionDuration-_minCleanMotionDuration)*((float)std::rand()/RAND_MAX);
				if(_useArduino)
				{
					sendValueArduino(255);
				}
			}
			break;
		}
		default:
		{
			break;
		}
	}

	// ROS_INFO_STREAM("Input " << _vd);
	// ROS_INFO_STREAM("Output " << obsModulator.obsModulationEllipsoid(_x, _vd, false));

	// Bound desired velocity // not related to the 
	if (_vd.norm()>0.3f)
	{
		_vd = _vd*0.3f/_vd.norm();
	}

	// Desired quaternion to have the end effector looking down
	_qd << 0.0f, 0.0f, 1.0f, 0.0f;

	// std::cerr << "trialCount: " << _trialCount << " target: " << (int) (_currentTarget) << " perturbation: " << (int) (_perturbation) << " perturbationCount: " << _perturbationCount << std::endl;
	// std::cerr << "target: " << _xd << " position: " << _x  << std::endl;
	// std::cerr << "obstacle: " << _obs._x0  << std::endl;

	}