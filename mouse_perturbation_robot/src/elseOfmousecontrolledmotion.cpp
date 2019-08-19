else
			{
				// won't visiting here at all

				// Go to starting point

				// Compute desired target position
				_xd = _x0+_targetOffset.col(_previousTarget);
				// Compute distance to target
				float distance = (_xd-_x).norm();

				_eventLogger |= 1;

				if (_previousTarget == Target::A)
				{
					_eventLogger |= 1 << 2;
				}
				else
				{
					_eventLogger |= 1 << 1;
				}

				if(distance < TARGET_TOLERANCE)
				{
					if (_previousTarget != _currentTarget)
					{
						_eventLogger = 15;
						_trialCount++;
						if (_switchingTrajectories and (float)std::rand()/RAND_MAX>0.25)
						{	
							_msg_para_up.data = 0.0f;
							//sendMsgForParameterUpdate(_msg_para_up);
						}
					}
					else
					{
						_eventLogger = 0;
					}

					_currentTarget = _previousTarget;
					// Update target
					_reachedTime = ros::Time::now().toSec();
					// _state = State::PAUSE;
				}

				obsModulator.setObstacle(_obs); // sf is set here

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

			}