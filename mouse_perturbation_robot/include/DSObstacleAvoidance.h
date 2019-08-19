#ifndef __DS_OBSTACLE_AVOIDANCE_H__
#define __DS_OBSTACLE_AVOIDANCE_H__

#include <vector>
#include "Eigen/Eigen"
#include "ros/ros.h"



struct Obstacle
{
    Eigen::Vector3f _a, _p, _x0;
    double _safetyFactor, _rho, _thR;
    bool _tailEffect, _bContour;
};

class DSObstacleAvoidance
{

	private:

		Obstacle _obs;
		Obstacle _obs2;

		Eigen::Vector3f _modulatedVel;
    	Eigen::Matrix3f _modulationMatrix, _rotationMatrix, _basisMatrix;
    	Eigen::Matrix3f _modulationMatrix1;

    	double _gamma;
    	int _numObstacle;

	public:

		DSObstacleAvoidance();

		void setObstacle(Obstacle &obs, Obstacle &obs2, int _numObstacle);

		Eigen::Vector3f obsModulationEllipsoid(Eigen::Vector3f x, Eigen::Vector3f xd, bool bContour, int _numObstacle);

	private:

		void computeBasisMatrix(Eigen::Vector3f x);

};

#endif