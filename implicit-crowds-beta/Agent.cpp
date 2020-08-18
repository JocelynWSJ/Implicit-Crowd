#include "Agent.h"

Agent::Agent()
{
	_enabled = false;
	_proximityToken = NULL;
}

Agent::~Agent()
{
	destroy();
}

void Agent::init(Vector2D position, Vector2D goal, Vector2D velocity, double radius, double prefSpeed, double maxSpeed, double goalRadius, int gid, int input_id, SpatialProximityDatabase *const pd)
{
	// initialize the agent based on the initial conditions
	_position = position;
	_radius = radius;
	_prefSpeed = prefSpeed;
	_id = input_id;
	_activeid = _id;
	_gid = gid;
	_goalRadiusSq = goalRadius*goalRadius;
	_velocity = velocity;
	_goal = goal;
	_orientation = (_goal - _position).normalized();
	_enabled = true;

	_proximityToken = pd->allocateToken(this);
	_proximityToken->updateForNewPosition(_position);
	
	_path.push_back(_position);
	_orientations.push_back(_orientation);
}

void Agent::destroy()
{
	if (_proximityToken != NULL)
	{
		delete _proximityToken;
		_proximityToken = 0x0;
	}
}


void Agent::doStep(double dt)
{
	_vPref = _goal - _position;
	double distSqToGoal = _vPref.squaredNorm();
	if (distSqToGoal < _goalRadiusSq)
	{
		destroy();
		_enabled = false;
		return;
	}

	// compute preferred velocity
	if (_prefSpeed * dt*_prefSpeed * dt > distSqToGoal)
		_vPref = _vPref / dt;
	else
		_vPref *= _prefSpeed / sqrt(distSqToGoal);
}



void Agent::update(double dt)
{
	//clamp(_velocity, _maxSpeed);		
	_position += _velocity * dt;

	//simple smoothing of the orientation; there are more elaborate approaches
	if (_velocity.x() != 0 || _velocity.y() != 0)
		_orientation = _orientation + (_velocity.normalized() - _orientation) * 0.4;

	// notify proximity database that our position has changed
	_proximityToken->updateForNewPosition(_position);
	// add position and orientation to the list
	_path.push_back(position());
	_orientations.push_back(_orientation);
}

void Agent::findNeighbors(double neighborDist, vector<ProximityDatabaseItem*>& nn)
{
	_proximityToken->findNeighbors(_position, neighborDist, nn);
}