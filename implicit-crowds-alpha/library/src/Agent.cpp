#include "Agent.h"

Agent::Agent()
{
	enabled = false;
	_proximityToken = NULL;
}

Agent::~Agent()
{
	destroy();
}

void Agent::destroy()
{
	if (_proximityToken != NULL)
	{
		delete _proximityToken;
		_proximityToken = 0x0;
	}
}

void Agent::init(Vector2D position, Vector2D goal, Vector2D velocity, double radius, double prefSpeed, double maxSpeed, double goalRadius, int gid, int input_id, SpatialProximityDatabase *const pd)
{
	// initialize the agent based on the initial conditions
	pos = position;
	r = radius;
	prefVel = prefSpeed;
	_id = input_id;
	activeid = input_id;
	groupid = gid;
	goalRadiusSq = goalRadius*goalRadius;
	vel = velocity;
	_goal = goal;
	_orientation = (_goal - pos).normalized();
	enabled = true;
	_proximityToken = pd->allocateToken(this);
	_proximityToken->updateForNewPosition(position);
	_path.push_back(get_position());
	_orientations.push_back(_orientation);
}


void Agent::goForward(double dt)
{
	_vPref = _goal - pos;
	double distSqToGoal = _vPref.squaredNorm();
	if (distSqToGoal < goalRadiusSq)
	{
		destroy();
		enabled = false;
		return;
	}
	if (prefVel * dt*prefVel * dt > distSqToGoal)
		_vPref = _vPref / dt;
	else
		_vPref *= prefVel / sqrt(distSqToGoal);
}



void Agent::update(double dt)
{
	pos += vel * dt;
	if (vel.x() != 0 || vel.y() != 0)
		_orientation = _orientation + (vel.normalized() - _orientation) * 0.4;
	_proximityToken->updateForNewPosition(pos);
	_path.push_back(get_position());
	_orientations.push_back(_orientation);
}

void Agent::findNeighbors(double neighborDist, vector<Item*>& nn)
{
	_proximityToken->findNeighbors(pos, neighborDist, nn);
}

bool Agent::isAgent() {
	return true;
}
/// Returns true if the agent is active.
bool Agent::isActived() const {
	return enabled;
}
/// Returns the position of the agent.  
Vector2D Agent::get_position() const {
	return pos;
}
/// Returns the velocity of the agent.  
Vector2D Agent::velocity() const {
	return vel;
}
/// Returns the goals of the agent.  
Vector2D Agent::goal() const {
	return _goal;
}
/// Returns the preferred velocity of the agent.  
Vector2D Agent::vPref() const {
	return _vPref;
}
/// Returns the orientation of the agent.  
Vector2D Agent::orientation() const {
	return _orientation;
}
/// Returns the preferred speed of the agent.  
double Agent::prefSpeed() const {
	return prefVel;
}
/// Returns the radius of the agent.  
double Agent::radius() const {
	return r;
}
/// Returns the id of the agent.  
int Agent::id() const {
	return _id;
}
/// Returns the active id of the agent.  
int Agent::getActiveID() const {
	return activeid;
}
/// Returns the group id of the agent.  
int Agent::gid() const {
	return groupid;
}
/// Sets the preferred velocity of the agent to a specific value.	
void Agent::setPreferredVelocity(const Vector2D& v) {
	_vPref = v;
}
/// Sets the  velocity of the agent to a specific value.	
void Agent::setVelocity(const Vector2D& v) {
	vel = v;
}
/// Sets the active id of the agent to a specific value.	
void Agent::setActiveID(const int& id) {
	activeid = _id;
}
/// Returns the path of the agent
vector<Vector2D> Agent::path(void) const {
	return _path;
}
/// Returns the orientations of the agent across its trajectory
vector<Vector2D> Agent::orientations(void) const {
	return _orientations;
}