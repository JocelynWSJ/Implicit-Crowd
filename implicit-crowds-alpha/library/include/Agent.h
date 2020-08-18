#pragma once
#include <Eigen/Dense>
using namespace Eigen;
typedef Eigen::Matrix<double, 2, 1, Eigen::DontAlign> Vector2D;
#include "proximitydatabase/Proximity2D.h"

class Agent : public Item
{
public:
	Agent();
	~Agent();
	void init(Vector2D pos, Vector2D goal, Vector2D velocity, double radius, double prefSpeed, double maxSpeed, double goalRadius, int gid, int id, SpatialProximityDatabase *const);
	void update(double dt);
	void goForward(double dt);

	bool isAgent();
	bool isActived() const;

	Vector2D get_position() const;
	Vector2D velocity() const;
	Vector2D goal() const;
	Vector2D vPref() const;
	Vector2D orientation() const;
	double prefSpeed() const;
	double radius() const;
	int id() const;
	int getActiveID() const;
	int gid() const;
	void setPreferredVelocity(const Vector2D& v);
	void setVelocity(const Vector2D& v);
	void setActiveID(const int& id);
	vector<Vector2D> path(void) const;
	vector<Vector2D> orientations(void) const;
	void findNeighbors(double neighborDist, vector<Item*>& nn);
	inline void destroy();
	bool get_enabled() const { return enabled; }

protected:
	Vector2D pos;
	Vector2D _goal;
	Vector2D vel;
	double r;
	double prefVel;
	double maxVel;
	double goalRadiusSq;
	int groupid;
	int _id;
	Vector2D _vPref;
	bool enabled;
	Vector2D _orientation;
	int activeid;
	ProximityToken* _proximityToken;
	vector<Vector2D> _path;
	vector<Vector2D> _orientations;
};