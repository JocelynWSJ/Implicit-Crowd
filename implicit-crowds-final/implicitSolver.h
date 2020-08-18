#pragma once
#include "Agent.h"
#include <string>
#include <iostream>
using namespace std;
template <typename T>
using Vector = Eigen::Matrix<T, Eigen::Dynamic, 1>;

class ImplicitSolver
{
public:
	ImplicitSolver();
	~ImplicitSolver();
	void init(double x, double y);	
	void createAgent(Vector2D pos, Vector2D goal, Vector2D velocity, double radius, double prefSpeed, double maxSpeed, double goalRadius, int gid, int bool_agent);
	void updateSimulation();
	void setParameter(string key, string value);
	bool endSimulation();

	const vector<Agent*> & getAgents() const { return cmd_agent; }
	Agent* getAgent(int id) const { return _agents[id]; }
	double getTimeStep() const { return _dt; }
	void setdt(double dt) { _dt = dt; }
	int getMaxSteps() const { return _maxSteps; }
	void setMaxSteps(int steps) { _maxSteps = steps; }
	double getGlobalTime() const { return _globalTime; }
	int getNumAgents() const { return _noAgents; }
	int getIterationNumber() const { return _iteration; }
	const vector<Agent*> & getObstacles() const { return _obstacles; }


protected:
	void initializeProblem();
	void finalizeProblem();
	double value(const  VectorXd &x);
	double value(const  VectorXd &x, VectorXd &grad);
	inline double inverse_ttc_energy(double Pa_x, double Pa_y, double Pb_x, double Pb_y, double Va_x, double Va_y, double Vb_x, double Vb_y, double radius, double* grad = NULL);
	inline bool min_distance_energy(double Pa_x, double Pa_y, double Pb_x, double Pb_y, double Va_x, double Va_y, double Vb_x, double Vb_y, double radius, double& energy, double* grad = NULL);
	inline void minimize(Vector<double> & x0);
	inline double linesearch(const Vector<double> & x0, const Vector<double> & searchDir, const double phi0, const Vector<double>& grad, const double alpha_init = 1.0);

protected:
	double  _dt;
	double  _globalTime;
	int _iteration;
	int _maxSteps;
	bool _reachedGoals;
	SpatialProximityDatabase * _spatialDatabase;
	vector<Agent* >  _agents;
	vector<Agent* >  cmd_agent;
	vector<Agent* >  _obstacles;
	int _max_threads;
	unsigned int _noAgents;
	double _k, _p, _t0, _eps;
	double _ksi;
	double	_eta;
	double  _neighborDist;
	int _newtonIter;
	double _eps_x;
	int _window;
	VectorXd _pos, _posNew, _vel, _vGoal, _radius, _vNew;
	size_t _noVars;
	int _activeAgents;
	vector<vector<ProximityDatabaseItem*>> _nn;
};