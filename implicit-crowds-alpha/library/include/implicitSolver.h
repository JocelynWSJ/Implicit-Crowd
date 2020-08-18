#pragma once
#include "Agent.h"
template <typename T>
using Vector = Eigen::Matrix<T, Eigen::Dynamic, 1>;

/**
* @brief The engine that performs implicit simulations.
*/
class ImplicitSolver
{
public:
	ImplicitSolver();
	~ImplicitSolver();
	void init(double x, double y);
	const vector<Agent*> & getAgents() const { return agents; }
	void setFrames(int st) { maxSteps = st; }
	void setdt(double t) { dt = t; }
	void updateSimulation();
	bool endSimulation();
	void setParameter(std::string key, std::string value);
	void createAgent(Vector2D pos, Vector2D goal, Vector2D velocity, double radius, double prefSpeed, double maxSpeed, double goalRadius, int gid);
	// returns the objective value
	double obj_value(const  VectorXd &v);
	// computes the gradient of the objective
	double obj_gradient(const  VectorXd &v, VectorXd &grad);
	inline void optimize(Vector<double> & v);
	inline bool min_distance(double Pa_x, double Pa_y, double Pb_x, double Pb_y, double Va_x, double Va_y,
		double Vb_x, double Vb_y, double radius, double& energy, double* grad = NULL);
	inline double inverse_energy(double Pa_x, double Pa_y, double Pb_x, double Pb_y, double Va_x, double Va_y,
		double Vb_x, double Vb_y, double radius, double* grad = NULL);
	inline double searchline(const Vector<double> & x0, const Vector<double> & searchDir, const double phi0,
		const Vector<double>& grad, const double alpha_init = 1.0);

protected:
	double  dt;
	double  globalTime;
	SpatialProximityDatabase * database;
	int iteration;
	int maxSteps;
	bool reachedGoals;
	vector<Agent* > agents;
	int max_threads;
	unsigned int numAgents;
	double k, p, t0, eps;
	double ksi;
	double	eta;
	double  neighborDist;
	int newtonIter;
	double eps_x;
	int window;
	VectorXd pos, posNew, vel, vGoal, radius, vNew;
	size_t noVars;
	int activeAgents;
	vector<vector<Item*>> nn;
};