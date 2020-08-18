#include "ImplicitSolver.h"
#include <omp.h>
#include <algorithm>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
using namespace std;

ImplicitSolver::ImplicitSolver()
{
	numAgents = 0;
	database = NULL;
	max_threads = omp_get_max_threads();
}

ImplicitSolver::~ImplicitSolver()
{
	for (vector<Agent*>::iterator it = agents.begin(); it != agents.end(); ++it) {
		delete *it;
		*it = 0x0;
	}
	if (database != NULL)
	{
		delete database;
		database = 0x0;
	}
}

void ImplicitSolver::init(double x, double y)
{
	srand(23);
	iteration = 0;
	globalTime = 0;
	database = new SpatialProximityDatabase(VectorXd::Zero(2, 1), Vector2D(x, y), Vector2D(10, 10));
	k = 1.5;
	p = 2.;
	t0 = 3.;
	ksi = 2.;
	eps = 0.2;
	eta = 0.01;
	neighborDist = 10.;
	newtonIter = 100;
	window = 5;
	eps_x = 1e-5;
}

void ImplicitSolver::createAgent(Vector2D pos, Vector2D goal, Vector2D velocity, double radius, double prefSpeed, double maxSpeed, double goalRadius, int gid)
{
	Agent* newAgent = new Agent();
	if (newAgent != NULL) {
		newAgent->init(pos, goal, velocity, radius, prefSpeed, maxSpeed, goalRadius, gid, numAgents, database);
		agents.push_back(newAgent);
		++numAgents;
	}
}

void setDouble(const string& value, double& par)
{
	par = atof(value.c_str());
}

void setInt(const string& value, int& par)
{
	par = atoi(value.c_str());
}

void ImplicitSolver::setParameter(string key, string value)
{
	if (key == "k") {
		setDouble(value, k);
	}
	else if (key == "p") {
		setDouble(value, p);
	}
	else if (key == "t0") {
		setDouble(value, t0);
	}
	else if (key == "ksi") {
		setDouble(value, ksi);
	}
	else if (key == "eps") {
		setDouble(value, eps);
	}
	else if (key == "repulsive") {
		setDouble(value, eta);
	}
	else if (key == "neighborDist") {
		setDouble(value, neighborDist);
	}
	else if (key == "newtonIter") {
		setInt(value, newtonIter);
	}
	else if (key == "lbfgsWindow") {
		setInt(value, window);
	}
	else if (key == "eps_x") {
		setDouble(value, eps_x);
	}
}

bool ImplicitSolver::endSimulation()
{
	if (reachedGoals || iteration >= maxSteps) {
		return true;
	}
	else {
		return false;
	}
}

void ImplicitSolver::updateSimulation()
{
	reachedGoals = true;
	activeAgents = 0;
	//std::cout << "update dt " << dt << std::endl;
	for (unsigned int i = 0; i < numAgents; ++i)
	{
		agents[i]->goForward(dt);
		if (agents[i]->get_enabled())
		{
			reachedGoals = false;
			++activeAgents;
		}
	}

	if (reachedGoals) {
		return;
	}

	noVars = activeAgents + activeAgents;
	pos.resize(noVars);
	vel.resize(noVars);
	vGoal.resize(noVars);
	radius.resize(activeAgents);
	nn.resize(activeAgents);
	vNew = VectorXd::Zero(noVars);

	int counter = 0;
	for (unsigned int i = 0; i < numAgents; ++i)
	{
		if (agents[i]->get_enabled())
		{
			size_t id_y = counter + activeAgents;
			pos[counter] = agents[i]->get_position().x();
			pos[id_y] = agents[i]->get_position().y();
			vel[counter] = agents[i]->velocity().x();
			vel[id_y] = agents[i]->velocity().y();

			vGoal[counter] = agents[i]->vPref().x();
			//std::cout << "vG counter is" << vGoal[counter] << std::endl;
			vGoal[id_y] = agents[i]->vPref().y();
			//std::cout << "vG y is" << vGoal[id_y] << std::endl;

			radius[counter] = agents[i]->radius();
			agents[i]->setActiveID(counter);
			nn[counter].clear();
			agents[i]->findNeighbors(neighborDist, nn[counter]);
			++counter;
		}
	}
	this->optimize(vNew);

	for (unsigned int i = 0; i < numAgents; ++i)
	{
		if (agents[i]->get_enabled()) {
			agents[i]->setVelocity(Vector2D(vNew(agents[i]->getActiveID()), vNew(agents[i]->getActiveID() + activeAgents)));
		}
	}

	for (unsigned int i = 0; i < numAgents; ++i)
	{
		if (agents[i]->get_enabled()) {
			agents[i]->update(dt);
		}
	}
	globalTime += dt;
	iteration++;
}

void ImplicitSolver::optimize(Vector<double> & x0)
{
	MatrixXd s = MatrixXd::Zero(noVars, window);
	MatrixXd y = MatrixXd::Zero(noVars, window);
	Vector<double> alpha = Vector<double>::Zero(window);
	Vector<double> rho = Vector<double>::Zero(window);
	Vector<double> grad(noVars), q(noVars), grad_old(noVars), x_old(noVars), s_temp(noVars), y_temp(noVars);
	double f = obj_gradient(x0, grad);
	//std::cout << "f is" << f << std::endl;

	double gamma_k = 1;
	double alpha_init = min(1.0, 1.0 / grad.lpNorm<Eigen::Infinity>());
	int iter;
	int end = 0;
	int j;
	int maxiter = newtonIter;

	for (int index = 0; index < maxiter; index++)
	{
		x_old = x0;
		grad_old = grad;
		q = grad;
		iter = min(window, index);
		j = end;
		for (int i = 0; i < iter; ++i) {
			if (--j == -1) {
				j = window - 1;
			}
			rho(j) = 1.0 / ((s.col(j)).dot(y.col(j)));
			alpha(j) = rho(j) * (s.col(j)).dot(q);
			q = q - alpha(j) * y.col(j);
		}

		q = gamma_k * q;
		for (int i = 0; i < iter; ++i)
		{
			double beta = rho(j) * q.dot(y.col(j));
			q = q + (alpha(j) - beta)*s.col(j);
			if (++j == window) {
				j = 0;
			}
		}
		double dir = q.dot(grad);
		if (dir < 1e-4) {
			q = grad;
			maxiter -= index;
			index = 0;
			alpha_init = min(1.0, 1.0 / grad.lpNorm<Eigen::Infinity>());
		}
		const double rate = searchline(x0, -q, f, grad, alpha_init);
		//std::cout << "rate is" << rate << std::endl;
		x0 = x0 - rate * q;
		s_temp = x0 - x_old;
		if (s_temp.lpNorm<Eigen::Infinity>() < eps_x) {
			break;
		}
		f = obj_gradient(x0, grad);
		//std::cout << "f is" << f << std::endl;
		y_temp = grad - grad_old;
		s.col(end) = s_temp;
		y.col(end) = y_temp;
		gamma_k = s_temp.dot(y_temp) / y_temp.dot(y_temp);
		alpha_init = 1.0;
		if (++end == window) {
			end = 0;
		}
	}

}

double ImplicitSolver::obj_value(const VectorXd &vN)
{
	posNew = pos + vN * dt;
	double f = 0.5*dt*((vN - vel).array().square()).sum() + 0.5*ksi*((vN - vGoal).array().square()).sum();
	bool exit = false;
#pragma omp parallel for shared(exit) reduction(+:f) num_threads(max_threads)
	for (int i = 0; i < activeAgents; ++i)
	{
		if (!exit)
		{
			size_t id_y = i + activeAgents;
			for (unsigned int j = 0; j < nn[i].size() && !exit; ++j)
			{
				const Agent* other = static_cast<Agent*>(nn[i][j]);
				int other_id = other->getActiveID();
				if (other_id > i) {
					size_t other_id_y = other_id + activeAgents;
					double rad = radius[i] + radius[other_id];

					double distance_energy = .0;
					if (min_distance(pos[i], pos[id_y], pos[other_id], pos[other_id_y],
						vN[i], vN[id_y], vN[other_id], vN[other_id_y], rad, distance_energy)) {
						exit = true;
					}
					else {
						double ttc_energy = inverse_energy(posNew[i], posNew[id_y], posNew[other_id], posNew[other_id_y],
							vN[i], vN[id_y], vN[other_id], vN[other_id_y], rad);
						f += ttc_energy;
						f += distance_energy;
					}
				}
			}
		}
	}
	if (exit) {
		f = 9e9;
	}
	return f;
}

double ImplicitSolver::obj_gradient(const VectorXd &vN, VectorXd &grad)
{
	posNew = pos + vN * dt;
	VectorXd vNewMinVel = vN - vel;
	VectorXd vNewMinVGoal = vN - vGoal;
	double f = 0.5 * dt * (vNewMinVel.array().square()).sum() + 0.5 * ksi * (vNewMinVGoal.array().square()).sum();
	//std::cout << "objgradientb init f is" << f << std::endl;

	grad = ksi * vNewMinVGoal + (1 / dt) * vNewMinVel;
	bool exit = false;
#pragma omp parallel for shared(exit) reduction(+:f) num_threads(max_threads)
	for (int i = 0; i < activeAgents; ++i)
	{
		if (!exit) {
			size_t id_y = i + activeAgents;
			for (unsigned int j = 0; j < nn[i].size() && !exit; ++j)
			{
				const Agent* other = static_cast<Agent*>(nn[i][j]);
				int other_id = other->getActiveID();
				if (other_id != i)
				{
					size_t other_id_y = other_id + activeAgents;
					double rad = radius[i] + radius[other_id];
					double distance_energy = 0;
					double g[] = { 0, 0 };
					if (min_distance(pos[i], pos[id_y], pos[other_id], pos[other_id_y], vN[i], vN[id_y], vN[other_id], vN[other_id_y], rad, distance_energy, g)) {
						exit = true;
					}
					else {
						double ttc_energy = inverse_energy(posNew[i], posNew[id_y], posNew[other_id], posNew[other_id_y], vN[i], vN[id_y], vN[other_id], vN[other_id_y], rad, g);
						//std::cout << "ttc energy is" << ttc_energy << std::endl;
						if (other_id > i) {
							f += ttc_energy;
							f += distance_energy;
						}
						grad[i] += g[0];
						grad[id_y] += g[1];
					}
				}
			}
		}
	}

	if (exit) {
		f = 9e9;
	}
	return f;
}

bool ImplicitSolver::min_distance(double Pa_x, double Pa_y, double Pb_x, double Pb_y, double Va_x, double Va_y, double Vb_x, double Vb_y, double radius, double& energy, double* grad)
{
	energy = 0;
	double Xx = Pb_x - Pa_x;
	double Xy = Pb_y - Pa_y;
	double Vx = Va_x - Vb_x;
	double Vy = Va_y - Vb_y;
	double speed = Vx * Vx + Vy * Vy;
	double rate = Xx * Vx + Xy * Vy;
	double tti = rate / (speed + 1e-4);
	tti = max(min(tti, dt), 0.);
	double dx = Vx * tti - Xx;
	double dy = Vy * tti - Xy;
	double d = dx * dx + dy * dy;
	if (d <= radius*radius) {
		return true;
	}
	d = sqrt(d);
	double distance = d - radius;
	energy = min(eta / distance, 9e9);
	if (grad != NULL && rate >0)
	{
		double tti_prime_x = 0, tti_prime_y = 0;
		if (tti > 0 && tti < dt) {
			double tti_prime_x = (Xx - 2 * tti * Vx) / speed;
			double tti_prime_y = (Xy - 2 * tti * Vy) / speed;
		}
		double scale = -eta / (d * distance * distance);
		double distance_prime_x = dx * (tti + Vx * tti_prime_x) + dy * (Vy * tti_prime_x);
		double distance_prime_y = dy * (tti + Vy * tti_prime_y) + dx * (Vx * tti_prime_y);
		grad[0] += scale * distance_prime_x;
		grad[1] += scale * distance_prime_y;
	}
	return false;
}

double ImplicitSolver::inverse_energy(double Pa_x, double Pa_y, double Pb_x, double Pb_y, double Va_x, double Va_y, double Vb_x, double Vb_y, double rad, double* grad)
{
	double f = 0;
	double V_x = Va_x - Vb_x;
	double V_y = Va_y - Vb_y;
	double X_x = Pb_x - Pa_x;
	double X_y = Pb_y - Pa_y;
	double x = sqrt(X_x*X_x + X_y*X_y);
	double Xhat_x = X_x;
	double Xhat_y = X_y;
	if (x > 0) {
		Xhat_x /= x;
		Xhat_y /= x;
	}
	double vp = Xhat_x*V_x + Xhat_y*V_y;
	if (vp < 0)
	{
		return 0;
	}
	double VT_x = V_x - vp*Xhat_x;
	double VT_y = V_y - vp*Xhat_y;
	double vt = sqrt(VT_x*VT_x + VT_y*VT_y);
	double rSq = rad*rad;
	double xMinR = x*x - rSq;
	double xMinR_sqrt = sqrt(xMinR);
	double nominator = sqrt(1 - eps*eps);
	double vtstar = nominator*rad*vp / xMinR_sqrt;
	if (vt < vtstar)
	{
		double discr = sqrt(rSq*vp*vp - xMinR*vt*vt);
		double inv_ttc = (x*vp + discr) / xMinR;
		if (inv_ttc > 0)
		{
			double mult = k*pow(inv_ttc, p - 1)*exp(-(1 / inv_ttc) / t0);
			f = mult*inv_ttc;
			if (grad != NULL)
			{
				double VP_x = vp*Xhat_x;
				double VP_y = vp*Xhat_y;
				double A_x = -X_x + V_x*dt - vp*dt*Xhat_x;
				double A_y = -X_y + V_y*dt - vp*dt*Xhat_y;
				double B_x = (((dt*vp + x)*VT_x)*xMinR / x - X_x*dt*vt*vt + rSq*vp*A_x / x) / discr + dt*VP_x;
				double B_y = (((dt*vp + x)*VT_y)*xMinR / x - X_y*dt*vt*vt + rSq*vp*A_y / x) / discr + dt*VP_y;
				grad[0] += -mult / xMinR*((A_x + B_x)*(p + 1 / (t0*inv_ttc)) - 2 * dt*(1 / t0 + p*inv_ttc)*X_x);
				grad[1] += -mult / xMinR*((A_y + B_y)*(p + 1 / (t0*inv_ttc)) - 2 * dt*(1 / t0 + p*inv_ttc)*X_y);
			}
		}
	}
	else
	{
		double inv_ttc = (x + eps*rad)*vp / xMinR - nominator / eps*(vt - vtstar) / xMinR_sqrt;
		if (inv_ttc > 0)
		{
			double mult = k*exp(-(1 / inv_ttc) / t0);
			f = mult*pow(inv_ttc, p);
			if (grad != NULL)
			{
				double A_x = -X_x / x + V_x*dt / x - vp*dt*Xhat_x / x;
				double A_y = -X_y / x + V_y*dt / x - vp*dt*Xhat_y / x;
				double B_x = ((eps*rad + x)*A_x) / xMinR + (nominator*((VT_x*dt*vp / x + VT_x) / vt + rad*nominator / xMinR_sqrt*(A_x - dt*vp*X_x / (xMinR)))) / (eps*xMinR_sqrt) - dt*X_x / xMinR*(vp*(eps*rad + x) / xMinR - vp / x + inv_ttc);
				double B_y = ((eps*rad + x)*A_y) / xMinR + (nominator*((VT_y*dt*vp / x + VT_y) / vt + rad*nominator / xMinR_sqrt*(A_y - dt*vp*X_y / (xMinR)))) / (eps*xMinR_sqrt) - dt*X_y / xMinR*(vp*(eps*rad + x) / xMinR - vp / x + inv_ttc);
				mult *= -pow(inv_ttc, p - 1)*(p + 1 / (t0*inv_ttc));
				grad[0] += mult*B_x;
				grad[1] += mult*B_y;
			}

		}
	}

	return  f;
}

double ImplicitSolver::searchline(const Vector<double> & x0, const Vector<double> & searchDir, const double phi0, const Vector<double>& grad, const double alpha_init)
{
	double phi_prime = searchDir.dot(grad);
	Vector<double> tmp(noVars);
	for (size_t i = 0; i < noVars; ++i) {
		tmp(i) = max(fabs(x0(i)), 1.);
	}
	double temp = (searchDir.array().abs() / tmp.array()).maxCoeff();
	double alpha_min = 1e-3 / temp;
	Vector<double> x(noVars);
	double c = 1e-4;
	double alpha = alpha_init;
	double alpha_prev = 0;
	double phi_prev = phi0;
	double alpha_next;

	while (true)
	{
		if (alpha < alpha_min) {
			return alpha;
		}
		x = x0 + alpha * searchDir;
		const double phi = obj_value(x);
		if (phi < phi0 + c*alpha*phi_prime) {
			break;
		}
		else {
			if (alpha_prev == 0) {
				alpha_next = -(phi_prime*alpha*alpha) / (2.0 * (phi - phi0 - phi_prime*alpha));
			}
			else {
				double rhs1 = phi - phi0 - alpha*phi_prime;
				double rhs2 = phi_prev - phi0 - alpha_prev*phi_prime;
				double alphaSq = alpha*alpha;
				double alpha2Sq = alpha_prev*alpha_prev;
				double denominator = alpha - alpha_prev;
				double a = (rhs1 / alphaSq - rhs2 / alpha2Sq) / denominator;
				double b = (-alpha_prev*rhs1 / alphaSq + alpha*rhs2 / alpha2Sq) / denominator;

				if (a == 0.0) {
					alpha_next = -phi_prime / (2.0 * b);
				}
				else {
					const double disc = b*b - 3.0*a*phi_prime;
					if (disc < 0.0) {
						alpha_next = 0.5*alpha;
					}
					else if (b <= 0.0) {
						alpha_next = (-b + sqrt(disc)) / (3.0*a);
					}
					else {
						alpha_next = -phi_prime / (b + sqrt(disc));
					}
				}
				if (alpha_next > 0.5 * alpha) {
					alpha_next = 0.5 * alpha;
				}
			}
			alpha_prev = alpha;
			phi_prev = phi;
			alpha = max(alpha_next, 0.1 * alpha);
		}
	}
	return alpha;
}
