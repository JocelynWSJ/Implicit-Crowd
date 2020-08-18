#include "callisto/VisualizerCallisto.h"
#include "ImplicitSolver.h"
#include "conio.h"
#include <string>
#include <vector>
#include <fstream>
#include <iostream>

using namespace std;
using namespace Callisto;

ImplicitSolver * solver = 0;
int numAgent = 14;
int frames = 80;
double dt = 0.2;
string scenarioFile = "../agents.txt";
string parameterFile = "../settings.ini";


void getNext(std::string& value);
void destroy();

int main(int argc, char **argv)
{
	solver = new ImplicitSolver();
//
	solver->setdt(dt);
	solver->setFrames(frames);

		std::ifstream input(scenarioFile);
		if (input.fail())
		{
			std::cerr << "cannot read scenario file \n";
			destroy();
			exit(1);
		}
		try {
			double xMin, xMax, yMin, yMax;
			input >> xMin;
			input >> xMax;
			input >> yMin;
			input >> yMax;
			solver->init(xMax - xMin, yMax - yMin);

			Vector2D position;
			Vector2D goal;
			Vector2D velocity = Vector2D(0, 0);
			double radius;
			double prefSpeed;
			double maxSpeed = 2.;
			double goalRadius = 1.;
			int gid;

			for (int i = 0; i < numAgent; ++i)
			{
				input >> gid;
				input >> position.x();
				input >> position.y();
				input >> goal.x();
				input >> goal.y();
				input >> prefSpeed;
				input >> radius;
				solver->createAgent(position, goal, velocity, radius, prefSpeed, maxSpeed, goalRadius, gid);
			}
		}
		catch (std::exception &e) {
			std::cerr << "Error reading the scenario file \n";
			destroy();
			exit(1);
		}
		input.close();

	

		if (!parameterFile.empty()) {
			std::ifstream input(parameterFile);
			if (input.fail()) {
				std::cerr << "parameter file not found" << std::endl;
				destroy();
				exit(1);
			}
			string line;
			while (getline(input, line) && input.good()) {
				getNext(line);

				if (!line.empty())
				{
					size_t pos = line.find('=');
					if (pos != string::npos)
					{
						string key = line.substr(0, pos);
						string value = line.substr(pos + 1);
						solver->setParameter(key, value);
					}
				}
			}
			input.close();
		}
	

	//setup Callisto visualizer
	VisualizerCallisto::init();
	VisualizerCallisto::displayEnvironment(scenarioFile.substr(scenarioFile.find_last_of('/') + 1));
	VisualizerCallisto::setBackgroundColour(0, 0.99f, 0.99f, 0.99f);
	VisualizerCallisto::resetDrawing();
	VisualizerCallisto::resetAnimation();

	do {
		solver->updateSimulation();
	} while (!solver->endSimulation());

	const vector<Agent*>& agents = solver->getAgents();
	VisualizerCallisto::resetAnimation();
	//animate ceach agent	
	for (unsigned int i = 0; i < numAgent; ++i)
	{
		const Agent* agent = agents[i];
		double time = 0.;
		int characterId;
		VisualizerCallisto::createCylinderCharacter(characterId, (float)agent->radius(), 2.5f, "agent", true);
		VisualizerCallisto::setCharacterColor(characterId, 0.99f, 0.99f, 0.99f);
		vector<Vector2D> path = agent->path();
		vector<Vector2D>::iterator it_start = path.begin();
		vector<Vector2D>::iterator it_end = path.end();

		vector<Vector2D> orientation = agent->orientations();
		vector<Vector2D>::iterator it_or = orientation.begin();

		for (vector<Vector2D>::iterator it = it_start; it != it_end; ++it, time += dt, ++it_or)
		{

			float pos[] = { (float)it->x(), (float)it->y(), 0 };
			float orientation[] = { 0, 0, float(atan2((*it_or).y(), (*it_or).x()) + M_PI) };
			VisualizerCallisto::addAnimationKey((float)time, pos, characterId, orientation, false);
		}
	}

	//collisto terminate
	while (!_kbhit()) {}
	destroy();
	VisualizerCallisto::destroy();

	return 0;
}

void getNext(std::string& value)
{
	char const* delims = " \t\r\n";
	size_t pos = value.find_first_not_of(delims);
	if (pos == string::npos)
	{
		value.erase();
		return;
	}
	value = value.substr(pos, value.size() - pos);
	value = value.substr(0, value.find_last_not_of(delims) + 1);
}

void destroy()
{
	delete solver;
	solver = 0x0;
}