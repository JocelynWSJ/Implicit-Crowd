#include "CrowdCmd.h"
#include "ImplicitSolver.h"
#include <maya/MGlobal.h>
#include <list>
#include <maya/MSyntax.h>
#include <maya/MArgDatabase.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
using std::string;

const char *commandFlag = "-c", *commandLongFlag = "-command";
const char *scenarioFileFlag = "-s", *scenarioFileLongFlag = "-scenarioFile";
const char *frameFlag = "-f", *frameLongFlag = "-frames";

const char *gidFlag = "-gid", *gidLongFlag = "-groupid";
const char *posXFlag = "-px", *posXLongFlag = "-positionx";
const char *posYFlag = "-py", *posYLongFlag = "-positiony";
const char *goalXFlag = "-gx", *goalXLongFlag = "-goalx"; 
const char *goalYFlag = "-gy", *goalYLongFlag = "-goaly";
const char *prefSpeedFlag = "-v", *prefSpeedLongFlag = "-velocity";
const char *radiusFlag = "-r", *radiusLongFlag = "-radius";

CrowdCmd::CrowdCmd() : MPxCommand()
{
}

CrowdCmd::~CrowdCmd()
{
}

MSyntax CrowdCmd::newSyntax()
{
	MSyntax syntax;
	syntax.addFlag(commandFlag, commandLongFlag, MSyntax::kLong);
	syntax.addFlag(scenarioFileFlag, scenarioFileLongFlag, MSyntax::kString);
	syntax.addFlag(frameFlag, frameLongFlag, MSyntax::kLong);

	syntax.addFlag(gidFlag, gidLongFlag, MSyntax::kLong);
	syntax.addFlag(posXFlag, posXLongFlag, MSyntax::kDouble);
	syntax.addFlag(posYFlag, posYLongFlag, MSyntax::kDouble);
	syntax.addFlag(goalXFlag, goalXLongFlag, MSyntax::kDouble);
	syntax.addFlag(goalYFlag, goalYLongFlag, MSyntax::kDouble);
	syntax.addFlag(prefSpeedFlag, prefSpeedLongFlag, MSyntax::kDouble);
	syntax.addFlag(radiusFlag, radiusLongFlag, MSyntax::kDouble);
	return syntax;
}

void CrowdCmd::getNext(std::string& value)
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

MStatus CrowdCmd::doIt(const MArgList& args)
{
	// message in Maya output window
	cout << "Implement Me!" << endl;
	std::cout.flush();

	// message in scriptor editor
	MGlobal::displayInfo("Implement Me!");
	int commandId = 0;
	int frames = 100;
	int numAgents = 12;
	double dt = 0.2;
	MString scenarioFile = "C:/Users/Jocelyn/Desktop/agents.txt";
	MString parameterFile = "C:/Users/Jocelyn/Desktop/settings.ini";

	MSyntax syntax = newSyntax();
	MArgDatabase argData(syntax, args);

	//Edit scenario File via Maya
	if (argData.isFlagSet(commandFlag)) {
		argData.getFlagArgument(commandFlag, 0, commandId);
	}
	if (argData.isFlagSet(scenarioFileFlag)) {
		argData.getFlagArgument(scenarioFileFlag, 0, scenarioFile);
	}


	// Run the solver implementation
	ImplicitSolver solver;
	solver.setdt(dt);
	solver.setMaxSteps(frames);

	switch (commandId)
	{
	case 0://read data from scenarioFile and simulate
	{
		std::ifstream input(scenarioFile.asChar());
		if (input.fail())
		{
			MGlobal::displayInfo("cannot read scenario file");
			return MStatus::kFailure;
		}
		try {
			double xMin, xMax, yMin, yMax;
			input >> xMin;
			input >> xMax;
			input >> yMin;
			input >> yMax;
			solver.init(xMax - xMin, yMax - yMin);

			input >> numAgents;
			input >> frames;
			Vector2D position;
			Vector2D goal;
			Vector2D velocity = Vector2D(0, 0);
			double radius;
			double prefSpeed;
			double maxSpeed = 2.;
			double goalRadius = 1.;
			int gid;

			//Edit number of frames via Maya
			if (argData.isFlagSet(frameFlag)) {
				argData.getFlagArgument(frameFlag, 0, frames);
			}

			for (int i = 0; i < numAgents; ++i)
			{
				input >> gid;
				input >> position.x();
				input >> position.y();
				input >> goal.x();
				input >> goal.y();
				input >> prefSpeed;
				input >> radius;
				solver.createAgent(position, goal, velocity, radius, prefSpeed, maxSpeed, goalRadius, gid);
				std::string creater_agent = "polyCylinder -r 0.2 -sx 10 -sy 5 -sz 5 -h 1 -name Agent" + std::to_string(i);
				MGlobal::displayInfo("create agent cylinder");
				MGlobal::executeCommand(creater_agent.c_str());

				std::string init_pos = "move -xz " + std::to_string(position.x()) + " " + std::to_string(position.y()) + " " + std::to_string(0) + " Agent" + std::to_string(i);
				MGlobal::executeCommand(init_pos.c_str());
			}
		}
		catch (std::exception &e) {
			MGlobal::displayInfo("Error reading the scenario file ");
			return MStatus::kFailure;
		}
		input.close();

		MGlobal::displayInfo("finish reading scenerio file");

		string pf = parameterFile.asChar();
		if (!pf.empty()) {
			std::ifstream input(pf);
			if (input.fail()) {
				MGlobal::displayInfo("parameter file not found");
				return MStatus::kFailure;
			}
			std::string line;
			while (getline(input, line) && input.good()) {
				getNext(line);

				if (!line.empty())
				{
					size_t pos = line.find('=');
					if (pos != string::npos)
					{
						string key = line.substr(0, pos);
						string value = line.substr(pos + 1);
						solver.setParameter(key, value);
					}
				}
			}
			input.close();
		}

		MGlobal::displayInfo("finish reading parameter file");

		do {
			solver.updateSimulation();
		} while (!solver.endSimulation());

		MGlobal::displayInfo("finish implicit simulation");

		const vector<Agent*>& agents = solver.getAgents();
		//animate ceach agent	
		for (unsigned int i = 0; i < numAgents; ++i)
		{
			const Agent* agent = agents[i];
			double time = 0.;
			int characterId;

			vector<Vector2D> path = agent->path();
			vector<Vector2D>::iterator it_start = path.begin();
			vector<Vector2D>::iterator it_end = path.end();
			vector<Vector2D> orient = agent->orientations();
			vector<Vector2D>::iterator it_or = orient.begin();

			int index = 0;
			float last_pos[] = { (float)it_start->x(), (float)it_start->y(), 0 };
			float last_ori[] = { 0, 0, float(atan2((*orient.begin()).y(), (*orient.begin()).x()) + M_PI) };
			float t_x, t_y, t_z, r_x, r_y, r_z;

			for (vector<Vector2D>::iterator it = it_start; it != it_end; ++it, time += dt, ++it_or)
			{
				///if (index > 0) {
				float pos[] = { (float)it->x(), (float)it->y(), 0 };
				float orientation[] = { 0, 0, float(atan2((*it_or).y(), (*it_or).x()) + M_PI) };

				std::string set_tx = "setAttr Agent" + std::to_string(i) + ".translateX " + std::to_string(pos[0]);
				std::string set_ty = "setAttr Agent" + std::to_string(i) + ".translateY " + std::to_string(pos[2]);
				std::string set_tz = "setAttr Agent" + std::to_string(i) + ".translateZ " + std::to_string(pos[1]);

				std::string set_rx = "setAttr Agent" + std::to_string(i) + ".rotateX " + std::to_string(orientation[0]);
				std::string set_ry = "setAttr Agent" + std::to_string(i) + ".rotateY " + std::to_string(orientation[2]);
				std::string set_rz = "setAttr Agent" + std::to_string(i) + ".rotateZ " + std::to_string(orientation[1]);

				MGlobal::executeCommand(set_tx.c_str());
				MGlobal::executeCommand(set_ty.c_str());
				MGlobal::executeCommand(set_tz.c_str());

				MGlobal::executeCommand(set_rx.c_str());
				MGlobal::executeCommand(set_ry.c_str());
				MGlobal::executeCommand(set_rz.c_str());

				std::string animate_tx = "setKeyframe -attribute translateX -t " + std::to_string(time - dt) + " -t " + std::to_string(time)
					+ " Agent" + std::to_string(i);
				std::string animate_ty = "setKeyframe -attribute translateY -t " + std::to_string(time - dt) + " -t " + std::to_string(time)
					+ " Agent" + std::to_string(i);
				std::string animate_tz = "setKeyframe -attribute translateZ -t " + std::to_string(time - dt) + " -t " + std::to_string(time)
					+ " Agent" + std::to_string(i);

				MGlobal::executeCommand(animate_tx.c_str());
				MGlobal::executeCommand(animate_ty.c_str());
				MGlobal::executeCommand(animate_tz.c_str());

				index++;
				//VisualizerCallisto::addAnimationKey((float)time, pos, characterId, orientation, false);
			}
		}
	}

	//==========================================================================================================================
	case 1://add agent
	{
		Vector2D position;
		Vector2D goal;
		Vector2D velocity = Vector2D(0, 0);
		double radius;
		double prefSpeed;
		double maxSpeed = 2.;
		double goalRadius = 1.;
		int gid;
		
		if (argData.isFlagSet(gidFlag)) {
			argData.getFlagArgument(gidFlag, 0, gid);
		}
		if (argData.isFlagSet(posXFlag)) {
			argData.getFlagArgument(posXFlag, 0, position.x());
		}
		if (argData.isFlagSet(posYFlag)) {
			argData.getFlagArgument(posYFlag, 0, position.y());
		}
		if (argData.isFlagSet(goalXFlag)) {
			argData.getFlagArgument(goalXFlag, 0, goal.x());
		}
		if (argData.isFlagSet(goalYFlag)) {
			argData.getFlagArgument(goalYFlag, 0, goal.x());
		}
		if (argData.isFlagSet(prefSpeedFlag)) {
			argData.getFlagArgument(prefSpeedFlag, 0, prefSpeed);
		}
		if (argData.isFlagSet(radiusFlag)) {
			argData.getFlagArgument(radiusFlag, 0, radius);
		}

	}


	//==========================================================================================================================
	case 2: //add obstacle
	{
		Vector2D position;
		Vector2D goal;
		Vector2D velocity = Vector2D(0, 0);
		double radius;
		double prefSpeed;
		double maxSpeed = 2.;
		double goalRadius = 1.;
		int gid;

		if (argData.isFlagSet(gidFlag)) {
			argData.getFlagArgument(gidFlag, 0, gid);
		}
		if (argData.isFlagSet(posXFlag)) {
			argData.getFlagArgument(posXFlag, 0, position.x());
		}
		if (argData.isFlagSet(posYFlag)) {
			argData.getFlagArgument(posYFlag, 0, position.y());
		}
		if (argData.isFlagSet(goalXFlag)) {
			argData.getFlagArgument(goalXFlag, 0, goal.x());
		}
		if (argData.isFlagSet(goalYFlag)) {
			argData.getFlagArgument(goalYFlag, 0, goal.x());
		}
		if (argData.isFlagSet(prefSpeedFlag)) {
			argData.getFlagArgument(prefSpeedFlag, 0, prefSpeed);
		}
		if (argData.isFlagSet(radiusFlag)) {
			argData.getFlagArgument(radiusFlag, 0, radius);
		}
	}
	default:
		break;
	}




	return MStatus::kSuccess;
}

