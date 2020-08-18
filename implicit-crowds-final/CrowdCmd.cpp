#include "CrowdCmd.h"
#include <maya/MGlobal.h>
#include <list>
#include <maya/MSyntax.h>
#include <maya/MArgDatabase.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <iterator>
#include <errno.h>

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
const char *editFlag = "-ec", *editLongFlag = "-editcount";

//---------------------------------------------ATTENTION----------------------------------------------------
const string TEMP_FILE_NAME = "C:/Users/yuanyao/Desktop/MPlugin/CrowdsParameters/tempScene.txt";
const string ORIGINAL_FILE_NAME = "C:/Users/yuanyao/Desktop/MPlugin/CrowdsParameters/original_file.txt";
MString parameterFile = "C:/Users/yuanyao/Desktop/MPlugin/settings.ini";

CrowdCmd::CrowdCmd() : MPxCommand()
{
	MGlobal::displayInfo("init Crowd Cmd");
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
	syntax.addFlag(editFlag, editLongFlag, MSyntax::kLong);
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

bool CrowdCmd::editAgentNumbersInFile(MString scenarioFile) {
	
	std::ifstream infile(scenarioFile.asChar());
	ofstream outfile(TEMP_FILE_NAME);
	int lines_read = 0;
	std::string line;

	if (infile.is_open()) {
		while (!infile.eof()) {
			lines_read++;
			getline(infile, line);
			//string d1 = "read line # " + to_string(lines_read) + ": " + line;
			//MGlobal::displayInfo(d1.c_str());
			if (lines_read == 3) {
				int read_agent = stoi(line) + 1;
				outfile << read_agent << endl;
				string display = "increment agent numbers to " + to_string(read_agent);
				MGlobal::displayInfo(display.c_str());
			}
			else {
				if (!line.empty()) {
					outfile << line << endl;
				}
			}
		}
		infile.close();
		outfile.close();
		if (edit_count == 1 && !is_file_exist(ORIGINAL_FILE_NAME.c_str())) {
			if (rename(scenarioFile.asChar(), ORIGINAL_FILE_NAME.c_str()) == 0) {
				if (rename(TEMP_FILE_NAME.c_str(), scenarioFile.asChar()) == 0) {
					MGlobal::displayInfo("Finish Changing number of agents in file");
					return true;
				}
			}
		}
		else {
			if (remove(scenarioFile.asChar()) == 0) {
				if (rename(TEMP_FILE_NAME.c_str(), scenarioFile.asChar()) == 0) {
					MGlobal::displayInfo("Finish Changing number of agents in file");
					return true;
				}
				else {
					MGlobal::displayInfo("rename temp fail");
				}
			}
			else {
				MGlobal::displayInfo("remove fail");
			}
		}
		return false;
	}
}

int CrowdCmd::getFileSize(MString scenarioFile) 
{
	std::ifstream infile(scenarioFile.asChar());
	int size = 0;
	std::string line;

	if (infile.is_open()) {
		while (!infile.eof())
		{
			getline(infile, line);
			if (!line.empty()) {
				size++;
			}
		}
		infile.close();
	}
	return size;
}

int CrowdCmd::readNumAgents(MString scenarioFile) {
	int num = 0;
	std::ifstream input(scenarioFile.asChar());
	if (input.fail())
	{
		MGlobal::displayInfo("error get number of existed agents");
		return 0;
	}
	try {
		double xMin, xMax, yMin, yMax;
		input >> xMin;
		input >> xMax;
		input >> yMin;
		input >> yMax;
		input >> num;
	}
	catch (std::exception &e) {
		MGlobal::displayInfo("error get number of existed agents");
		return 0;
	}
	input.close();
	return num;
}

bool CrowdCmd::is_file_exist(const char *fileName)
{
	std::ifstream infile(fileName);
	return infile.good();
}

MStatus CrowdCmd::doIt(const MArgList& args)
{
	int commandId = -1;
	int numAgents = 0;
	int frames = 120;
	double dt = 0.2;

	MString scenarioFile;
	MString parameterFile;

	MSyntax syntax = newSyntax();
	MArgDatabase argData(syntax, args);

	if (argData.isFlagSet(commandFlag)) {
		argData.getFlagArgument(commandFlag, 0, commandId);
	}
	std::string cmdstr = "command id " + std::to_string(commandId);
	MGlobal::displayInfo(cmdstr.c_str());

	if (argData.isFlagSet(scenarioFileFlag)) {
		argData.getFlagArgument(scenarioFileFlag, 0, scenarioFile);
		MGlobal::displayInfo(scenarioFile);
	}
	if (argData.isFlagSet(editFlag)) {
		argData.getFlagArgument(editFlag, 0, edit_count);
		std::string cmdstr = "edit count " + std::to_string(edit_count);
		MGlobal::displayInfo(cmdstr.c_str());
	}

	ImplicitSolver solver;
	solver.setdt(dt);

	switch (commandId)
	{
	case 0://read data from parameterFile and simulate
	{
		std::ifstream input(scenarioFile.asChar());
		int temp_obs = 0;
		int numObjects;

		if (input.fail())
		{
			MGlobal::displayInfo("error read scenario file");
			return MStatus::kFailure;
		}
		try {
			double xMin, xMax, yMin, yMax;
			
			input >> xMin;
			input >> xMax;
			input >> yMin;
			input >> yMax;
			solver.init(xMax - xMin, yMax - yMin);

			input >> numObjects;
			input >> frames;
			Vector2D position;
			Vector2D goal;
			Vector2D velocity = Vector2D(0, 0);
			solver.setMaxSteps(frames);

			double radius;
			double prefSpeed;
			double maxSpeed = 2.;
			double goalRadius = 1.;
			int gid, booleanAgent;

			std::string cmdst = "total objects read from file in the scene " + std::to_string(numObjects);
			MGlobal::displayInfo(cmdst.c_str());

			for (int i = 0; i < numObjects; i++)
			{
				input >> gid;
				input >> position.x();
				input >> position.y();
				input >> goal.x();
				input >> goal.y();
				input >> prefSpeed;
				input >> radius;
				input >> booleanAgent;
				
				if (booleanAgent == 1) {	
					solver.createAgent(position, goal, velocity, radius, prefSpeed, maxSpeed, goalRadius, gid, 1);
					std::string creater_agent = "polyCylinder -r 0.1 -sx 10 -sy 5 -sz 5 -h 1 -name Agent" + std::to_string(numAgents);
					string info = "create cylinder agent " + std::to_string(numAgents);
					MGlobal::displayInfo(info.c_str());
					MGlobal::executeCommand(creater_agent.c_str());
					std::string init_pos = "move -xz " + std::to_string(position.x()) + " " + std::to_string(position.y()) + " " + std::to_string(0) + " Agent" + std::to_string(numAgents);
					MGlobal::executeCommand(init_pos.c_str());
					numAgents += 1;
				}else{
					solver.createAgent(position, goal, velocity, radius, prefSpeed, maxSpeed, goalRadius, gid, 0);
					std::string creater_obst = "polyCube -sx 5 -sy 5 -sz 5 -h 0.5 -w 0.6 -d 0.6 -name obstacle" + std::to_string(temp_obs);
					string info = "create cube obstacle " + std::to_string(temp_obs);
					MGlobal::displayInfo(info.c_str());
					MGlobal::executeCommand(creater_obst.c_str());
					std::string init_pos = "move -xyz " + std::to_string(position.x()) + " -0.45 " + std::to_string(position.y()) + " "+ std::to_string(0) + " obstacle" + std::to_string(temp_obs);
					MGlobal::executeCommand(init_pos.c_str());
					temp_obs += 1;
				}
			}
		}
		catch (std::exception &e) {
			MGlobal::displayInfo("Error reading the scenario file ");
			return MStatus::kFailure;
		}
		input.close();
		MGlobal::displayInfo("finish reading scenerio file");
		std::string cmdstr1 = "total number of agents " + std::to_string(numAgents) +" total objects in the scene "+std::to_string(numObjects);
		MGlobal::displayInfo(cmdstr1.c_str());

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
		const vector<Agent*>& obstacles = solver.getObstacles();
		numAgents = agents.size();
		std::string cmdst1 = "total number of agents " + std::to_string(numAgents) + " total objects in the scene " + std::to_string(numObjects);
		MGlobal::displayInfo(cmdst1.c_str());
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
			}
		}
		MGlobal::displayInfo("finish animate agents");

		for (unsigned int i = 0; i < (numObjects - numAgents); ++i)
		{
			const Agent* obst = obstacles[i];
			double time = 0.;
			int characterId;

			vector<Vector2D> path = obst->path();
			vector<Vector2D>::iterator it_start = path.begin();
			vector<Vector2D>::iterator it_end = path.end();
			vector<Vector2D> orient = obst->orientations();
			vector<Vector2D>::iterator it_or = orient.begin();

			int index = 0;
			float last_pos[] = { (float)it_start->x(), (float)it_start->y(), 0 };
			float last_ori[] = { 0, 0, float(atan2((*orient.begin()).y(), (*orient.begin()).x()) + M_PI) };
			float t_x, t_y, t_z, r_x, r_y, r_z;

			for (vector<Vector2D>::iterator it = it_start; it != it_end; ++it, time += dt, ++it_or)
			{
				float pos[] = { (float)it->x(), (float)it->y(), 0 };
				float orientation[] = { 0, 0, float(atan2((*it_or).y(), (*it_or).x()) + M_PI) };

				std::string set_tx = "setAttr obstacle" + std::to_string(i) + ".translateX " + std::to_string(pos[0]);
				std::string set_ty = "setAttr obstacle" + std::to_string(i) + ".translateY " + std::to_string(pos[2]);
				std::string set_tz = "setAttr obstacle" + std::to_string(i) + ".translateZ " + std::to_string(pos[1]);

				std::string set_rx = "setAttr obstacle" + std::to_string(i) + ".rotateX " + std::to_string(orientation[0]);
				std::string set_ry = "setAttr obstacle" + std::to_string(i) + ".rotateY " + std::to_string(orientation[2]);
				std::string set_rz = "setAttr obstacle" + std::to_string(i) + ".rotateZ " + std::to_string(orientation[1]);

				MGlobal::executeCommand(set_tx.c_str());
				MGlobal::executeCommand(set_ty.c_str());
				MGlobal::executeCommand(set_tz.c_str());

				MGlobal::executeCommand(set_rx.c_str());
				MGlobal::executeCommand(set_ry.c_str());
				MGlobal::executeCommand(set_rz.c_str());

				std::string animate_tx = "setKeyframe -attribute translateX -t " + std::to_string(time - dt) + " -t " + std::to_string(time)
					+ " obstacle" + std::to_string(i);
				std::string animate_ty = "setKeyframe -attribute translateY -t " + std::to_string(time - dt) + " -t " + std::to_string(time)
					+ " obstacle" + std::to_string(i);
				std::string animate_tz = "setKeyframe -attribute translateZ -t " + std::to_string(time - dt) + " -t " + std::to_string(time)
					+ " obstacle" + std::to_string(i);

				MGlobal::executeCommand(animate_tx.c_str());
				MGlobal::executeCommand(animate_ty.c_str());
				MGlobal::executeCommand(animate_tz.c_str());
				index++;
			}
		}

		MGlobal::displayInfo("finish animate obstacles");
		break;
	}

	//==========================================================================================================================
	case 1://add agent
	{
		Vector2D position;
		Vector2D goal;
		Vector2D velocity = Vector2D(0, 0);
		double radius;
		double prefSpeed;
		double goalRadius = 1.;
		int gid;
		double maxSpeed = 2.;
		
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
			argData.getFlagArgument(goalYFlag, 0, goal.y());
		}
		if (argData.isFlagSet(prefSpeedFlag)) {
			argData.getFlagArgument(prefSpeedFlag, 0, prefSpeed);
		}
		if (argData.isFlagSet(radiusFlag)) {
			argData.getFlagArgument(radiusFlag, 0, radius);
		}
		

		if (editAgentNumbersInFile(scenarioFile)){
			std::string creater_agent = "polyCylinder -r 0.1 -sx 10 -sy 5 -sz 5 -h 1 -name addAgent" + std::to_string(edit_count);
			string info = "add agent cylinder name addAgent" + to_string(edit_count);
			MGlobal::displayInfo(info.c_str());
			MGlobal::displayInfo("add agent cylinder");
			MGlobal::executeCommand(creater_agent.c_str());
			std::string init_pos = "move -xz " + std::to_string(position.x()) + " " + std::to_string(position.y()) + " " + std::to_string(0) + " addAgent" + std::to_string(edit_count);
			MGlobal::executeCommand(init_pos.c_str());

			std::ofstream fout(scenarioFile.asChar(), std::ios::app);
			std::string line = to_string(gid) + " " + to_string(position.x()) + " " + to_string(position.y()) + " " + to_string(goal.x()) + " " + to_string(goal.y())
				+ " " + to_string(prefSpeed) + " " + to_string(radius) + " 1";
			fout << line;
			fout.close();
			MGlobal::displayInfo("Finish Appending");
		}


		break;
	}

	//==========================================================================================================================
	case 2: //add obstacle
	{
		Vector2D position;
		Vector2D goal;
		Vector2D velocity = Vector2D(0, 0);
		double radius;
		double prefSpeed;
		double goalRadius = 1.;
		int gid;
		double maxSpeed = 3.;

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
			argData.getFlagArgument(goalYFlag, 0, goal.y());
		}
		if (argData.isFlagSet(prefSpeedFlag)) {
			argData.getFlagArgument(prefSpeedFlag, 0, prefSpeed);
		}
		if (argData.isFlagSet(radiusFlag)) {
			argData.getFlagArgument(radiusFlag, 0, radius);
		}
		

		if (editAgentNumbersInFile(scenarioFile)) {
			std::string creater_obst = "polyCube -sx 5 -sy 5 -sz 5 -h 0.5 -w 0.6 -d 0.6 -name addObst" + std::to_string(edit_count);
			string info = "add obstacle cube name addObst" + to_string(edit_count);
			MGlobal::displayInfo(info.c_str());
			MGlobal::executeCommand(creater_obst.c_str());
			std::string init_pos = "move -xz " + std::to_string(position.x()) + " " + std::to_string(position.y()) + " "
				+ std::to_string(0) + " addObst" + std::to_string(edit_count);
			MGlobal::executeCommand(init_pos.c_str());

			std::ofstream fout(scenarioFile.asChar(), std::ios::app);
			std::string line = to_string(gid) + " " + to_string(position.x()) + " " + to_string(position.y()) + " " + to_string(goal.x()) + " " + to_string(goal.y())
				+ " " + to_string(prefSpeed) + " " + to_string(0.6) + " 0";
			fout << line;
			MGlobal::displayInfo("Finish Appending");
			fout.close();
		}

		break;
	}

	case 3: // temperally render initial position of agents in the scenario file
	{
		std::ifstream input(scenarioFile.asChar());
		if (input.fail())
		{
			MGlobal::displayInfo("error read scenario file");
			return MStatus::kFailure;
		}
		try {
			double xMin, xMax, yMin, yMax, radius, prefSpeed;
			input >> xMin;
			input >> xMax;
			input >> yMin;
			input >> yMax;
			input >> numAgents;
			input >> frames;
			Vector2D position;
			Vector2D goal;
			int gid;
			int booleanAgent;

			for (int i = 0; i < numAgents; ++i)
			{
				input >> gid;
				input >> position.x();
				input >> position.y();
				input >> goal.x();
				input >> goal.y();
				input >> prefSpeed;
				input >> radius;
				input >> booleanAgent;

				if (booleanAgent == 1) {
					std::string creater_agent = "polyCylinder -r 0.1 -sx 10 -sy 5 -sz 5 -h 1 -name A" + std::to_string(i);
					string info = "create cylinder A" + std::to_string(i);
					MGlobal::displayInfo(info.c_str());
					MGlobal::executeCommand(creater_agent.c_str());
					std::string init_pos = "move -xz " + std::to_string(position.x()) + " " + std::to_string(position.y()) + " " + std::to_string(0) + " A" + std::to_string(i);
					MGlobal::executeCommand(init_pos.c_str());
				}else {
					std::string creater_obst = "polyCube -sx 5 -sy 5 -sz 5 -h 0.5 -w 0.6 -d 0.6 -name O" + std::to_string(i);
					string info = "create cube O" + std::to_string(i);
					MGlobal::displayInfo(info.c_str());
					MGlobal::executeCommand(creater_obst.c_str());
					std::string init_pos = "move -xz " + std::to_string(position.x()) + " " + std::to_string(position.y()) + " " + std::to_string(0) + " O" + std::to_string(i);
					MGlobal::executeCommand(init_pos.c_str());
				}
			}
		}
		catch (std::exception &e) {
			MGlobal::displayInfo("Error reading the scenario file ");
			return MStatus::kFailure;
		}
		input.close();
		MGlobal::displayInfo("finish reading scenerio file");
		break;
	}

	case 4: //backup original scenario file
	{
		if (is_file_exist(ORIGINAL_FILE_NAME.c_str())) {
			remove(scenarioFile.asChar());
			rename(ORIGINAL_FILE_NAME.c_str(), scenarioFile.asChar());
		}
		break;
	}

	case 5: // undo the last add on scenario file
	{
		if (edit_count >= 1){

			MGlobal::displayInfo("undo the last add on scenario file");
			int remove_line = getFileSize(scenarioFile);
			int temp_line_count = 0;
			std::string line;

			std::ifstream infile1(scenarioFile.asChar());
			ofstream outfile(TEMP_FILE_NAME);
			if (infile1.is_open()) {
				while (!infile1.eof()) {
					temp_line_count++;
					getline(infile1, line);
					//string d1 = "read line # "+to_string(temp_line_count)+"need tp remove "+to_string(remove_line);
					//MGlobal::displayInfo(d1.c_str());
					if (temp_line_count == 3) {
						int read_agent = stoi(line) - 1;
						outfile << read_agent << endl;
						string display = "decrement agent numbers to " + to_string(read_agent);
						MGlobal::displayInfo(display.c_str());
					}else {
						if (temp_line_count == remove_line) {
							std::istringstream iss(line);
							std::vector<std::string> results((std::istream_iterator<std::string>(iss)), std::istream_iterator<std::string>());
							int if_agent = stoi(results[results.size()-1]);
							if (if_agent == 1) {
								std::string remove_cmd = "delete addAgent" + std::to_string(edit_count);
								MGlobal::executeCommand(remove_cmd.c_str());
								MGlobal::displayInfo(remove_cmd.c_str());
							}
							else {
								std::string remove_cmd = "delete addObst" + std::to_string(edit_count);
								MGlobal::executeCommand(remove_cmd.c_str());
								MGlobal::displayInfo(remove_cmd.c_str());
							}
							
						}else {
							outfile << line << endl;
						}
					}
				}
				infile1.close();
				outfile.close();
				if (remove(scenarioFile.asChar()) == 0) {
					if (rename(TEMP_FILE_NAME.c_str(), scenarioFile.asChar()) == 0) {
						MGlobal::displayInfo("finish undo the file modification");
					}
				}
			}
		}
		
		break;
	}

	case 6: // close the gui and delete original
	{
		if (is_file_exist(ORIGINAL_FILE_NAME.c_str())) {
			remove(ORIGINAL_FILE_NAME.c_str());
		}
		if (is_file_exist(TEMP_FILE_NAME.c_str())) {
			remove(TEMP_FILE_NAME.c_str());
		}
		break;
	}
	default:
		break;
	}

	return MStatus::kSuccess;
}

