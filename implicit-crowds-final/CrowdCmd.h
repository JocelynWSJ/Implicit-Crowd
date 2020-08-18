#pragma once
#include <maya/MPxCommand.h>
#include <string>
#include "ImplicitSolver.h"

class  CrowdCmd : public MPxCommand
{
public:
	CrowdCmd();
	virtual ~CrowdCmd();
	static MSyntax newSyntax();
	
	static void* creator() { return new CrowdCmd(); }
	MStatus doIt(const MArgList& args);
	void getNext(std::string& value);
	bool editAgentNumbersInFile(MString scenarioFile);
	int getFileSize(MString scenarioFile);
	int readNumAgents(MString scenarioFile);
	bool is_file_exist(const char *fileName);

protected:
	int edit_count;
};



