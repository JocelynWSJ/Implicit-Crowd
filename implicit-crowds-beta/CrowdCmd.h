#pragma once
#include <maya/MPxCommand.h>
#include <string>

class  CrowdCmd : public MPxCommand
{
public:
	CrowdCmd();
	virtual ~CrowdCmd();
	static MSyntax newSyntax();
	
	static void* creator() { return new CrowdCmd(); }
	MStatus doIt(const MArgList& args);
	void getNext(std::string& value);
};



