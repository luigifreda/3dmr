// Copyright 2006-2015 Coppelia Robotics GmbH. All rights reserved. 
// marc@coppeliarobotics.com
// www.coppeliarobotics.com
// 
// -------------------------------------------------------------------
// THIS FILE IS DISTRIBUTED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
// WARRANTY. THE USER WILL USE IT AT HIS/HER OWN RISK. THE ORIGINAL
// AUTHORS AND COPPELIA ROBOTICS GMBH WILL NOT BE LIABLE FOR DATA LOSS,
// DAMAGES, LOSS OF PROFITS OR ANY OTHER KIND OF LOSS WHILE USING OR
// MISUSING THIS SOFTWARE.
// 
// You are free to use/modify/distribute this file for whatever purpose!
// -------------------------------------------------------------------
//
// This file was automatically created for V-REP release V3.2.2 Rev1 on September 5th 2015

#pragma once

#include <vector>
#include <string>

class CLuaFunctionDataItem
{
public:
	CLuaFunctionDataItem();
	CLuaFunctionDataItem(bool v);
	CLuaFunctionDataItem(int v);
	CLuaFunctionDataItem(float v);
    CLuaFunctionDataItem(double v);
    CLuaFunctionDataItem(const std::string& v);
    CLuaFunctionDataItem(const char* bufferPtr,unsigned int bufferLength);

	CLuaFunctionDataItem(const std::vector<bool>& v);
	CLuaFunctionDataItem(const std::vector<int>& v);
	CLuaFunctionDataItem(const std::vector<float>& v);
    CLuaFunctionDataItem(const std::vector<double>& v);
    CLuaFunctionDataItem(const std::vector<std::string>& v);

	virtual ~CLuaFunctionDataItem();

	bool isTable();
	int getType();
	void setNilTable(int size);
	int getNilTableSize();

	std::vector<bool> boolData;
	std::vector<int> intData;
	std::vector<float> floatData;
    std::vector<double> doubleData;
    std::vector<std::string> stringData;

protected:
	int _nilTableSize;
	bool _isTable;
    int _type; // -1=nil,0=bool,1=int,2=float,3=string,4=buffer,5=double
};
