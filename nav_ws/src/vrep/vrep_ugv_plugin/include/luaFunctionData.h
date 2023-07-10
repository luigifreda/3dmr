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

#include "luaFunctionDataItem.h"
#include "v_repLib.h"

#define SIM_LUA_ARG_NIL_ALLOWED (65536)

class CLuaFunctionData  
{
public:
	CLuaFunctionData();
	virtual ~CLuaFunctionData();

	bool readDataFromLua(const SLuaCallBack* p,const int* expectedArguments,int requiredArgumentCount,const char* functionName);
	std::vector<CLuaFunctionDataItem>* getInDataPtr();

	void pushOutData(const CLuaFunctionDataItem& dataItem);
	void writeDataToLua(SLuaCallBack* p);

	static void getInputDataForFunctionRegistration(const int* dat,std::vector<int>& outDat);


protected:
	std::vector<CLuaFunctionDataItem> _inData;
	std::vector<CLuaFunctionDataItem> _outData;
};
