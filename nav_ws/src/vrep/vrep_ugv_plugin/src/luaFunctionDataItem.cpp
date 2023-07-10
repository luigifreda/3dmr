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

#include "luaFunctionDataItem.h"

CLuaFunctionDataItem::CLuaFunctionDataItem()
{
	_nilTableSize=0;
	_isTable=false;
	_type=-1; // nil
}

CLuaFunctionDataItem::CLuaFunctionDataItem(bool v)
{
	_nilTableSize=0;
	_isTable=false;
	_type=0;
	boolData.push_back(v);
}

CLuaFunctionDataItem::CLuaFunctionDataItem(int v)
{
	_nilTableSize=0;
	_isTable=false;
	_type=1;
	intData.push_back(v);
}

CLuaFunctionDataItem::CLuaFunctionDataItem(float v)
{
	_nilTableSize=0;
	_isTable=false;
	_type=2;
	floatData.push_back(v);
}

CLuaFunctionDataItem::CLuaFunctionDataItem(double v)
{
    _nilTableSize=0;
    _isTable=false;
    _type=5;
    doubleData.push_back(v);
}

CLuaFunctionDataItem::CLuaFunctionDataItem(const std::string& v)
{
	_nilTableSize=0;
	_isTable=false;
	_type=3;
	stringData.push_back(v);
}

CLuaFunctionDataItem::CLuaFunctionDataItem(const char* bufferPtr,unsigned int bufferLength)
{
	_nilTableSize=0;
	_isTable=false;
	_type=4;
	std::string v(bufferPtr,bufferLength);
	stringData.push_back(v);
}

CLuaFunctionDataItem::CLuaFunctionDataItem(const std::vector<bool>& v)
{
	_nilTableSize=0;
	_isTable=true;
	_type=0;
	boolData.assign(v.begin(),v.end());
}

CLuaFunctionDataItem::CLuaFunctionDataItem(const std::vector<int>& v)
{
	_nilTableSize=0;
	_isTable=true;
	_type=1;
	intData.assign(v.begin(),v.end());
}

CLuaFunctionDataItem::CLuaFunctionDataItem(const std::vector<float>& v)
{
	_nilTableSize=0;
	_isTable=true;
	_type=2;
	floatData.assign(v.begin(),v.end());
}

CLuaFunctionDataItem::CLuaFunctionDataItem(const std::vector<double>& v)
{
    _nilTableSize=0;
    _isTable=true;
    _type=5;
    doubleData.assign(v.begin(),v.end());
}

CLuaFunctionDataItem::CLuaFunctionDataItem(const std::vector<std::string>& v)
{
	_nilTableSize=0;
	_isTable=true;
	_type=3;
	stringData.assign(v.begin(),v.end());
}

CLuaFunctionDataItem::~CLuaFunctionDataItem()
{
}

bool CLuaFunctionDataItem::isTable()
{
	return(_isTable);
}

int CLuaFunctionDataItem::getType()
{
	return(_type);
}

void CLuaFunctionDataItem::setNilTable(int size)
{
	if (_type==-1)
	{
		_isTable=true;
		_nilTableSize=size;
	}
}

int CLuaFunctionDataItem::getNilTableSize()
{
	return(_nilTableSize);
}
