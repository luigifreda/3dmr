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

#include "luaFunctionData.h"
#include <sstream>
#include <cstring>

CLuaFunctionData::CLuaFunctionData()
{
}

CLuaFunctionData::~CLuaFunctionData()
{

}

void CLuaFunctionData::getInputDataForFunctionRegistration(const int* dat,std::vector<int>& outDat)
{
	outDat.clear();
	outDat.push_back(dat[0]);
	for (int i=0;i<dat[0];i++)
		outDat.push_back((dat[1+2*i+0]|SIM_LUA_ARG_NIL_ALLOWED)-SIM_LUA_ARG_NIL_ALLOWED);
}

std::vector<CLuaFunctionDataItem>* CLuaFunctionData::getInDataPtr()
{
	return(&_inData);
}

bool CLuaFunctionData::readDataFromLua(const SLuaCallBack* p,const int* expectedArguments,int requiredArgumentCount,const char* functionName)
{
	_inData.clear();
	int argCnt=p->inputArgCount;
	if (argCnt<requiredArgumentCount)
	{
		simSetLastError(functionName,"Not enough arguments.");
		return(false);
	}

	int boolArgInd=0;
	int intArgInd=0;
	int floatArgInd=0;
    int doubleArgInd=0;
    int charArgInd=0;
	int charBuffArgInd=0;

	for (int i=0;i<argCnt;i++)
	{
		if (i>=expectedArguments[0])
			break;
		bool done=false;
		if (p->inputArgTypeAndSize[i*2+0]==sim_lua_arg_nil)
		{
			// is nil explicitely allowed?
			if (expectedArguments[1+i*2+0]&SIM_LUA_ARG_NIL_ALLOWED)
			{ // yes. This is for an argument that can optionally also be nil.
				CLuaFunctionDataItem dat;
				_inData.push_back(dat);
				done=true;
			}
			else
			{ // no
				if (int(_inData.size())<requiredArgumentCount)
				{
					std::ostringstream str;
					str << "Argument " << i+1 << " is not correct.";
					simSetLastError(functionName,str.str().c_str());
					return(false);
				}
				break; // this argument is nil, so it is like inexistant. But we also won't explore any more arguments, we have enough.
			}
		}
		if (!done)
		{
			if (p->inputArgTypeAndSize[i*2+0]!=((expectedArguments[1+i*2+0]|SIM_LUA_ARG_NIL_ALLOWED)-SIM_LUA_ARG_NIL_ALLOWED))
			{
				std::ostringstream str;
				str << "Argument " << i+1 << " is not correct.";
				simSetLastError(functionName,str.str().c_str());
				return(false);
			}
			if (p->inputArgTypeAndSize[i*2+0]&sim_lua_arg_table)
			{ // we have a table
				if ( (p->inputArgTypeAndSize[i*2+1]<expectedArguments[1+i*2+1])&&(expectedArguments[1+i*2+1]!=0) )
				{
					std::ostringstream str;
					str << "Argument " << i+1 << " is not correct (wrong table size).";
					simSetLastError(functionName,str.str().c_str());
					return(false);
				}
				else
				{
					int t=p->inputArgTypeAndSize[i*2+0]-sim_lua_arg_table;
					int itemCnt=p->inputArgTypeAndSize[i*2+1];

					if (t==sim_lua_arg_nil)
					{
						CLuaFunctionDataItem* a=new CLuaFunctionDataItem();
						a->setNilTable(itemCnt);
						CLuaFunctionDataItem dat;
						dat.setNilTable(itemCnt);
						_inData.push_back(dat);
					}
					if (t==sim_lua_arg_bool)
					{
						std::vector<bool> vect;
						for (int j=0;j<itemCnt;j++)
							vect.push_back(p->inputBool[boolArgInd++]!=0);
						CLuaFunctionDataItem dat(vect);
						_inData.push_back(dat);
					}
					if (t==sim_lua_arg_int)
					{
						std::vector<int> vect;
						for (int j=0;j<itemCnt;j++)
							vect.push_back(p->inputInt[intArgInd++]);
						CLuaFunctionDataItem dat(vect);
						_inData.push_back(dat);
					}
					if (t==sim_lua_arg_float)
					{
						std::vector<float> vect;
						for (int j=0;j<itemCnt;j++)
							vect.push_back(p->inputFloat[floatArgInd++]);
						CLuaFunctionDataItem dat(vect);
						_inData.push_back(dat);
					}
					if (t==sim_lua_arg_double)
					{
						std::vector<double> vect;
						for (int j=0;j<itemCnt;j++)
							vect.push_back(p->inputDouble[doubleArgInd++]);
						CLuaFunctionDataItem dat(vect);
						_inData.push_back(dat);
					}
					if (t==sim_lua_arg_string)
					{
						std::vector<std::string> vect;
						for (int j=0;j<itemCnt;j++)
						{
							std::string str(p->inputChar+charArgInd);
							vect.push_back(str);
							charArgInd+=int(strlen(p->inputChar+charArgInd))+1;
						}
						CLuaFunctionDataItem dat(vect);
						_inData.push_back(dat);
					}
					if (t==sim_lua_arg_charbuff)
					{
						std::ostringstream str;
						str << "Argument " << i+1 << " cannot be a table.";
						simSetLastError(functionName,str.str().c_str());
						return(false);
					}
				}
			}
			else
			{ // we do not have a table
				int t=p->inputArgTypeAndSize[i*2+0];
				if (t==sim_lua_arg_nil)
				{
					CLuaFunctionDataItem dat;
					_inData.push_back(dat);
				}
				if (t==sim_lua_arg_bool)
				{
					CLuaFunctionDataItem dat(p->inputBool[boolArgInd++]!=0);
					_inData.push_back(dat);
				}
				if (t==sim_lua_arg_int)
				{
					CLuaFunctionDataItem dat(p->inputInt[intArgInd++]);
					_inData.push_back(dat);
				}
				if (t==sim_lua_arg_float)
				{
					CLuaFunctionDataItem dat(p->inputFloat[floatArgInd++]);
					_inData.push_back(dat);
				}
				if (t==sim_lua_arg_double)
				{
					CLuaFunctionDataItem dat(p->inputDouble[doubleArgInd++]);
					_inData.push_back(dat);
				}
				if (t==sim_lua_arg_string)
				{
					CLuaFunctionDataItem dat(std::string(p->inputChar+charArgInd));
					charArgInd+=int(strlen(p->inputChar+charArgInd))+1;
					_inData.push_back(dat);
				}
				if (t==sim_lua_arg_charbuff)
				{
					if ( (p->inputArgTypeAndSize[i*2+1]<expectedArguments[1+i*2+1])&&(expectedArguments[1+i*2+1]!=0) )
					{
						std::ostringstream str;
						str << "Argument " << i+1 << " is not correct (wrong buffer size).";
						simSetLastError(functionName,str.str().c_str());
						return(false);
					}
					else
					{
						CLuaFunctionDataItem dat(p->inputCharBuff+charBuffArgInd,p->inputArgTypeAndSize[i*2+1]);
						charBuffArgInd+=p->inputArgTypeAndSize[i*2+1];
						_inData.push_back(dat);
					}
				}
			}
		}
	}
	return(true);
}

void CLuaFunctionData::pushOutData(const CLuaFunctionDataItem& dataItem)
{
	_outData.push_back(dataItem);
}

void CLuaFunctionData::writeDataToLua(SLuaCallBack* p)
{
	p->outputArgCount=0;
	int itemCnt=int(_outData.size());
	if (itemCnt>0)
	{
		p->outputArgCount=itemCnt;
		p->outputArgTypeAndSize=(simInt*)simCreateBuffer(p->outputArgCount*2*sizeof(simInt));

		int boolDataCnt=0;
		int intDataCnt=0;
		int floatDataCnt=0;
        int doubleDataCnt=0;
        int charDataCnt=0;
		int charBuffDataCnt=0;

		for (int i=0;i<itemCnt;i++)
		{
			if (_outData[i].isTable())
			{ // table
				if (_outData[i].getType()==-1)
				{
					p->outputArgTypeAndSize[i*2+0]=sim_lua_arg_nil|sim_lua_arg_table;
					p->outputArgTypeAndSize[i*2+1]=_outData[i].getNilTableSize();
				}
				if (_outData[i].getType()==0)
				{
					p->outputArgTypeAndSize[i*2+0]=sim_lua_arg_bool|sim_lua_arg_table;
					p->outputArgTypeAndSize[i*2+1]=int(_outData[i].boolData.size());
					boolDataCnt+=p->outputArgTypeAndSize[i*2+1];
				}
				if (_outData[i].getType()==1)
				{
					p->outputArgTypeAndSize[i*2+0]=sim_lua_arg_int|sim_lua_arg_table;
					p->outputArgTypeAndSize[i*2+1]=int(_outData[i].intData.size());
					intDataCnt+=p->outputArgTypeAndSize[i*2+1];
				}
				if (_outData[i].getType()==2)
				{
					p->outputArgTypeAndSize[i*2+0]=sim_lua_arg_float|sim_lua_arg_table;
					p->outputArgTypeAndSize[i*2+1]=int(_outData[i].floatData.size());
					floatDataCnt+=p->outputArgTypeAndSize[i*2+1];
				}
                if (_outData[i].getType()==5)
                {
                    p->outputArgTypeAndSize[i*2+0]=sim_lua_arg_double|sim_lua_arg_table;
                    p->outputArgTypeAndSize[i*2+1]=int(_outData[i].doubleData.size());
                    doubleDataCnt+=p->outputArgTypeAndSize[i*2+1];
                }
                if (_outData[i].getType()==3)
				{
					p->outputArgTypeAndSize[i*2+0]=sim_lua_arg_string|sim_lua_arg_table;
					p->outputArgTypeAndSize[i*2+1]=int(_outData[i].stringData.size());
					for (int j=0;j<int(_outData[i].stringData.size());j++)
						charDataCnt+=int(_outData[i].stringData[j].length())+1;
				}
			}
			else
			{
				if (_outData[i].getType()==-1)
				{
					p->outputArgTypeAndSize[i*2+1]=0;
					p->outputArgTypeAndSize[i*2+0]=sim_lua_arg_nil;
				}
				if (_outData[i].getType()==0)
				{
					p->outputArgTypeAndSize[i*2+1]=0;
					p->outputArgTypeAndSize[i*2+0]=sim_lua_arg_bool;
					boolDataCnt++;
				}
				if (_outData[i].getType()==1)
				{
					p->outputArgTypeAndSize[i*2+1]=0;
					p->outputArgTypeAndSize[i*2+0]=sim_lua_arg_int;
					intDataCnt++;
				}
				if (_outData[i].getType()==2)
				{
					p->outputArgTypeAndSize[i*2+1]=0;
					p->outputArgTypeAndSize[i*2+0]=sim_lua_arg_float;
					floatDataCnt++;
				}
                if (_outData[i].getType()==5)
                {
                    p->outputArgTypeAndSize[i*2+1]=0;
                    p->outputArgTypeAndSize[i*2+0]=sim_lua_arg_double;
                    doubleDataCnt++;
                }
                if (_outData[i].getType()==3)
				{
					p->outputArgTypeAndSize[i*2+1]=0;
					p->outputArgTypeAndSize[i*2+0]=sim_lua_arg_string;
					charDataCnt+=int(_outData[i].stringData[0].length())+1;
				}
				if (_outData[i].getType()==4)
				{
					p->outputArgTypeAndSize[i*2+1]=int(_outData[i].stringData[0].length());
					p->outputArgTypeAndSize[i*2+0]=sim_lua_arg_charbuff;
					charBuffDataCnt+=int(_outData[i].stringData[0].length());
				}
			}
		}

		// Now create the buffers:
		p->outputBool=(simBool*)simCreateBuffer(boolDataCnt*sizeof(simBool));
		p->outputInt=(simInt*)simCreateBuffer(intDataCnt*sizeof(simInt));
		p->outputFloat=(simFloat*)simCreateBuffer(floatDataCnt*sizeof(simFloat));
        p->outputDouble=(simDouble*)simCreateBuffer(doubleDataCnt*sizeof(simDouble));
        p->outputChar=(simChar*)simCreateBuffer(charDataCnt*sizeof(simChar));
		p->outputCharBuff=(simChar*)simCreateBuffer(charBuffDataCnt*sizeof(simChar));
		
		// Now populate the buffers:
		int boolDataOff=0;
		int intDataOff=0;
		int floatDataOff=0;
        int doubleDataOff=0;
        int charDataOff=0;
		int charBuffDataOff=0;

		for (int i=0;i<itemCnt;i++)
		{
			if (_outData[i].isTable())
			{ // table
				if (_outData[i].getType()==0)
				{
					for (int j=0;j<int(_outData[i].boolData.size());j++)
						p->outputBool[boolDataOff++]=_outData[i].boolData[j];
				}
				if (_outData[i].getType()==1)
				{
					for (int j=0;j<int(_outData[i].intData.size());j++)
						p->outputInt[intDataOff++]=_outData[i].intData[j];
				}
				if (_outData[i].getType()==2)
				{
					for (int j=0;j<int(_outData[i].floatData.size());j++)
						p->outputFloat[floatDataOff++]=_outData[i].floatData[j];
				}
                if (_outData[i].getType()==5)
                {
                    for (int j=0;j<int(_outData[i].doubleData.size());j++)
                        p->outputDouble[doubleDataOff++]=_outData[i].doubleData[j];
                }
                if (_outData[i].getType()==3)
				{
					for (int j=0;j<int(_outData[i].stringData.size());j++)
					{
						for (int k=0;k<int(_outData[i].stringData[j].length());k++)
							p->outputChar[charDataOff++]=_outData[i].stringData[j][k];
						p->outputChar[charDataOff++]=0;
					}
				}
			}
			else
			{
				if (_outData[i].getType()==0)
					p->outputBool[boolDataOff++]=_outData[i].boolData[0];
				if (_outData[i].getType()==1)
					p->outputInt[intDataOff++]=_outData[i].intData[0];
				if (_outData[i].getType()==2)
					p->outputFloat[floatDataOff++]=_outData[i].floatData[0];
                if (_outData[i].getType()==5)
                    p->outputDouble[doubleDataOff++]=_outData[i].doubleData[0];
                if (_outData[i].getType()==3)
				{
					for (int j=0;j<int(_outData[i].stringData[0].length());j++)
						p->outputChar[charDataOff++]=_outData[i].stringData[0][j];
					p->outputChar[charDataOff++]=0;
				}
				if (_outData[i].getType()==4)
				{
					for (int j=0;j<int(_outData[i].stringData[0].length());j++)
						p->outputCharBuff[charBuffDataOff++]=_outData[i].stringData[0][j];
				}
			}
		}
	}
}
