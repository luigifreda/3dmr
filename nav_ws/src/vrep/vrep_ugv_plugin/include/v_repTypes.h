// This file is part of V-REP, the Virtual Robot Experimentation Platform.
// 
// Copyright 2006-2015 Coppelia Robotics GmbH. All rights reserved. 
// marc@coppeliarobotics.com
// www.coppeliarobotics.com
// 
// V-REP is dual-licensed, under the terms of EITHER (at your option):
//   1. V-REP commercial license (contact us for details)
//   2. GNU GPL (see below)
// 
// GNU GPL license:
// -------------------------------------------------------------------
// V-REP is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// V-REP IS DISTRIBUTED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
// WARRANTY. THE USER WILL USE IT AT HIS/HER OWN RISK. THE ORIGINAL
// AUTHORS AND COPPELIA ROBOTICS GMBH WILL NOT BE LIABLE FOR DATA LOSS,
// DAMAGES, LOSS OF PROFITS OR ANY OTHER KIND OF LOSS WHILE USING OR
// MISUSING THIS SOFTWARE.
// 
// See the GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with V-REP.  If not, see <http://www.gnu.org/licenses/>.
// -------------------------------------------------------------------
//
// This file was automatically created for V-REP release V3.2.2 Rev1 on September 5th 2015

#if !defined(V_REPTYPES_INCLUDED_)
#define V_REPTYPES_INCLUDED_

// Various types used in the interface functions:
typedef unsigned char simBool;
typedef char simChar;
typedef int simInt;
typedef float simFloat;
typedef double simDouble;
typedef void simVoid;
typedef unsigned char simUChar;
typedef unsigned int simUInt;

struct SLuaCallBack
{
	simInt objectID;
	simBool* inputBool;
	simInt* inputInt;
	simFloat* inputFloat;
	simChar* inputChar;
	simInt inputArgCount;
	simInt* inputArgTypeAndSize;
	simBool* outputBool;
	simInt* outputInt;
	simFloat* outputFloat;
	simChar* outputChar;
	simInt outputArgCount;
	simInt* outputArgTypeAndSize;
	simChar waitUntilZero;
	simChar* inputCharBuff;
	simChar* outputCharBuff;
	simInt scriptID;
	simDouble* inputDouble;
	simDouble* outputDouble;
};

typedef int (*contactCallback)(int,int,int,int*,float*);
typedef int (*jointCtrlCallback)(int,int,int,const int*,const float*,float*);

#endif // !defined(V_REPTYPES_INCLUDED_)
