/**
* This file is part of the ROS package path_planner which belongs to the framework 3DMR. 
*
* Copyright (C) 2016-present Luigi Freda <luigifreda at gmail dot com>
* For more information see <https://github.com/luigifreda/3dmr>
*
* 3DMR is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* 3DMR is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with 3DMR. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef COLOR_UTILS_H_
#define COLOR_UTILS_H_

///\class Color 
///\brief Basic class for openGL color management in Qt
///\author Luigi Freda
class Color
{
	/// \brief Default constructor
	public: Color(): r(0), g(0), b(0), a(1) {}
	
	/// \brief Constructor
	public: Color(float _r, float _g, float _b, float _a = 1.) : r(_r), g(_g), b(_b), a(_a) {}
	
	/// Red color information
	public: float r;
	
	/// Green color information
	public: float g;
	
	/// Blue color information
	public: float b;
	
	/// Alpha color information
	public: float a;
};


///\namespace Colors
///\brief Some predefined colors.
///\author Luigi Freda
namespace Colors
{

	static const float NONE		= 0.00;
	static const float DARK		= 0.33;
	static const float MEDIUM	= 0.66;
	static const float LIGHT	= 0.99;
        
        enum ColorType {kYellow=0, kBlue, kGreen, kRed, kCyan, kWhite, kOrange, kMagenta, kBlack, kNumColors};

        /// < colors definitions 
	inline const Color Black()		{return Color(NONE,NONE,NONE);}
	inline const Color White()		{return Color(1.,1.,1.);}
	/*grey*/
	inline const Color DarkGrey()		{return Color(DARK,DARK,DARK);}
	inline const Color LightGrey()	{return Color(MEDIUM,MEDIUM,MEDIUM);}
	/*red*/
	inline const Color LightRed()		{return Color(LIGHT,NONE,NONE);}
	inline const Color Red()		{return Color(MEDIUM,NONE,NONE);}
	inline const Color DarkRed()		{return Color(DARK,NONE,NONE);}
	/*green*/
	inline const Color LightGreen()	{return Color(NONE,LIGHT,NONE);}
	inline const Color Green()		{return Color(NONE,MEDIUM,NONE);}
	inline const Color DarkGreen()	{return Color(NONE,DARK,NONE);}
	/*blue*/
	inline const Color LightBlue()	{return Color(NONE,NONE,LIGHT);}
	inline const Color Blue()		{return Color(NONE,NONE,MEDIUM);}
	inline const Color DarkBlue()		{return Color(NONE,NONE,DARK);}
	/*magenta*/
	inline const Color LightMagenta()	{return Color(LIGHT,NONE,LIGHT);}
	inline const Color Magenta()		{return Color(MEDIUM,NONE,MEDIUM);}
	inline const Color DarkMagenta()	{return Color(DARK,NONE,DARK);}
	/*cyan*/
	inline const Color LightCyan()	{return Color(NONE,LIGHT,LIGHT);}
	inline const Color Cyan()		{return Color(NONE,NONE,MEDIUM);}
	inline const Color DarkCyan()		{return Color(NONE,DARK,DARK);}
	/*yellow*/
	inline const Color LightYellow()	{return Color(LIGHT,LIGHT,NONE);}
	inline const Color Yellow()		{return Color(MEDIUM,MEDIUM,NONE);}
	inline const Color DarkYellow()	{return Color(DARK,DARK,NONE);}
	/*orange*/
	inline const Color Orange()		{return Color(LIGHT,MEDIUM,NONE);}
	inline const Color DarkOrange()	{return Color(LIGHT,DARK,NONE);}
	/******/
        
        inline const Color GetColor(int i)
        {
            i = i % kNumColors;
            switch(i)
            {
            case kWhite: { return White(); }
            case kRed: { return Red(); }
            case kGreen: { return Green(); }
            case kBlue: { return Blue(); }
            case kMagenta: { return Magenta(); }
            case kCyan: { return Cyan(); }
            case kYellow: { return Yellow(); }
            case kOrange: { return Orange(); }
            case kBlack: { return Black(); }
            default: { return White(); }
            }
        }

}


#endif // COLOR_UTILS_H_
