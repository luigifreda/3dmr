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

#ifndef PP_EXCEPTION_H
#define PP_EXCEPTION_H

#include <exception>
#include <string>
#include <sstream>

///	\class Exception
///	\author Luigi Freda 
///	\brief A class for managing exceptions 
///	\note 
/// 	\todo 
///	\date
///	\warning
class Exception: public std::exception
{
public:

    Exception(const std::string& description);
    virtual ~Exception() throw();

    // override of std::exception::what()
    virtual const char* what() const throw();

private:

    std::string description_;
};


#define DEFINE_EXCEPTION(DERIVED, BASE)					\
									\
	class DERIVED : public BASE					\
	{								\
		public:							\
			DERIVED(const std::string& msg) : BASE(msg){}	\
			virtual ~DERIVED() throw(){}			\
	};								\


#define STR(x) #x


#define THROW_EXCEPTION(EXCEPTION, MSG)										\
{														\
	std::stringstream ss;											\
	ss << STR(EXCEPTION) << "(" << MSG << ") launched in function \'" << __PRETTY_FUNCTION__ << "\' at line " << __LINE__ ; \
	throw EXCEPTION(ss.str());											\
}



#endif
