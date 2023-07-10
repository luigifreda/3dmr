/**
* This file is part of the ROS package trajectory_control which belongs to the framework 3DPATROLLING. 
*
* Copyright (C) 2016-present Luigi Freda <freda at diag dot uniroma1 dot it> and Alcor Lab (La Sapienza University)
* For more information see <https://gitlab.com/luigifreda/3dpatrolling>
*
* 3DPATROLLING is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* 3DPATROLLING is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with 3DPATROLLING. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef LOGGER_FILE_H
#define LOGGER_FILE_H

#include <iostream>
#include <fstream>

///	\class LoggerFile
///	\author Luigi Freda
///	\brief A class implementing a logger which writes on a file (it is capable of intercepting std::endl)
///	\note
/// 	\todo 
///	\date
///	\warning
class LoggerFile
{
public:

    LoggerFile(const std::string &filename) : _filename(filename)
    {
        if (!filename.empty())
        {
            _ofile.open(filename.c_str(), std::fstream::out);
            if (!_ofile.is_open())
            {
                std::cout << "LoggerFile: ERROR: unable to open file" << filename << std::endl; 
            }
        }
        else
        {
            std::cout << "LoggerFile: ERROR: filename empty";
        }
    }

    ~LoggerFile()
    {
        if (_ofile.is_open())
        {
            _ofile.close();
        }
    }

    template <typename T>
    LoggerFile &operator<<(const T &a)
    {
        _ofile << a;
        return *this;
    }

    LoggerFile &operator<<(std::ostream& (*pf) (std::ostream&))
    {
        // here we intercept std::endl
        _ofile << pf;
        //_bFirst = false;
        return *this;
    }

    /// Writes the block of data pointed by s, with a size of n characters, into the output buffer
    void Write(const char* s, std::streamsize n)
    {
        _ofile.write(s, n);
    }

    void Clear()
    {
        _ofile.clear();
    }

protected:
    std::fstream _ofile;
    std::string _filename;
};


#endif // LOGGER_FILE_H

