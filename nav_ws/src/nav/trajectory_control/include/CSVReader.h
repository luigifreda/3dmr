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

#pragma once 

#include <iostream>     // cout, endl
#include <fstream>      // fstream
#include <vector>
#include <string>
#include <boost/tokenizer.hpp>


///	\class CSVReader
///	\author Luigi
///	\brief A class for reading csv files 
///	\note 
/// 	\todo 
///	\date
///	\warning
class CSVReader
{
    typedef boost::tokenizer< boost::escaped_list_separator<char> > Tokenizer;
    
public: 
    
    // basic constructor: open file and parse it 
    CSVReader(const std::string& filename)
    {
        str_filename_ = filename;

        std::ifstream in(str_filename_);
        if (!in.is_open()) 
        {
            std::cout << "CSVReader::parse() - error opening file: " << str_filename_ << std::endl; 
        }
                
        std::vector< std::string > vec;
        std::string line;
    
        int i = 0; 
        while (std::getline(in,line))
        {
            Tokenizer tok(line);
            vec.assign(tok.begin(),tok.end());

            data_.push_back(vec);
            i++;
        }
    }

    void convertDataToDoubles(std::vector<std::vector<double>>& mat_data)
    {
        mat_data.resize(data_.size());
        for(size_t ii=0; ii<data_.size(); ii++)
        {
            const std::vector<std::string>& vec = data_[ii];
            for(int j=0; j<vec.size(); j++)
            {
                mat_data[ii].push_back(std::stod(vec[j])); 
            }
        }
    }
    
public: // getters 
    
    // get the mat in which the data have been put 
    std::vector< std::vector<std::string> >& getData() { return data_; }


public: // utils 
    
    void print(std::ostream& str = std::cout)
    {
        for(int i=0; i<data_.size(); i++)
        {
            for(int j=0; j<data_[i].size(); j++)
            {
                str << data_[i][j] << " ";
            }
            str << std::endl; 
        } 
    }
    
  
public: 
    
    std::string str_filename_; 
    std::vector< std::vector<std::string>> data_; // string data     
};


inline std::ostream& operator<<(std::ostream& str, CSVReader& csv_reader)
{
    csv_reader.print(str); 
    return str;
}


/// < for testing 
//int main(int argc, char* argv[])
//{
//    std::string csv_filename;
//    if(argc==1)
//    {
//        std::cout << "enter input filename" << std::endl; 
//    }
//    else
//    {
//        csv_filename = argv[1];
//        std::cout << "opening file: " << csv_filename << std::endl;
//    }
//    
//    CSVReader csv_reader(csv_filename); 
//    std::vector<std::vector<double>> mat_data;
//    csv_reader.convertDataToDoubles(mat_data);
//    csv_reader.print(); 
//}



