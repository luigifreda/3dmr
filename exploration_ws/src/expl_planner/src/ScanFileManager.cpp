/**
* This file is part of the ROS package trajectory_control which belongs to the framework 3DMR. 
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

#include "ScanFileManager.h"

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

#include <iomanip>
#include <memory>
#include <limits> 
#include <sstream> 

#include <iostream>
#include <fstream>

namespace explplanner
{

std::string ScanFileManager::kBaseFolderPath = "~/.ros/3dmr/"; 

std::string expand_user(std::string path) 
{
    if (not path.empty() and path[0] == '~') 
    {
        assert(path.size() == 1 or path[1] == '/');  // or other error handling
        char const* home = getenv("HOME");
        if (home or ((home = getenv("USERPROFILE")))) 
        {
            path.replace(0, 1, home);
        }
        else 
        {
            char const *hdrive = getenv("HOMEDRIVE"), 
                       *hpath = getenv("HOMEPATH");
            assert(hdrive);  // or other error handling
            assert(hpath);
            path.replace(0, 1, std::string(hdrive) + hpath);
        }
    }
    return path;
}

ScanFileManager::ScanFileManager(const int& robotId)
{
    setRobotId(robotId);

    fileEnding_ = "pcd";
    filePrefix_ = "pointcloud";

    boost::filesystem::path dir(folderPath_);
    if (!boost::filesystem::exists( dir )) 
    {
        if(boost::filesystem::create_directory(dir))
        {
            ROS_INFO_STREAM("Directory Created: "<< folderPath_);
        }    
        else
        {
            ROS_ERROR_STREAM("Directory could not be created: "<< folderPath_);
            quick_exit(1);
        }
    }
    else
    {
        ROS_WARN_STREAM("Directory already exists: "<< folderPath_);        
    }
    
}

std::string ScanFileManager::getFilePath(uint32_t index) 
{
    std::stringstream filePath;
    filePath << folderPath_ << "/";
    if (!filePrefix_.empty()) 
    {
        filePath << filePrefix_;
    }
    filePath << "_" << index << "." << fileEnding_;

    return filePath.str();
}

bool ScanFileManager::save(uint32_t index, ScanFileManager::PointCloud::Ptr& pclCloud) 
{
    bool ret = true; 

    ROS_INFO_STREAM("Saving point cloud with " << pclCloud->size() << " points.");

    std::string filePath = getFilePath(index);

    if (fileEnding_ == "ply") 
    {
        // Write .ply file.
        pcl::PLYWriter writer;
        bool binary = true;
        bool use_camera = false;
        if (writer.write(filePath, *pclCloud, binary, use_camera) != 0) 
        {
            ROS_ERROR("Something went wrong when trying to write the point cloud file.");
            return false;
        }
    }
    else if (fileEnding_ == "pcd") 
    {
        pcl::io::savePCDFile(filePath, *pclCloud);
    } 
    else 
    {
        ROS_ERROR_STREAM("Data format not supported.");
        return false;
    }

    ROS_INFO_STREAM("Saved point cloud to " << filePath << ".");
    return ret; 
}

bool ScanFileManager::save(uint32_t index, const pcl::PCLPointCloud2 & cloud2)
{
    bool ret = true; 

    ROS_INFO_STREAM("Saving pointcloud2 with " << cloud2.height * cloud2.width  << " points.");

    std::string filePath = getFilePath(index);

    if (fileEnding_ == "ply") 
    {
        // Write .ply file.
        pcl::PLYWriter writer;
        if (writer.writeBinary(filePath, cloud2) != 0) 
        {
            ROS_ERROR("Something went wrong when trying to write the point cloud file.");
            return false;
        }
    }
    else if (fileEnding_ == "pcd") 
    {
        pcl::PCDWriter writer;
        if (writer.writeBinaryCompressed(filePath, cloud2) !=0 ) // this should be the fastest way 
        {
            ROS_ERROR("Something went wrong when trying to write the point cloud file.");
            return false;
        }        
    } 
    else 
    {
        ROS_ERROR_STREAM("Data format not supported.");
        return false;
    }

    ROS_INFO_STREAM("Saved point cloud to " << filePath << ".");
    return ret;     
}

bool ScanFileManager::read(uint32_t index, ScanFileManager::PointCloud::Ptr& pclCloud) 
{
    std::string filePath = getFilePath(index);

    if (filePath.find(".ply") != std::string::npos) 
    {
        // Load .ply file.
        if (pcl::io::loadPLYFile(filePath, *pclCloud) != 0) 
        {
            ROS_ERROR_STREAM ("Couldn't read file " << filePath);
            return false;
        }
    } 
    else if (filePath.find(".pcd") != std::string::npos) 
    {
        // Load pcd file.
        if (pcl::io::loadPCDFile<ScanFileManager::Point> (filePath, *pclCloud) == -1) //* load the file
        {
            ROS_ERROR_STREAM ("Couldn't read file " << filePath);
            return false;
        }
    } 
    else 
    {
        ROS_ERROR_STREAM("Data format not supported.");
        return false;
    }
    return true;
}

bool ScanFileManager::read(uint32_t index, pcl::PCLPointCloud2 & cloud2)
{
    std::string filePath = getFilePath(index);

    if (filePath.find(".ply") != std::string::npos) 
    {
        // Load .ply file.
        if (pcl::io::loadPLYFile(filePath, cloud2) != 0) 
        {
            ROS_ERROR_STREAM ("Couldn't read file " << filePath);
            return false;
        }
    } 
    else if (filePath.find(".pcd") != std::string::npos) 
    {
        // Load pcd file.
        if (pcl::io::loadPCDFile(filePath, cloud2) == -1) //* load the file
        {
            ROS_ERROR_STREAM ("Couldn't read file " << filePath);
            return false;
        }
    } 
    else 
    {
        ROS_ERROR_STREAM("Data format not supported.");
        return false;
    }
    return true;
}

void ScanFileManager::setRobotId(int id)
{
    robot_id_ = id;
    std::stringstream ss; 
    ss << "ugv" << id+1; 
    robotName_ = ss.str();
    folderPath_ = expand_user(kBaseFolderPath) + robotName_;    
}

} // namespace explplanner
