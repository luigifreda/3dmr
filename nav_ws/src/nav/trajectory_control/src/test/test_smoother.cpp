#include "LoggerFile.h"
#include "PathManager.h"

#include <ros/ros.h>

#include <math.h> 
#include <string>
#include <sstream>
#include <iostream>

nav_msgs::Path loadPath(const std::string& file_path)
{
    std::cout << "loading path " << file_path << std::endl; 

    std::string global_frame = "map"; 

    nav_msgs::Path path;
    path.header.frame_id = global_frame;
    
    std::ifstream trajectory_filename;
    trajectory_filename.open(file_path.c_str());

    std::string line;
    int id = 0;
    while (std::getline(trajectory_filename, line))
    {
        double timestamp, x, y;
        // writing pattern: timestamp x y 
        if (sscanf(line.c_str(), "%lf, %lf, %lf", &timestamp, &x, &y) == 3)
        {
            //std::cout << "node " << id << " - x: " << x << " y: " << y << std::endl;
            geometry_msgs::PoseStamped ps;
            ps.header.frame_id = global_frame;
            ps.header.stamp = ros::Time(timestamp);
            ps.pose.position.x = x;
            ps.pose.position.y = y;
            ps.pose.position.z = 0;
            ps.pose.orientation.w = 1;
            path.poses.push_back(ps);
            id++;
        }
    }
    trajectory_filename.close();
    return path;
}

const std::string pathNameFromUri(const std::string& uri)
{
    std::size_t pos = uri.rfind('/');
    if (pos == std::string::npos) return ".";
    std::string pathname(uri.substr(0, pos));
    return pathname;
}
const std::string fileNameFromUri(const std::string& uri)
{
    std::size_t pos = uri.rfind('/');
    if (pos == std::string::npos) return uri;
    std::string filename(uri.substr(pos + 1, uri.length() - pos - 1));
    return filename;
}

void savePath(const nav_msgs::Path& path, const std::string& file_path)
{
    LoggerFile file(file_path);
    const size_t num_points = path.poses.size();
    file << "# timestamp, x, y" << std::endl; 
    for (int i = 0; i < num_points; i++)
    {
        file << i << "," << path.poses[i].pose.position.x << "," << path.poses[i].pose.position.y << std::endl;  
    }
}

int main(int argc, char** argv)
{
    if(argc < 3)
    {
        std::cout << "usage: " << argv[0] << " <path_file> <smoother_type>" << std::endl; 
    }
    const std::string file_path = argv[1]; 
    //PathSmoother::PathSmootherType smoother_type = PathSmoother::PathSmootherType::kSmootherSGw5;
    PathSmoother::PathSmootherType smoother_type = (PathSmoother::PathSmootherType)std::stoi(argv[2]); 

    std::cout << "opening " << file_path << std::endl; 
    std::cout << "using smoother " << PathSmoother::kPathSmootherTypeStr[smoother_type] << std::endl; 

    nav_msgs::Path path = loadPath(file_path);
    nav_msgs::Path path_smoothed = PathSmoother::smooth(path, smoother_type);

    const std::string out_file_path = pathNameFromUri(file_path) + "/" + "output_path.csv";
    std::cout << "saving smoothed path " << out_file_path << std::endl; 
    savePath(path_smoothed, out_file_path);

    return 0;
}
