#include "path_trace.h"

PathTrace::PathTrace(){}

std::queue<std::pair<geometry_msgs::Point, char>> PathTrace::getPath(PathStatus pathType){
    return this->Path[pathType];
}

geometry_msgs::Point PathTrace::getCalibrationPoint(PathStatus pathType){
    return this->Path[pathType].front().first;
}

void PathTrace::readPath(std::string file_path){
    YAML::Node pathConfig = YAML::LoadFile(file_path);

    this->Path.resize(FINISHED + 5);

    int i = 0;
    for(auto path : pathConfig){

        auto pathName = path[PathS[i]];
        for(auto xyz : pathName){
            geometry_msgs::Point point;
            point.x = xyz["x"].as<double>();
            point.y = xyz["y"].as<double>();
            point.z = xyz["z"].as<double>();

            this->Path[i].push({point, xyz["mode"].as<char>()});
        }

        if(i >= FINISHED) break;
        i++;
    }
}
