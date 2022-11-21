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

    this->Path.resize(FINISHED);

    for(auto path : pathConfig){

        auto pathName = path["IDLE"];
        for(auto xyz : pathName){
            geometry_msgs::Point point;
            point.x = xyz["x"].as<double>();
            point.y = xyz["y"].as<double>();
            point.z = xyz["z"].as<double>();

            this->Path[IDLE].push({point, xyz["mode"].as<char>()});
        }

        pathName = path["FIRST_CATCH"];
        for(auto xyz : pathName){
            geometry_msgs::Point point;
            point.x = xyz["x"].as<double>();
            point.y = xyz["y"].as<double>();
            point.z = xyz["z"].as<double>();

            this->Path[IDLE].push({point, xyz["mode"].as<char>()});
        }

        pathName = path["SECOND_CATCH"];
        for(auto xyz : pathName){
            geometry_msgs::Point point;
            point.x = xyz["x"].as<double>();
            point.y = xyz["y"].as<double>();
            point.z = xyz["z"].as<double>();

            this->Path[IDLE].push({point, xyz["mode"].as<char>()});
        }

        pathName = path["PUT_BLOCK"];
        for(auto xyz : pathName){
            geometry_msgs::Point point;
            point.x = xyz["x"].as<double>();
            point.y = xyz["y"].as<double>();
            point.z = xyz["z"].as<double>();

            this->Path[IDLE].push({point, xyz["mode"].as<char>()});
        }

        pathName = path["SECOND_READY"];
        for(auto xyz : pathName){
            geometry_msgs::Point point;
            point.x = xyz["x"].as<double>();
            point.y = xyz["y"].as<double>();
            point.z = xyz["z"].as<double>();

            this->Path[IDLE].push({point, xyz["mode"].as<char>()});
        }

        pathName = path["INSIDE_TWENTY"];
        for(auto xyz : pathName){
            geometry_msgs::Point point;
            point.x = xyz["x"].as<double>();
            point.y = xyz["y"].as<double>();
            point.z = xyz["z"].as<double>();

            this->Path[IDLE].push({point, xyz["mode"].as<char>()});
        }

        pathName = path["INSIDE_FORTY"];
        for(auto xyz : pathName){
            geometry_msgs::Point point;
            point.x = xyz["x"].as<double>();
            point.y = xyz["y"].as<double>();
            point.z = xyz["z"].as<double>();

            this->Path[IDLE].push({point, xyz["mode"].as<char>()});
        }

        pathName = path["INSIDE_SIXTY"];
        for(auto xyz : pathName){
            geometry_msgs::Point point;
            point.x = xyz["x"].as<double>();
            point.y = xyz["y"].as<double>();
            point.z = xyz["z"].as<double>();

            this->Path[IDLE].push({point, xyz["mode"].as<char>()});
        }

        pathName = path["OUT_SIXTY"];
        for(auto xyz : pathName){
            geometry_msgs::Point point;
            point.x = xyz["x"].as<double>();
            point.y = xyz["y"].as<double>();
            point.z = xyz["z"].as<double>();

            this->Path[IDLE].push({point, xyz["mode"].as<char>()});
        }

        pathName = path["THIRD_READY"];
        for(auto xyz : pathName){
            geometry_msgs::Point point;
            point.x = xyz["x"].as<double>();
            point.y = xyz["y"].as<double>();
            point.z = xyz["z"].as<double>();

            this->Path[IDLE].push({point, xyz["mode"].as<char>()});
        }

        pathName = path["TURBO_UP"];
        for(auto xyz : pathName){
            geometry_msgs::Point point;
            point.x = xyz["x"].as<double>();
            point.y = xyz["y"].as<double>();
            point.z = xyz["z"].as<double>();

            this->Path[IDLE].push({point, xyz["mode"].as<char>()});
        }

        pathName = path["FLAT_SLOWDOWN"];
        for(auto xyz : pathName){
            geometry_msgs::Point point;
            point.x = xyz["x"].as<double>();
            point.y = xyz["y"].as<double>();
            point.z = xyz["z"].as<double>();

            this->Path[IDLE].push({point, xyz["mode"].as<char>()});
        }

        pathName = path["MOVE_OUT"];
        for(auto xyz : pathName){
            geometry_msgs::Point point;
            point.x = xyz["x"].as<double>();
            point.y = xyz["y"].as<double>();
            point.z = xyz["z"].as<double>();

            this->Path[IDLE].push({point, xyz["mode"].as<char>()});
        }

        pathName = path["FINISHED"];
        for(auto xyz : pathName){
            geometry_msgs::Point point;
            point.x = xyz["x"].as<double>();
            point.y = xyz["y"].as<double>();
            point.z = xyz["z"].as<double>();

            this->Path[IDLE].push({point, xyz["mode"].as<char>()});
        }
    }

}
