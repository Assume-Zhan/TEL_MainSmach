#pragma once

#include <string>
#include <queue>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include "geometry_msgs/Point.h"

typedef enum{

    IDLE = 0,
    FIRST_CATCH,
    SECOND_CATCH,
    PUT_BLOCK,
    SECOND_READY, /* Start second stage */
    INSIDE_TWENTY,
    INSIDE_FORTY,
    INSIDE_SIXTY,
    OUT_SIXTY,
    THIRD_READY, /* Start third stage */
    TURBO_UP,
    FLAT_SLOWDOWN,
    MOVE_OUT,
    FINISHED

} PathStatus;

class PathTrace{
public:

    PathTrace();

    std::queue<std::pair<geometry_msgs::Point, char>> getPath(PathStatus pathType);

    geometry_msgs::Point getCalibrationPoint(PathStatus pathType);

private:

    void readPath(std::string file_path);

    PathStatus pathPointer = IDLE;
    std::vector<std::queue<std::pair<geometry_msgs::Point, char>>> Path;

};