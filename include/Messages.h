#pragma once

#include <sgtdv_msgs/ConeArr.h>
#include <geometry_msgs/PoseWithCovariance.h>

struct SLAMMsg
{
    sgtdv_msgs::ConeArrConstPtr cones;
    geometry_msgs::PoseWithCovarianceConstPtr pwc;
};