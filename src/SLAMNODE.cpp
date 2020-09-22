/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/


#include <ros/ros.h>
#include "../include/SLAMSynch.h"
#include <sgtdv_msgs/ConeArr.h>
#include <sgtdv_msgs/CarState.h>

int main(int argc, char** argv)
{
    SLAMSynch synchObj;

    ros::init(argc, argv, "SLAM");
    ros::NodeHandle handle;

    ros::Publisher mapPublisher = handle.advertise<sgtdv_msgs::ConeArr>("slam_map", 1);
    ros::Publisher carStatePublisher = handle.advertise<sgtdv_msgs::CarState>("slam_pose", 1);

    synchObj.SetMapPublisher(mapPublisher);
    synchObj.SetCarStatePublisher(carStatePublisher);

    ros::Subscriber fusionSub = handle.subscribe("fusion_cones", 1, &SLAMSynch::DoMap, &synchObj);
    ros::Subscriber poseWithCov = handle.subscribe("pose", 1, &SLAMSynch::DoPose, &synchObj);

    ros::spin();

    return 0;
}