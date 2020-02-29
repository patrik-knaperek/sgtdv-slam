/*****************************************************/
//Organization: Stuba Green Team
//Authors: Martin Lučan, Andrej Érdelsky, Juraj Krasňanský
/*****************************************************/


#include "../include/SLAM.h"

SLAM::SLAM()
{

}

SLAM::~SLAM()
{

}

void SLAM::SetMapPublisher(ros::Publisher mapPublisher)
{
    m_mapPublisher = mapPublisher;
}

void SLAM::SetCarStatePublisher(ros::Publisher carStatePublisher)
{
    m_carStatePublisher = carStatePublisher;
}

void SLAM::Do(const sgtdv_msgs::ConeArr::ConstPtr &msg)
{
    sgtdv_msgs::ConeArr cones;
    sgtdv_msgs::CarState carState;



    m_carStatePublisher.publish(carState);
    m_mapPublisher.publish(cones);
}