/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/


#include "../include/SLAMSynch.h"

SLAMSynch::SLAMSynch()
{
    m_receivedMap = false;
}

SLAMSynch::~SLAMSynch()
{

}

void SLAMSynch::SetCarStatePublisher(ros::Publisher carStatePublisher)
{
    m_slam.SetCarStatePublisher(carStatePublisher);
}

void SLAMSynch::SetMapPublisher(ros::Publisher mapPublisher)
{
    m_slam.SetMapPublisher(mapPublisher);
}

void SLAMSynch::DoPose()
{

}

void SLAMSynch::DoMap(const sgtdv_msgs::Point2DArr::ConstPtr &msg)
{
   // m_slam.Do(msg);
}