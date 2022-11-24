/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/


#include "../include/SLAMSynch.h"

SLAMSynch::SLAMSynch()
{
    m_poseReceived = false;
}

SLAMSynch::~SLAMSynch()
{

}

void SLAMSynch::SetCarPosePublisher(ros::Publisher carPosePublisher)
{
    m_slam.SetCarPosePublisher(carPosePublisher);
}

void SLAMSynch::SetMapPublisher(ros::Publisher mapPublisher)
{
    m_slam.SetMapPublisher(mapPublisher);
}

void SLAMSynch::DoPose(const geometry_msgs::PoseWithCovariance::ConstPtr &msg)
{
    m_slamMsg.pwc = msg;
    m_poseReceived = true;
}

void SLAMSynch::DoMap(const sgtdv_msgs::ConeArr::ConstPtr &msg)
{
    if (m_poseReceived)
    {
        m_poseReceived = false;
        m_slamMsg.cones = msg;
        m_slam.Do(m_slamMsg);
    }
}