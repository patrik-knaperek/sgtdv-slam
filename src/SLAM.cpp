/*****************************************************/
//Organization: Stuba Green Team
//Authors: Martin Lučan, Juraj Krasňanský
/*****************************************************/


#include "../include/SLAM.h"


SLAM::SLAM()
{
    m_muUpdate = cv::Mat::zeros(cv::Size(3, 1), CV_32FC1);
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

void SLAM::Do(const SLAMMsg &msg)
{
    Pose carPose;
    sgtdv_msgs::ConeArr cones;
    sgtdv_msgs::CarState carState;

    m_observations.clear();

    SetupMatrices(msg.cones->cones.size() * 2 + 3);
    InitPose(carPose, msg.pwc->pose);
    InitObservations(msg);
    EkfPredict(carPose);
    EkfUpdate();

    m_carStatePublisher.publish(carState);
    m_mapPublisher.publish(cones);
}

void SLAM::SetupMatrices(size_t size)
{
    m_muUpdate.resize(size);
    m_covUpdate = cv::Mat_<float>(size, size);
    cv::setIdentity(m_covUpdate, std::numeric_limits<float>::min());
    ZeroDiagonal(m_covUpdate, 3);
}

void SLAM::ZeroDiagonal(cv::Mat mat, size_t rowCount) const
{
    for(size_t i = 0; i < rowCount; i++)
    {
        *(mat.ptr<float>(i) + i) = 0.f;
    }
}

void SLAM::InitPose(Pose &pose, const geometry_msgs::Pose &msg) const
{
    pose.x = msg.position.x;
    pose.y = msg.position.y;
    //pose.theta = msg->orientation.  TODO: kvaterniony do stupnov
}

void SLAM::InitObservations(const SLAMMsg &msg)
{
    m_observations.reserve(msg.cones->cones.size());

    for (size_t i = 0; i < msg.cones->cones.size(); i++)
    {
        Observation temp;

        temp.distance = cv::norm(cv::Vec2f( msg.cones->cones[i].coords.x,  msg.cones->cones[i].coords.y)
         - cv::Vec2f(msg.pwc->pose.position.x, msg.pwc->pose.position.y));
        //temp.alpha = ;    //TODO: orientation from quaternions

        m_observations.push_back(temp);
    }
}

void SLAM::EkfPredict(const Pose &pose)
{

}

void SLAM::EkfUpdate()
{

}