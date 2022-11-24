/*****************************************************/
//Organization: Stuba Green Team
//Authors: Martin Lučan, Juraj Krasňanský
/*****************************************************/


#include <ros/ros.h>
#include <sgtdv_msgs/Point2DArr.h>
#include <sgtdv_msgs/ConeArr.h>
#include <sgtdv_msgs/CarPose.h>
#include "Messages.h"
#include <cmath>
#include "opencv2/core/core.hpp"
#include "opencv2/core/saturate.hpp"
#include <vector>
#include <limits>
#include <geometry_msgs/Pose.h>
#include <sgtdv_msgs/Cone.h>
#include <list>

constexpr float INF = 1e6;
constexpr float THRESHOLD_DISTANCE = 3.f;

struct Pose
{
    Pose() { x = 0.f; y = 0.f; theta = 0.f; };
    float x;
    float y;
    float theta;
};

struct Observation
{
    Observation() { distance = 0.f; alpha = 0.f; };
    float distance;
    float alpha;
};

class SLAM
{
public:
    SLAM();
    ~SLAM();

    void SetMapPublisher(ros::Publisher mapPublisher);
    void SetCarPosePublisher(ros::Publisher carPosePublisher);
    void Do(const SLAMMsg &msg);

private:
    ros::Publisher m_mapPublisher;
    ros::Publisher m_carPosePublisher;
    std::vector<Observation> m_observations;
    std::list<sgtdv_msgs::Cone> m_coneCandidates;
    cv::Mat1f m_muUpdate;           //vektor stavov
    cv::Mat1f m_covUpdate;          //matica vztahov
    cv::Mat1f m_muPredict;
    cv::Mat1f m_covPredict;
    Pose m_lastPose;
    float m_traveledDistance = 0.f;
    float m_rotationDiff = 0.f;
    cv::Mat1f m_RT;       //motion noise
    cv::Mat1f m_QT;       //measurement noise

    //preallocated buff objects
    cv::Mat1f m_motion;
    cv::Mat1f m_Jakobian;
    cv::Mat1f m_Hz;
    cv::Mat1f m_Zdiff;
    cv::Mat1f m_Zhat;
    cv::Mat1f m_delta;
    cv::Mat1f m_buff1;
    cv::Mat1i m_buff2;

    void SetupMatrices(size_t size);
    void ZeroDiagonal(cv::Mat1f &mat, size_t rowCount) const;
    void InitPose(Pose &pose, const geometry_msgs::Pose &msg);
    void InitObservations(const SLAMMsg &msg);
    void EkfPredict(const Pose &pose);
    void EkfUpdate();
    void DataAssociation(const sgtdv_msgs::ConeArr::ConstPtr &cones, const geometry_msgs::Point &carPosition);
    void ModuloMatMembers(cv::Mat1f &mat, float modulo);

    void SetupNoiseMatrices();
    double GetDistance(const sgtdv_msgs::Point2D &p1, const sgtdv_msgs::Point2D &p2) const;
};