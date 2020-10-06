/*****************************************************/
//Organization: Stuba Green Team
//Authors: Martin Lučan, Juraj Krasňanský
/*****************************************************/


#include <ros/ros.h>
#include <sgtdv_msgs/Point2DArr.h>
#include <sgtdv_msgs/ConeArr.h>
#include <sgtdv_msgs/CarState.h>
#include "Messages.h"
#include <cmath>
#include "opencv2/core/core.hpp"
#include "opencv2/core/saturate.hpp"
#include <vector>
#include <limits>
#include <geometry_msgs/Pose.h>
#include <sgtdv_msgs/Cone.h>

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
    void SetCarStatePublisher(ros::Publisher carStatePublisher);
    void Do(const SLAMMsg &msg);

private:
    ros::Publisher m_mapPublisher;
    ros::Publisher m_carStatePublisher;
    std::vector<Observation> m_observations;
    cv::Mat_<float> m_muUpdate;           //vektor stavov
    cv::Mat_<float> m_covUpdate;          //matica vztahov
    cv::Mat_<float> m_muPredict;
    cv::Mat_<float> m_covPredict;
    Pose m_lastPose;
    float m_traveledDistance = 0.f;
    float m_rotationDiff = 0.f;
    cv::Mat m_RT;       //motion noise
    cv::Mat m_QT;       //measurement noise

    //preallocated buff objects
    cv::Mat m_motion;
    cv::Mat m_Jakobian;

    void SetupMatrices(size_t size);
    void ZeroDiagonal(cv::Mat mat, size_t rowCount) const;
    void InitPose(Pose &pose, const geometry_msgs::Pose &msg);
    void InitObservations(const SLAMMsg &msg);
    void EkfPredict(const Pose &pose);
    void EkfUpdate();

    void SetupNoiseMatrices();
};