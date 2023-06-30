/*****************************************************/
//Organization: Stuba Green Team
//Authors: Martin Lučan, Juraj Krasňanský
/*****************************************************/


#include <ros/ros.h>
#include <sgtdv_msgs/Point2DArr.h>
#include <sgtdv_msgs/ConeArr.h>
#include <sgtdv_msgs/CarPose.h>
#include <fsd_common_msgs/CarState.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include "opencv2/core/core.hpp"
#include "opencv2/core/saturate.hpp"
#include <eigen3/Eigen/Dense>
#include <vector>
#include <limits>
#include <geometry_msgs/Pose.h>
#include <sgtdv_msgs/Cone.h>
#include <list>

constexpr float INF = 1e6;
constexpr float THRESHOLD_DISTANCE = 3.f;
using namespace std;


struct Pose
{
    Pose() { x = 0.f; y = 0.f; yaw = 0.f; };
    float x;
    float y;
    float yaw;
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

    void CarStateCallbackSim(const fsd_common_msgs::CarState::ConstPtr& msg);
    void CarStateCallbackReal(const sgtdv_msgs::CarPose::ConstPtr& msg);
    void PoseDiff(Pose &poseActual, Pose &posePrevious);
    void ConesCallbackSim(const sensor_msgs::PointCloud2::ConstPtr& msg);        
    void ConesCallbackReal(const sgtdv_msgs::ConeArr::ConstPtr& msg);
    void DataAssEuclid();
    void PubCarState();
    void PubCones();
    void SetupMatrices();
    void EkfPredict();
    void EkfUpdate();
    bool cumulatedMeas = true;
 

private:

    ros::Publisher m_mapPublisher;
    ros::Publisher m_carPosePublisher;

    Pose m_poseActual;
    Pose m_posePrevious;
    float m_positionDiff;
    float m_rotationDiff;
    float m_positionBearing;

 
    Eigen::Matrix3f m_RT;
    Eigen::Matrix2f m_QT;
    Eigen::VectorXf m_stateVector;
    Eigen::MatrixXf m_covMatrix;
    Eigen::Vector3f m_motion; 
  

    double m_odomX, m_odomY, m_odomYaw, m_coneRange, m_coneBearing, m_coneAbsX, m_coneAbsY, m_euclidDist;
    double m_coneColor;
    float m_euclidThresh = 0.5;
    int m_N = 0;
    vector<vector<double>> m_coneAbsVect;
    vector<vector<double>> m_coneMap;

    geometry_msgs::Point pointCarPose; 
    geometry_msgs::Point pointCone;
    std_msgs::ColorRGBA coneRGBA;
    sgtdv_msgs::CarPose carPose;
    sgtdv_msgs::ConeArr coneArr;         
    sgtdv_msgs::Cone cone; 
    visualization_msgs::Marker carPoseMarker;
    visualization_msgs::Marker coneMarker;


    void InitPose(Pose &pose, const geometry_msgs::Pose &msg);
    
    Eigen::Vector3f MotionModelDiff();

    
    int MapConesCounter(const Eigen::VectorXf stateVector);

    double GetDistance(const sgtdv_msgs::Point2D &p1, const sgtdv_msgs::Point2D &p2) const;
};