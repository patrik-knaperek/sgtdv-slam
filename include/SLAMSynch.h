/*****************************************************/
//Organization: Stuba Green Team
//Authors: Juraj Krasňanský
/*****************************************************/


#include <ros/ros.h>
#include "SLAM.h"
#include <sgtdv_msgs/ConeArr.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include "Messages.h"

class SLAMSynch
{
public:
    SLAMSynch();
    ~SLAMSynch();

    void SetMapPublisher(ros::Publisher mapPublisher);
    void SetCarStatePublisher(ros::Publisher carStatePublisher);
    void DoMap(const sgtdv_msgs::ConeArr::ConstPtr &msg);
    void DoPose(const geometry_msgs::PoseWithCovariance::ConstPtr &msg);

private:
    SLAM m_slam;
    bool m_poseReceived;
    SLAMMsg m_slamMsg;
};