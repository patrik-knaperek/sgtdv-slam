#include <ros/ros.h>
#include <sgtdv_msgs/Point2DArr.h>
#include <sgtdv_msgs/ConeArr.h>
#include <sgtdv_msgs/CarState.h>

class SLAM
{
public:
    SLAM();
    ~SLAM();

    void SetMapPublisher(ros::Publisher mapPublisher);
    void SetCarStatePublisher(ros::Publisher carStatePublisher);
    void Do(const sgtdv_msgs::ConeArr::ConstPtr &msg);

private:
    ros::Publisher m_mapPublisher;
    ros::Publisher m_carStatePublisher;
};