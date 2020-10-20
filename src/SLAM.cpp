/*****************************************************/
//Organization: Stuba Green Team
//Authors: Martin Lučan, Juraj Krasňanský
/*****************************************************/


#include "../include/SLAM.h"


SLAM::SLAM()
{
    m_muUpdate = cv::Mat::zeros(cv::Size(3, 1), CV_32FC1);
    m_lastPose.theta = 0.f;
    m_lastPose.x = 0.f;
    m_lastPose.y = 0.f;

    SetupNoiseMatrices();

    m_motion = cv::Mat_<float>(3, 1, CV_32FC1);
    m_Jakobian = cv::Mat::zeros(3, 3, CV_32FC1);
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

    m_lastPose = carPose;
}

void SLAM::SetupMatrices(size_t size)
{
    m_muUpdate.resize(size);
    m_covUpdate = cv::Mat_<float>(size, size, CV_32FC1);
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

void SLAM::InitPose(Pose &pose, const geometry_msgs::Pose &msg)
{
    pose.x = msg.position.x;
    pose.y = msg.position.y;
    //pose.theta = msg->orientation.  TODO: kvaterniony do stupnov

    float xDiff = pose.x - m_lastPose.x;
    float yDiff = pose.y - m_lastPose.y;
    m_traveledDistance = sqrt(xDiff * xDiff + yDiff * yDiff);
    m_rotationDiff = pose.theta - m_lastPose.theta;
}

void SLAM::InitObservations(const SLAMMsg &msg)
{
    m_observations.reserve(msg.cones->cones.size());

    for (size_t i = 0; i < msg.cones->cones.size(); i++)
    {
        Observation temp;

        temp.distance = cv::norm(cv::Vec2f( msg.cones->cones[i].coords.x,  msg.cones->cones[i].coords.y)
         - cv::Vec2f(msg.pwc->pose.position.x, msg.pwc->pose.position.y));
        temp.alpha = std::atan2(msg.cones->cones[i].coords.y, msg.cones->cones[i].coords.x);

        m_observations.push_back(temp);
    }
}

void SLAM::EkfPredict(const Pose &pose)
{
    int n = m_muUpdate.size().height;
    float cosRes = cos(*(m_muUpdate.ptr<float>(2)));
    float sinRes = sin(*(m_muUpdate.ptr<float>(2)));

    *(m_motion.ptr<float>(0)) = m_traveledDistance * cosRes;
    *(m_motion.ptr<float>(1)) = m_traveledDistance * sinRes;
    *(m_motion.ptr<float>(2)) = m_rotationDiff;

    *(m_Jakobian.ptr<float>(0) + 2) = -m_traveledDistance * sinRes;
    *(m_Jakobian.ptr<float>(1) + 2) = m_traveledDistance * cosRes;

    cv::Mat F(3, n - 3, CV_32FC1, cv::Scalar(0.));
    F.at<float>(0, 0) = 1.f;
    F.at<float>(1, 1) = 1.f;
    F.at<float>(2, 2) = 1.f;

    auto transpF = F.t();
    m_muPredict = m_muUpdate + (transpF * m_motion);
    
    cv::Mat G = cv::Mat::eye(n, n, CV_32FC1);
    G += transpF * m_Jakobian * F;
    cv::Mat noise = transpF * m_RT * F;

    m_covPredict = G * m_covUpdate * G.t() + noise;
}

void SLAM::EkfUpdate()
{
    for(size_t i = 0; i < m_observations.size(); i++)
    {
        int index = -2;
        Observation obs = m_observations[i];
        size_t recordsCount = m_muPredict.size().height;
        size_t currentKnownCones = (recordsCount - 3) / 2;
        std::vector<float> euclideanConeDist;
        euclideanConeDist.reserve(currentKnownCones);   //needs one less but 0 - 1 as size_t would give max number of 64bit uint

        cv::Point2f globalConePos(
            m_muPredict.at<float>(0, 0) + obs.distance * cos(obs.alpha + m_muPredict.at<float>(2, 0)),
            m_muPredict.at<float>(1, 0) + obs.distance * sin(obs.alpha + m_muPredict.at<float>(2, 0))
        );

        float min = std::numeric_limits<float>::max();
        size_t minIdx = 0;

        for (size_t j = 0; j < currentKnownCones; j++)
        {            
            float tempX = globalConePos.x - m_muUpdate.at<float>(i * 2 + 3);
            float tempY = globalConePos.y - m_muUpdate.at<float>(i * 2 + 4);
            float tempDist(sqrt(tempX * tempX + tempY * tempY));

            if (tempDist < min)
            {
                minIdx = j;
                min = tempDist;
            }

            euclideanConeDist.push_back(tempDist);
        }

        size_t predictRowIndex = 2 * minIdx + 3;

        if (currentKnownCones == 0 || euclideanConeDist[minIdx] >= THRESHOLD_DISTANCE)
        {
            m_muPredict.push_back(cv::Mat(1, 1, CV_32FC1, cv::Scalar(globalConePos.x)));
            m_muPredict.push_back(cv::Mat(1, 1, CV_32FC1, cv::Scalar(globalConePos.y)));

            m_covPredict.push_back(cv::Mat::zeros(2, m_covPredict.size().width, CV_32FC1));
            cv::hconcat(m_covPredict, cv::Mat::zeros(m_covPredict.size().height, 2, CV_32FC1), m_covPredict);

            m_covPredict.at<float>(recordsCount, recordsCount) = INF;
            m_covPredict.at<float>(recordsCount + 1, recordsCount + 1) = INF;

            predictRowIndex = m_muPredict.size().height - 2;
        }

        size_t N = m_muPredict.size().height;
        cv::Mat delta(2, 1, CV_32FC1);

        delta.at<float>(0, 0) = m_muPredict.at<float>(predictRowIndex, 0) - m_muPredict.at<float>(0, 0);
        delta.at<float>(1, 0) = m_muPredict.at<float>(predictRowIndex, 1) - m_muPredict.at<float>(1, 0);

        float q = delta.t().dot(delta);
        float sq = sqrt(q);

        float z_theta = std::atan2(delta.at<float>(0, 0), delta.at<float>(1, 0));
        cv::Mat z_hat = (cv::Mat_<float>(2, 1, CV_32FC1) << sq, z_theta - m_muPredict.at<float>(2, 0)); //TODO

        cv::Mat F = cv::Mat::zeros(5, N, CV_32FC1);
        F.at<float>(0, 0) = 1.f;
        F.at<float>(1, 1) = 1.f;
        F.at<float>(2, 2) = 1.f;
        F.at<float>(3, index) = 1;
        F.at<float>(4, index + 1) = 1;

        cv::Mat H_z = (cv::Mat_<float>(2, 5, CV_32FC1) << -sq * delta.at<float>(1), 0.f, sq * delta.at<float>(0), sq * delta.at<float>(1),
            delta.at<float>(1), -delta.at<float>(0), -q, -delta.at<float>(1), delta.at<float>(0));
        
        cv::Mat H = 1.f / q * (H_z * F);

        cv::Mat K = (m_covPredict * H.t()) * ( (H * m_covPredict) * H.t() + m_QT).inv();

        cv::Mat z_dif = (cv::Mat_<float>(2, 1, CV_32FC1) << obs.distance, obs.alpha);
        z_dif -= z_hat;

        ModuloMatMembers(z_dif, 2 * M_PI);
        z_dif -= M_PI;        

        m_muUpdate = m_muPredict + K * z_dif;
        m_covUpdate = cv::Mat::eye(N, N, CV_32FC1) - (K * H) * m_covPredict;
    }
}

void SLAM::SetupNoiseMatrices()
{
    m_RT = (cv::Mat_<float>(3, 3, CV_32FC1) << 0.0000001f, 0.f, 0.f, 0.f, 0.0000001f, 0.f, 0.f, 0.f, 0.0000001f);
    m_QT = (cv::Mat_<float>(2, 2, CV_32FC1) << 0.0000001f, 0.f, 0.f, 0.0000001f);
}

void SLAM::ModuloMatMembers(cv::Mat &mat, float modulo) const
{
    cv::Mat copy;
    cv::Mat buff;
    mat.copyTo(copy);

    copy += M_PI;
    copy /= (2 * M_PI);
    copy.convertTo(buff, CV_32SC1);

    buff *= 2 * M_PI;
    mat -= buff - M_PI;
}