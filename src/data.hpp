#ifndef TESTRUNNER_DATA_HPP
#define TESTRUNNER_DATA_HPP

#include <vector>
#include <opencv4/opencv2/core/matx.hpp>


namespace testrunner
{

struct Pose
{
    cv::Matx44f pose;
    std::vector<cv::Matx44f> bones;
    std::vector<double> facemods;

    Pose(cv::Matx44f const &pose = cv::Matx44f(),
         std::vector<cv::Matx44f> const &bones = std::vector<cv::Matx44f>(),
         std::vector<double> const &facemods = std::vector<double>());
};

}

#endif //TESTRUNNER_DATA_HPP
