#include "data.hpp"

namespace testrunner
{

Pose::Pose(
        cv::Matx44f const &pose,
        std::vector<cv::Matx44f> const &bones,
        std::vector<double> const &facemods):
    pose(pose),
    bones(bones),
    facemods(facemods)
{
}

}
// namespace testrunner
