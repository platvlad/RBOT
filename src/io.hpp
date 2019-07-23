#ifndef TESTRUNNER_IO_HPP
#define TESTRUNNER_IO_HPP

#include "data.hpp"

#include <map>

#include <boost/filesystem/path.hpp>

#include <opencv2/core/core.hpp>

namespace testrunner
{

cv::Matx44f readCamera(boost::filesystem::path const &configPath);

std::map<int, Pose> readPoses(boost::filesystem::path const &configPath);

std::map<int, boost::filesystem::path> listFrames(
        boost::filesystem::path const &dir);

void writePoses(std::map<int, Pose> const &poses,
                boost::filesystem::path const &dstPath);

void writeErrors(std::map<int, float> const &errors,
                 boost::filesystem::path const &dstPath);

void writeTimes(std::map<int, float> const &errors,
        boost::filesystem::path const &dstPath);

void writeDiameter(float diameter,
                   boost::filesystem::path const &dstPath);

}
// namespace testrunner

#endif // TESTRUNNER_IO_HPP
