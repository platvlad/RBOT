#include "io.hpp"

#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/lexical_cast.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core.hpp>

#include <yaml-cpp/yaml.h>

namespace testrunner
{

namespace
{
    void parseNode(boost::filesystem::path const &configPath,
                   YAML::Node &node)
    {
        boost::filesystem::ifstream configStream(configPath);
        YAML::Parser parser(configStream);
        parser.GetNextDocument(node);
    }


    template<class T>
    T readMat(YAML::Node const &node)
    {
        T mat;
        int const rowCount = static_cast<int>(node.size());
        for (int rowIdx = 0; rowIdx < rowCount; ++rowIdx) {
            YAML::Node const &row = node[rowIdx];
            int const colCount = static_cast<int>(row.size());
            for (int colIdx = 0; colIdx < colCount; ++colIdx) {
                row[colIdx] >> mat(rowIdx, colIdx);
            }
        }
        return mat;
    }

    template<class T>
    T readVec(YAML::Node const &node)
    {
        T vec;
        int const nodeSize = static_cast<int>(node.size());
        for (int i = 0; i < nodeSize; ++i) {
            node[i] >> vec(0, i);
        }
        return vec;
    }

    cv::Matx44f readPosition(YAML::Node const &node)
    {
        cv::Matx13f const t = readVec<cv::Matx13f>(node["t"]);
        cv::Matx33f const r = readMat<cv::Matx33f>(node["R"]);
        return { r(0, 0), r(0, 1), r(0, 2), t(0, 0),
                 r(1, 0), r(1, 1), r(1, 2), t(0, 1),
                 r(2, 0), r(2, 1), r(2, 2), t(0, 2),
                 0.0f,    0.0f,    0.0f,    1.0f };
    }

    void operator >>(YAML::Node const &node, cv::Matx44f &pose)
    {
        pose = readPosition(node);
    }

    template<class T>
    std::vector<T> readVector(YAML::Node const &node)
    {
        std::vector<T> vec(node.size());
        int const nodeSize = static_cast<int>(node.size());
        for (int i = 0; i < nodeSize; ++i) {
            node[i] >> vec[i];
        }
        return vec;
    }

    std::pair<int, Pose> readFrameAndPose(YAML::Node const &node)
    {
        std::pair<int, Pose> frameAndPose;

        node["frame"] >> frameAndPose.first;

        frameAndPose.second.pose = readPosition(node["pose"]);

        YAML::Node const *bonesNode = node.FindValue("bones");
        if (bonesNode) {
            frameAndPose.second.bones =
                    readVector<cv::Matx44f>(*bonesNode);
        }

        YAML::Node const *facemodsNode = node.FindValue("facemods");
        if (facemodsNode) {
            frameAndPose.second.facemods =
                    readVector<double>(*facemodsNode);
        }

        return frameAndPose;
    }
}


cv::Matx44f readCamera(boost::filesystem::path const &configPath)
{
    YAML::Node docNode;
    parseNode(configPath, docNode);
    YAML::Node const &matNode = docNode["matrix"];
    return readMat<cv::Matx44f>(matNode);
}


std::map<int, Pose> readPoses(boost::filesystem::path const &configPath)
{
    YAML::Node docNode;
    parseNode(configPath, docNode);

    YAML::Node const &framesNode = docNode["frames"];
    std::map<int, Pose> allPoses;
    int nodeSize = static_cast<int>(framesNode.size());
    for (int i = 0; i < nodeSize; ++i) {
        allPoses.insert(readFrameAndPose(framesNode[i]));
    }

    return allPoses;
}

float readDiameter(boost::filesystem::path const &path)
{
    YAML::Node docNode;
    parseNode(path, docNode);

    float diameter;
    docNode["diameter"] >> diameter;

    return diameter;
}


std::map<int, boost::filesystem::path> listFrames(
        boost::filesystem::path const &dir)
{
    typedef boost::filesystem::directory_iterator Iter;
    Iter const end;

    std::map<int, boost::filesystem::path> dst;
    for (Iter it(dir); it != end; ++it) {
        boost::filesystem::path const path = it->path();
        int const frame = boost::lexical_cast<int>(path.stem().string());
        dst.insert(std::make_pair(frame, it->path()));
    }

    return dst;
}

}
// namespace testrunner


namespace
{
    std::string protectInf(float const x)
    {
        if (isinf(x)) {
            return (x > 0) ? ".inf" : "-.inf";
        }
        return boost::lexical_cast<std::string>(x);
    }
}


namespace YAML
{

void operator <<(YAML::Emitter &emitter, cv::Matx44f const &pose)
{
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "R" << YAML::Value;
    {
        emitter << YAML::BeginSeq;
        for (int row = 0; row < 3; ++row) {
            emitter << YAML::Flow;
            emitter << YAML::BeginSeq;
            emitter << protectInf(pose(row, 0))
                    << protectInf(pose(row, 1))
                    << protectInf(pose(row, 2));
            emitter << YAML::EndSeq;
        }
        emitter << YAML::EndSeq;
    }
    emitter << YAML::Key << "t" << YAML::Value;
    {
        emitter << YAML::Flow;
        emitter << YAML::BeginSeq;
        emitter << protectInf(pose(0, 3))
                << protectInf(pose(1, 3))
                << protectInf(pose(2, 3));
        emitter << YAML::EndSeq;
    }
    emitter << YAML::EndMap;
}

}
// namespace YAML


namespace testrunner
{

void writePoses(std::map<int, Pose> const &poses,
                boost::filesystem::path const &dstPath)
{
    YAML::Emitter emitter;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "frames";
    emitter << YAML::Value;
    {
        emitter << YAML::BeginSeq;
        typedef std::pair<const int, Pose> FrameAndPose;
        BOOST_FOREACH (FrameAndPose const &frameAndPose, poses) {
            emitter << YAML::BeginMap;
            emitter << YAML::Key << "frame"
                    << YAML::Value << frameAndPose.first;
            emitter << YAML::Key << "pose"
                    << YAML::Value << frameAndPose.second.pose;
            if (!frameAndPose.second.bones.empty()) {
                emitter << YAML::Key << "bones"
                        << YAML::Value << frameAndPose.second.bones;
            }
            if (!frameAndPose.second.facemods.empty()) {
                emitter << YAML::Key << "facemods"
                        << YAML::Value << frameAndPose.second.facemods;
            }
            emitter << YAML::EndMap;
        }
        emitter << YAML::EndSeq;
    }
    emitter << YAML::EndMap;

    boost::filesystem::ofstream dstStream(dstPath);
    dstStream << emitter.c_str();
}


void writeErrors(
        std::map<int, float> const &errors,
        boost::filesystem::path const &dstPath)
{
    YAML::Emitter emitter;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "frames";
    emitter << YAML::Value;
    {
        emitter << YAML::BeginSeq;
        typedef std::pair<int, float> FrameAndError;
        BOOST_FOREACH (FrameAndError const &frameAndError, errors) {
            emitter << YAML::BeginMap;
            emitter << YAML::Key << "frame"
                    << YAML::Value << frameAndError.first;
            emitter << YAML::Key << "error"
                    << YAML::Value << protectInf(frameAndError.second);
            emitter << YAML::EndMap;
        }
        emitter << YAML::EndSeq;
    }
    emitter << YAML::EndMap;

    boost::filesystem::ofstream dstStream(dstPath);
    dstStream << emitter.c_str();
}


void writeDiameter(float diameter, boost::filesystem::path const &dstPath)
{
    YAML::Emitter emitter;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "diameter";
    emitter << YAML::Value << diameter;
    emitter << YAML::EndMap;

    boost::filesystem::ofstream dstStream(dstPath);
    dstStream << emitter.c_str();
}

}
// namespace testrunner
