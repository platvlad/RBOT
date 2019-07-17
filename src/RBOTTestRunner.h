#ifndef RBOT_RBOTTESTRUNNER_H
#define RBOT_RBOTTESTRUNNER_H

#include <opencv4/opencv2/core/matx.hpp>
#include "argparsing.h"
#include "object3d.h"
#include "pose_estimator6d.h"
#include "EnhancingFrameGetter.h"
#include "data.hpp"
#include <opencv2/videoio.hpp>

namespace testrunner
{

    class RBOTTestRunner
    {
    public:
        explicit RBOTTestRunner(TrackingConfig& config);

        ~RBOTTestRunner();

        void runTest();

    private:
        boost::filesystem::path resultsFile;
        bool showGui;

        cv::Matx33f camera;
        cv::Matx44f groundTruth;
        cv::VideoCapture videoCapture = cv::VideoCapture();
        std::vector<Object3D*> objects;
        PoseEstimator6D* poseEstimator;
        std::map<int, Pose> resultPoses;

        size_t frameNumber = 0;
        size_t width = 0;
        size_t height = 0;
        float zNear = 0.0;
        float zFar = 0.0;

        void getCamera(const boost::filesystem::path& path);

        void getGroundTruth(const boost::filesystem::path& path);

        void getVideo(const boost::filesystem::path& videoPath);

        void getObject(const boost::filesystem::path& meshPath);

        cv::Mat drawResultOverlay(const cv::Mat& frame);

        void writeResult();

        static cv::Matx33f convertCameraMatrix(const cv::Matx44f& matrix4);

        static void convertTransformMatrix(cv::Matx44f &openGLMatrix);
    };
}
#endif //RBOT_RBOTTESTRUNNER_H
