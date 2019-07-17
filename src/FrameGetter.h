#ifndef RBOT_FRAMEGETTER_H
#define RBOT_FRAMEGETTER_H
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include "pose_estimator6d.h"

struct IterationResult
{
    IterationResult(cv::Mat frame, float residual, bool isNextFrame) : frame(frame),
                                                                       residual(residual),
                                                                       isNextFrame(isNextFrame)
    {}

    cv::Mat frame;
    float residual;
    bool isNextFrame;

};

class FrameGetter
{

public:
    FrameGetter(const cv::VideoCapture& videoCapture, PoseEstimator6D* poseEstimator):
            videoCapture(videoCapture),
            poseEstimator(poseEstimator),
            lastFrame(cv::Mat())
    {
    }

    virtual ~FrameGetter() = default;

    virtual IterationResult getFrame() = 0;

    bool runToggleTracking = false;



protected:
    cv::VideoCapture videoCapture;
    PoseEstimator6D* poseEstimator;
    cv::Mat lastFrame;


};


#endif //RBOT_FRAMEGETTER_H
