#include "SingleIterationFrameGetter.h"

SingleIterationFrameGetter::SingleIterationFrameGetter(const cv::VideoCapture& videoCapture,
                                                       PoseEstimator6D* poseEstimator) :
                                                        FrameGetter(videoCapture, poseEstimator)
{
}

IterationResult SingleIterationFrameGetter::getFrame(int frameCounter)
{
    if (videoCapture.isOpened())
    {
        videoCapture >> lastFrame;
    }
    float residual = 0.0f;
    if (!lastFrame.empty())
    {
        if (runToggleTracking)
        {
            poseEstimator->toggleTracking(lastFrame, 0, false);
//            residual = poseEstimator->estimatePoses(lastFrame, frameCounter, false, false);
            runToggleTracking = false;
        }
        else
        {
            residual = poseEstimator->estimatePoses(lastFrame, frameCounter + 1, false, true);
        }
    }
    return IterationResult(lastFrame, residual, true);
}