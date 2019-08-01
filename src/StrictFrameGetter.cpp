#include "StrictFrameGetter.h"

StrictFrameGetter::StrictFrameGetter(const cv::VideoCapture& videoCapture,
                                     PoseEstimator6D* poseEstimator,
                                     size_t numOfIterations) :
                                        FrameGetter(videoCapture, poseEstimator),
                                        numOfIterations(numOfIterations),
                                        currentIteration(0)
{
}

IterationResult StrictFrameGetter::getFrame(int frameCounter)
{
    bool isNextFrame = false;
    if (videoCapture.isOpened() && currentIteration == 0)
    {
        videoCapture >> lastFrame;
        isNextFrame = true;
    }
    ++currentIteration;
    currentIteration %= numOfIterations;
    float residual = 0.0f;
    if (!lastFrame.empty())
    {
        if (runToggleTracking)
        {
            poseEstimator->toggleTracking(lastFrame, 0, false);
            residual = poseEstimator->estimatePoses(lastFrame, false, false);
            runToggleTracking = false;
        }
        else
        {
            residual = poseEstimator->estimatePoses(lastFrame, false, true);
        }
    }
    std::pair<bool, cv::Mat> answer;
    return IterationResult(lastFrame, residual, isNextFrame);
}