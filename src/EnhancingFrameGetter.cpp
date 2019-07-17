#include "EnhancingFrameGetter.h"

EnhancingFrameGetter::EnhancingFrameGetter(const cv::VideoCapture& videoCapture,
                                           PoseEstimator6D* poseEstimator,
                                           size_t bufferSize) :
                                            FrameGetter(videoCapture, poseEstimator),
                                            bufferSize(bufferSize)
{
    residualBuffer = new float[bufferSize];
    for (int i = 0; i < bufferSize; ++i)
    {
        residualBuffer[i] = -1;
    }
}

EnhancingFrameGetter::~EnhancingFrameGetter()
{
    delete [] residualBuffer;
}

IterationResult EnhancingFrameGetter::getFrame()
{
    size_t nextBufferIndex = (bufferIndex + 1) % bufferSize;
    if ((residualBuffer[bufferIndex] >= residualBuffer[nextBufferIndex] &&        // not better than 10 iterations before
        residualBuffer[nextBufferIndex] >= 0 &&                                  // at least 10 iterations on first frame
        (residualBuffer[bufferIndex] <= lastFrameResidual || iterationCounter > bufferSize)) ||  // not worse than previous frame
        firstFrame)
    {
        firstFrame = false;
        if (videoCapture.isOpened())
        {
            videoCapture >> lastFrame;
        }
        else
        {
            lastFrame = cv::Mat();
        }
        if (lastFrame.empty())
        {
            return IterationResult(lastFrame, 0.0f, true);
        }
        iterationCounter = 0;
        if (runToggleTracking)
        {
            poseEstimator->toggleTracking(lastFrame, 0, false);
            lastFrameResidual = poseEstimator->estimatePoses(lastFrame, false, false);
            runToggleTracking = false;
        }
        else
        {
            lastFrameResidual = poseEstimator->estimatePoses(lastFrame, false, true);
        }
        bufferIndex = nextBufferIndex;
        residualBuffer[bufferIndex] = lastFrameResidual;
        return IterationResult(lastFrame, lastFrameResidual, true);
    }
    else
    {
        bufferIndex = nextBufferIndex;
        if (runToggleTracking)
        {
            poseEstimator->toggleTracking(lastFrame, 0, false);
            residualBuffer[bufferIndex] = poseEstimator->estimatePoses(lastFrame, false, false);
            runToggleTracking = false;
        }
        else
        {
            residualBuffer[bufferIndex] = poseEstimator->estimatePoses(lastFrame, false, true);
        }
        ++iterationCounter;
        return IterationResult(lastFrame, residualBuffer[bufferIndex], false);
    }
}