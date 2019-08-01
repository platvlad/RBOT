#ifndef RBOT_ENHANCINGFRAMEGETTER_H
#define RBOT_ENHANCINGFRAMEGETTER_H

#include "FrameGetter.h"

class EnhancingFrameGetter : public FrameGetter
{
public:
    EnhancingFrameGetter(const cv::VideoCapture& videoCapture,
                         PoseEstimator6D* poseEstimator,
                         size_t bufferSize = 10);

    ~EnhancingFrameGetter() override;

    IterationResult getFrame(int frameCounter) override;
private:
    const size_t bufferSize;
    float* residualBuffer;
    size_t bufferIndex = 0;
    size_t iterationCounter = 0;
    float lastFrameResidual = 0;
    bool firstFrame = true;
};
#endif //RBOT_ENHANCINGFRAMEGETTER_H
