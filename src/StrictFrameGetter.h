#ifndef RBOT_STRICTFRAMEGETTER_H
#define RBOT_STRICTFRAMEGETTER_H

#include "FrameGetter.h"

class StrictFrameGetter : public FrameGetter
{
public:
    StrictFrameGetter(const cv::VideoCapture& videoCapture,
                      PoseEstimator6D* poseEstimator,
                      size_t numOfIterations = 10);

    ~StrictFrameGetter() override = default;

    IterationResult getFrame() override;
private:
    size_t numOfIterations;
    size_t currentIteration;
};
#endif //RBOT_STRICTFRAMEGETTER_H
