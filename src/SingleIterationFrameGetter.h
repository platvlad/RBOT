#ifndef RBOT_SINGLEITERATIONFRAMEGETTER_H
#define RBOT_SINGLEITERATIONFRAMEGETTER_H

#include "FrameGetter.h"

class SingleIterationFrameGetter : public FrameGetter
{
public:
    SingleIterationFrameGetter(const cv::VideoCapture& videoCapture,
                               PoseEstimator6D* poseEstimator);

    ~SingleIterationFrameGetter() override = default;

    IterationResult getFrame(int frameCounter) override;
};
#endif //RBOT_SINGLEITERATIONFRAMEGETTER_H
