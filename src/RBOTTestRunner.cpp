#include <iostream>

#include <boost/filesystem/operations.hpp>
#include <opencv4/opencv2/highgui.hpp>

#include "RBOTTestRunner.h"
#include "SingleIterationFrameGetter.h"
#include "io.hpp"

namespace testrunner
{
    RBOTTestRunner::RBOTTestRunner(TrackingConfig& config) : resultsFile(config.results),
                                                             showGui(config.showGui)
    {
        getCamera(config.camera);
        getGroundTruth(config.groundTruth);
        if (config.rgb)
        {
            getVideo(config.rgb.value());
        }
        getObject(config.mesh);

        cv::Vec3f t = cv::Vec3f(groundTruth(0, 3), groundTruth(1, 3), groundTruth(2, 3));
        double distanceToObject = cv::norm(t);
        zNear = distanceToObject * 0.01;
        zFar = distanceToObject * 100;

        cv::Matx14f distCoeffs =  cv::Matx14f(0.0, 0.0, 0.0, 0.0);
        poseEstimator = new PoseEstimator6D(width, height, zNear, zFar, camera, distCoeffs, objects);
    }

    RBOTTestRunner::~RBOTTestRunner()
    {
        for(int i = 0; i < objects.size(); i++)
        {
            delete objects[i];
        }
        objects.clear();
        delete poseEstimator;
    }

    cv::Mat RBOTTestRunner::drawResultOverlay(const cv::Mat& frame)
    {
        // render the models with phong shading
        RenderingEngine::Instance()->setLevel(0);

        std::vector<cv::Point3f> colors;
        colors.emplace_back(cv::Point3f(1.0, 0.5, 0.0));
        RenderingEngine::Instance()->renderShaded(std::vector<Model*>
                (objects.begin(), objects.end()), GL_FILL, colors, true);

        // download the rendering to the CPU
        cv::Mat rendering = RenderingEngine::Instance()->downloadFrame(RenderingEngine::RGB);

        // download the depth buffer to the CPU
        cv::Mat depth = RenderingEngine::Instance()->downloadFrame(RenderingEngine::DEPTH);

        // compose the rendering with the current camera image for demo purposes (can be done more efficiently directly in OpenGL)
        cv::Mat result = frame.clone();
        for(int y = 0; y < frame.rows; y++)
        {
            for(int x = 0; x < frame.cols; x++)
            {
                cv::Vec3b color = rendering.at<cv::Vec3b>(y,x);
                if(depth.at<float>(y,x) != 0.0f)
                {
                    result.at<cv::Vec3b>(y,x)[0] = color[2];
                    result.at<cv::Vec3b>(y,x)[1] = color[1];
                    result.at<cv::Vec3b>(y,x)[2] = color[0];
                }
            }
        }
        return result;
    }

    void RBOTTestRunner::getCamera(const boost::filesystem::path& path)
    {
        cv::Matx44f inputCamera = readCamera(path);
        camera = convertCameraMatrix(inputCamera);
    }

    void RBOTTestRunner::getGroundTruth(const boost::filesystem::path& path)
    {
        std::map<int, Pose> poses = readPoses(path);
        groundTruth = poses[1].pose;
        frameNumber = poses.size();
        convertTransformMatrix(groundTruth);
    }

    void RBOTTestRunner::getVideo(const boost::filesystem::path& filePath)
    {
        using namespace boost::filesystem;
        std::string fileName = filePath.string();
        if (is_directory(filePath))
        {
            fileName += path::preferred_separator;
            fileName += "%04d.png";
        }
        videoCapture = cv::VideoCapture(fileName);
        if (videoCapture.isOpened())
        {
            width = (size_t) videoCapture.get(cv::CAP_PROP_FRAME_WIDTH);
            height = (size_t) videoCapture.get(cv::CAP_PROP_FRAME_HEIGHT);
        }
    }

    void RBOTTestRunner::getObject(const boost::filesystem::path& meshPath)
    {
        std::vector<float> distances = {200.0f, 400.0f, 600.0f};
        float scale = 1;
        float qualityThreshold = 0.65f;
        int radius = 40 * width / 640; // radius of 40 px used in the article for 640-width frames
        objects.push_back(new Object3D(meshPath.string(), groundTruth, scale, qualityThreshold, distances, radius));
    }

    void RBOTTestRunner::writeResult()
    {
        for (int i = resultPoses.size() + 1; i <= frameNumber; ++i)
        {
            std::cout << "cycle iteration" << std::endl;
            Pose poseToWrite;
            poseToWrite.pose = cv::Matx44f();
            resultPoses[i] = poseToWrite;
        }
        writePoses(resultPoses, resultsFile);
        std::cout << "Wrote results " << resultPoses.size() << std::endl;
    }

    void RBOTTestRunner::runTest()
    {
        // move the OpenGL context for offscreen rendering to the current thread, if run in a seperate QT worker thread
        // (unnessary in this example)
        //RenderingEngine::Instance()->getContext()->moveToThread(this);

        // active the OpenGL context for the offscreen rendering engine during pose estimation
        RenderingEngine::Instance()->makeCurrent();
        int timeout = 0;
        bool showHelp = showGui;

        SingleIterationFrameGetter frameGetter = SingleIterationFrameGetter(videoCapture, poseEstimator);
        bool firstFrame = true;
        size_t frameCounter = 1;

        cv::Matx44f bestPose;
        float bestResidual = 0;
        std::cout << "Started reading frames" << std::endl;
        while (true)
        {
            IterationResult iteration = frameGetter.getFrame();
            if (frameCounter == 180)
            {
                std::cout << "frameCounter == 180" << std::endl;
            }
            if (iteration.isNextFrame && !firstFrame)
            {
                convertTransformMatrix(bestPose);
                Pose poseToWrite;
                poseToWrite.pose = bestPose;
                resultPoses[frameCounter] = poseToWrite;
                bestResidual = 0.0f;
                firstFrame = true;
                ++frameCounter;
            }
            if (firstFrame || iteration.residual < bestResidual)
            {
                bestResidual = iteration.residual;
                bestPose = objects[0]->getPose();
            }
            firstFrame = false;
            cv::Mat frame = iteration.frame;
            if (frame.empty())
            {
                break;
            }
            if (showGui)
            {
                cv::Mat result = drawResultOverlay(frame);
                if(showHelp)
                {
                    putText(result, "Press '1' to initialize",
                            cv::Point(150, 250),
                            cv::FONT_HERSHEY_DUPLEX,
                            1.0,
                            cv::Scalar(255, 255, 255),
                            1);
                    putText(result,
                            "or 'c' to quit",
                            cv::Point(205, 285),
                            cv::FONT_HERSHEY_DUPLEX,
                            1.0,
                            cv::Scalar(255, 255, 255),
                            1);
                }
                cv::imshow("result", result);

                int key = cv::waitKey(timeout);
                if (key == (int)'1')
                {
                    frameGetter.runToggleTracking = true;
                    timeout = 1;
                    showHelp = !showHelp;
                    continue;
                }
                if (key == (int)'r')
                {
                    poseEstimator->reset();
                }
                if (key == (int)'c')
                {
                    break;
                }
            }
        }
        std::cout << "Frame counter = " << frameCounter << std::endl;
        // deactivate the offscreen rendering OpenGL context
        RenderingEngine::Instance()->doneCurrent();

        // clean up
        // RenderingEngine::Instance()->destroy(); // seems that instance is destroyed twice. The first time here and the second at PoseEstimator destructor
        writeResult();
    }

    cv::Matx33f RBOTTestRunner::convertCameraMatrix(const cv::Matx44f& matrix4)
    {
        return { matrix4(0, 0), matrix4(0, 1), -matrix4(0, 2),
                 matrix4(1, 0), matrix4(1, 1), -matrix4(1, 2),
                 matrix4(2, 0), matrix4(2, 1), -matrix4(2, 2) };
    }

    void RBOTTestRunner::convertTransformMatrix(cv::Matx44f &openGLMatrix)
    {
        cv::Matx44f transformMatrix = {1.0f,  0.0f,  0.0f, 0.0f,
                                       0.0f, -1.0f,  0.0f, 0.0f,
                                       0.0f,  0.0f, -1.0f, 0.0f,
                                       0.0f,  0.0f,  0.0f, 1.0f};
        openGLMatrix = transformMatrix * openGLMatrix;
    }
}