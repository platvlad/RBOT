#include <iostream>
#include <fstream>

#include <boost/filesystem/operations.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/mesh.h>
#include <assimp/postprocess.h>

#include "RBOTTestRunner.h"
#include "SingleIterationFrameGetter.h"
#include "io.hpp"
#include "StrictFrameGetter.h"

namespace testrunner
{
    RBOTTestRunner::RBOTTestRunner(TrackingConfig& config) : resultsFile(config.results),
                                                             showGui(config.showGui)
    {
        getGroundTruth(config.groundTruth);
        getMesh(config.mesh);
        if (config.rgb)
        {
            getVideo(config.rgb.value());
        }
        getCamera(config.camera);

        cv::Vec3f t = cv::Vec3f(groundTruth(0, 3), groundTruth(1, 3), groundTruth(2, 3));
        float distanceToObject = cv::norm(t);
        zNear = distanceToObject * 0.01;
        zFar = distanceToObject * 100;
        distances = {0.5f * distanceToObject, distanceToObject, 1.5f * distanceToObject};
        // distances = {0.0f, 0.0f, 0.0f};

        logFileName = resultsFile.parent_path().string() +
                      boost::filesystem::path::preferred_separator +
                      "residuals" +
                      std::to_string(config.iteration_factor) + ".yml";

        getObject(config.mesh);

        cv::Matx14f distCoeffs =  cv::Matx14f(0.0, 0.0, 0.0, 0.0);
        poseEstimator = new PoseEstimator6D(width, height, zNear, zFar, camera, distCoeffs, objects, config.iteration_factor);

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
        colors.emplace_back(cv::Point3f(0.5, 1.0, 0.0));
        RenderingEngine::Instance()->renderShaded(std::vector<Model*>
                (objects.begin(), objects.end()), GL_LINE, colors, true);

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
        std::cout << "camera = " << camera << std::endl;
    }

    void RBOTTestRunner::getGroundTruth(const boost::filesystem::path& path)
    {
        std::map<int, Pose> poses = readPoses(path);
        groundTruth = poses[1].pose;
        frameNumber = poses.size();
        convertTransformMatrix(groundTruth);
    }

    void RBOTTestRunner::getMesh(const boost::filesystem::path& path)
    {
        Assimp::Importer importer;
        const aiScene* scene = importer.ReadFile(path.string(), aiProcessPreset_TargetRealtime_Fast);
        aiMesh** meshes = scene->mMeshes;
        cv::Vec3f bbCenter = boundingBoxCenter(meshes, scene->mNumMeshes);
        T_n = Transformations::scaleMatrix(scaling)*Transformations::translationMatrix(-bbCenter[0], -bbCenter[1], -bbCenter[2]);
        T_n_inv = T_n.inv();
        groundTruth = groundTruth * T_n_inv;
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


    void RBOTTestRunner::getObject(const boost::filesystem::path& path)
    {
        // distances = {200.0f, 400.0f, 600.0f};
        float qualityThreshold = 0.55f;
        int radius = 40 * width / 640; // radius of 40 px used in the article for 640-width frames

        objects.push_back(new Object3D(path.string(), groundTruth, scaling, qualityThreshold, distances, radius));
    }


    void RBOTTestRunner::writeResult()
    {
        for (int i = resultPoses.size() + 1; i <= frameNumber; ++i)
        {
            Pose poseToWrite;
            poseToWrite.pose = cv::Matx44f();
            resultPoses[i] = poseToWrite;
        }
        writePoses(resultPoses, resultsFile);
        boost::filesystem::path logFile = boost::filesystem::path(logFileName);
        writeErrors(resultResiduals, logFile);
        boost::filesystem::path resultDir = resultsFile.parent_path();
        std::string timesFileName = resultDir.string() + boost::filesystem::path::preferred_separator + "times.yml";
        boost::filesystem::path timesFile = boost::filesystem::path(timesFileName);
        writeTimes(resultTimes, timesFile);
        std::string videoFileName = resultDir.string() + boost::filesystem::path::preferred_separator + "output.mov";
        boost::filesystem::path videoFilePath(videoFileName);
        boost::filesystem::remove(videoFilePath);
        cv::VideoWriter outputVideo;
        cv::Size frameSize(width, height);
        outputVideo.open(videoFileName, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 25, frameSize);
        if (outputVideo.isOpened())
        {
            for (int i = 0; i < outputFrames.size(); ++i)
            {
                outputVideo << outputFrames[i];
            }
        }
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
        //StrictFrameGetter frameGetter = StrictFrameGetter(videoCapture, poseEstimator, 10);
        bool firstFrame = true;
        size_t frameCounter = 1;

        cv::Matx44f bestPose;
        float bestResidual = 0;
        std::cout << "Started reading frames" << std::endl;
        std::ofstream fout(logFileName);
        while (true)
        {
            auto start = std::chrono::steady_clock::now();
            IterationResult iteration = frameGetter.getFrame();
            auto end = std::chrono::steady_clock::now();
            std::chrono::duration<double> timeDiff = end - start;
            if (iteration.isNextFrame && !firstFrame)
            {
                convertTransformMatrix(bestPose);
                Pose poseToWrite;
                poseToWrite.pose = bestPose * T_n;
                resultPoses[frameCounter] = poseToWrite;
                resultResiduals[frameCounter] = bestResidual;
                resultTimes[frameCounter] = timeDiff.count();
                //fout << bestResidual << std::endl;
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
            cv::Mat result = drawResultOverlay(frame);
            cv::Mat sum = (0.5 * result + 0.5 * frame);

            if (iteration.isNextFrame)
            {
                outputFrames.push_back(sum);
            }
            if (showGui)
            {


                if(showHelp)
                {
                    putText(sum, "Press '1' to initialize",
                            cv::Point(150, 250),
                            cv::FONT_HERSHEY_DUPLEX,
                            1.0,
                            cv::Scalar(255, 255, 255),
                            1);
                    putText(sum,
                            "or 'c' to quit",
                            cv::Point(205, 285),
                            cv::FONT_HERSHEY_DUPLEX,
                            1.0,
                            cv::Scalar(255, 255, 255),
                            1);
                }
                cv::imshow("result", sum);

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
            else if (frameCounter == 1)
            {
                frameGetter.runToggleTracking = true;
            }
        }
        std::cout << "Frame counter = " << frameCounter << std::endl;
        // deactivate the offscreen rendering OpenGL context
        RenderingEngine::Instance()->doneCurrent();

        // clean up
        // RenderingEngine::Instance()->destroy(); // seems that instance is destroyed twice. The first time here and the second at PoseEstimator destructor
        writeResult();
        fout.close();
    }



    cv::Vec3f RBOTTestRunner::boundingBoxCenter(aiMesh** meshes, size_t meshNumber)
    {
        float inf = std::numeric_limits<float>::infinity();
        cv::Vec3f lbn = cv::Vec3f(inf, inf, inf);
        cv::Vec3f rtf = cv::Vec3f(-inf, -inf, -inf);
        for (int i = 0; i < meshNumber; ++i)
        {
            aiMesh* mesh = meshes[i];
            for (int j = 0; j < mesh->mNumVertices; j++)
            {
                aiVector3D v = mesh->mVertices[j];

                cv::Vec3f p(v.x, v.y, v.z);

                // compute the 3D bounding box of the model
                if (p[0] < lbn[0]) lbn[0] = p[0];
                if (p[1] < lbn[1]) lbn[1] = p[1];
                if (p[2] < lbn[2]) lbn[2] = p[2];
                if (p[0] > rtf[0]) rtf[0] = p[0];
                if (p[1] > rtf[1]) rtf[1] = p[1];
                if (p[2] > rtf[2]) rtf[2] = p[2];

            }
        }

        cv::Vec3f bbCenter = (rtf + lbn)/2;
        return bbCenter;
    }


    cv::Matx33f RBOTTestRunner::convertCameraMatrix(const cv::Matx44f& matrix4)
    {
        return { matrix4(0, 0), matrix4(0, 1), -matrix4(0, 2),
                 matrix4(1, 0), matrix4(1, 1), height - (-matrix4(1, 2)),
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
