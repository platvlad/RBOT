/**
 *   #, #,         CCCCCC  VV    VV MM      MM RRRRRRR
 *  %  %(  #%%#   CC    CC VV    VV MMM    MMM RR    RR
 *  %    %## #    CC        V    V  MM M  M MM RR    RR
 *   ,%      %    CC        VV  VV  MM  MM  MM RRRRRR
 *   (%      %,   CC    CC   VVVV   MM      MM RR   RR
 *     #%    %*    CCCCCC     VV    MM      MM RR    RR
 *    .%    %/
 *       (%.      Computer Vision & Mixed Reality Group
 *                For more information see <http://cvmr.info>
 *
 * This file is part of RBOT.
 *
 *  @copyright:   RheinMain University of Applied Sciences
 *                Wiesbaden Rüsselsheim
 *                Germany
 *     @author:   Henning Tjaden
 *                <henning dot tjaden at gmail dot com>
 *    @version:   1.0
 *       @date:   30.08.2018
 *
 * RBOT is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RBOT is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with RBOT. If not, see <http://www.gnu.org/licenses/>.
 */
#include <fstream>
#include <cstring>

#include <QApplication>
#include <QThread>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <yaml-cpp/yaml.h>

#include "object3d.h"
#include "pose_estimator6d.h"
#include "argparsing.h"
#include "RBOTTestRunner.h"


using namespace std;
using namespace cv;

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        cout << "Wrong parameters" << endl;
        return 1;
    }
    if (strncmp(argv[1], "tracking", 10) != 0)
    {
        cout << "Wrong parameters" << endl;
        return 0;
    }
    QApplication a(argc, argv);
    boost::optional<testrunner::TrackingConfig> config = testrunner::parseTrackingArguments(argc, argv);
    if (config)
    {
        testrunner::RBOTTestRunner testRunner = testrunner::RBOTTestRunner(config.value());
        testRunner.runTest();
    }
    else
    {
        cout << "Wrong parameters" << endl;
        return 1;
    }

}
