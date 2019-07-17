#ifndef RBOT_ARGPARSING_H
#define RBOT_ARGPARSING_H
#include <boost/optional.hpp>
#include <boost/filesystem/path.hpp>
namespace testrunner
{

    struct TrackingConfig {
        boost::filesystem::path results;
        boost::filesystem::path camera;
        boost::filesystem::path groundTruth;
        boost::filesystem::path mesh;
        boost::filesystem::path precalc;
        boost::optional <boost::filesystem::path> rgb;
        bool showGui;
    };

    boost::optional <TrackingConfig> parseTrackingArguments(
            int argc, char *argv[]);
}
#endif //RBOT_ARGPARSING_H
