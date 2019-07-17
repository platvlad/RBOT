#include "argparsing.h"
#include "OptProcessor.h"

namespace testrunner {

    boost::optional <TrackingConfig> parseTrackingArguments(
            int argc, char *argv[]) {
        using boost::program_options::value;
        using boost::program_options::bool_switch;
        using boost::filesystem::path;
        using boost::optional;

        TrackingConfig config;
        utils::OptProcessor optProcessor;
        optProcessor.addOptions()
                ("results,o", value<path>(&config.results)->required(), "Results destination path.")
                ("camera,c", value<path>(&config.camera)->required(), "Camera description path.")
                ("ground-truth,g", value<path>(&config.groundTruth)->required(), "Ground truth poses path.")
                ("mesh,m", value<path>(&config.mesh)->required(), "Object model path.")
                ("precalc,p", value<path>(&config.precalc)->required(), "Analysis file path.")
                ("show_gui,s", bool_switch(&config.showGui), "Show tracking results")
                ("rgb,v", value < optional < path > > (&config.rgb), "Input video path (optional).");

        utils::OptProcessor::Result const res = optProcessor.process(argc, argv);
        if (res.shutdown) {
            return boost::none;
        }
        return config;
    }
}