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
                ("experimental,e", bool_switch(&config.experimental), "Experimental tracking.")
                ("back-tracking,b", bool_switch(&config.backTracking), "Back tracking (ignored in refine).")
                ("to-frame,t", bool_switch(&config.toFrame), "Tracking to frame (ignored in refine).")
                ("fixations,f", value<optional<path> >(&config.fixations), "DoF fixations description path (optional).")
                ("show_gui,s", bool_switch(&config.showGui), "Show tracking results")
                ("rgb,v", value < optional < path > > (&config.rgb), "Input video path (optional).")
                ("iteration-factor,i", value <int> (&config.iteration_factor)->default_value(1), "optimization iterations (optional).");

        utils::OptProcessor::Result const res = optProcessor.process(argc, argv);
        if (res.shutdown) {
            return boost::none;
        }
        return config;
    }
}