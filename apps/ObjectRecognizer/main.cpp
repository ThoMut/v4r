
#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <glog/logging.h>

#include <v4r/apps/ObjectRecognizer.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <v4r/io/filesystem.h>

namespace po = boost::program_options;

int
main (int argc, char ** argv)
{
    typedef pcl::PointXYZRGB PT;

    std::string test_file;
    std::string recognizer_config = "cfg/multipipeline_config.xml";
    int verbosity = -1;

    po::options_description desc("Single-View Object Instance Recognizer\n======================================\n**Allowed options");
    desc.add_options()
            ("help,h", "produce help message")
            ("test_file,t", po::value<std::string>(&test_file)->required(), "Directory with test scenes stored as point clouds (.pcd). The camera pose is taken directly from the pcd header fields \"sensor_orientation_\" and \"sensor_origin_\" (if the test directory contains subdirectories, each subdirectory is considered as seperate sequence for multiview recognition)")
            ("recognizer_config", po::value<std::string>(&recognizer_config)->default_value(recognizer_config), "Config XML of the multi-pipeline recognizer")
            ("verbosity", po::value<int>(&verbosity)->default_value(verbosity), "set verbosity level for output (<0 minimal output)")
            ;
    po::variables_map vm;
    po::parsed_options parsed = po::command_line_parser(argc, argv).options(desc).allow_unregistered().run();
    std::vector<std::string> to_pass_further = po::collect_unrecognized(parsed.options, po::include_positional);
    po::store(parsed, vm);
    if (vm.count("help")) { std::cout << desc << std::endl; to_pass_further.push_back("-h"); }
    try { po::notify(vm); }
    catch(std::exception& e) { std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl;  }

    v4r::apps::ObjectRecognizerParameter param(recognizer_config);
    v4r::apps::ObjectRecognizer<PT> recognizer (param);
    recognizer.initialize(to_pass_further);

    LOG(INFO) << "Recognizing file " << test_file;
    pcl::PointCloud<PT>::Ptr cloud(new pcl::PointCloud<PT>());
    pcl::io::loadPCDFile( test_file, *cloud);

    //reset view point - otherwise this messes up PCL's visualization (this does not affect recognition results)
    cloud->sensor_orientation_ = Eigen::Quaternionf::Identity();
    cloud->sensor_origin_ = Eigen::Vector4f::Zero(4);

    std::vector<typename v4r::ObjectHypothesis<PT>::Ptr > verified_hypotheses = recognizer.recognize(cloud);

    for ( const v4r::ObjectHypothesis<PT>::Ptr &voh : verified_hypotheses )
    {
        std::cout << voh->model_id_ << std::endl;
    }
}

