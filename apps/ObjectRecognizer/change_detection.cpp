#include <boost/serialization/vector.hpp>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>

#include <pcl/segmentation/segment_differences.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <chrono>

#include <v4r/recognition/hypotheses_verification.h>

#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <glog/logging.h>
#include <v4r/apps/ObjectRecognizer.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <v4r/io/filesystem.h>
#include <string>

#include <v4r/common/normals.h>
#include <v4r/core/macros.h>
#include <v4r/common/camera.h>
#include <v4r/common/miscellaneous.h>
#include <v4r/common/normals.h>

#include <v4r/io/filesystem.h>

#include <v4r/apps/change_detection.h>



typedef pcl::PointXYZRGB PointT;

namespace po = boost::program_options;

int
main (int argc, char** argv)
{

#if 0
    pcl::SegmentDifferences<PointT> seg;
    seg.setInputCloud(cloud_in);
    seg.setTargetCloud(cloud_target);
    seg.setDistanceThreshold (0.002);
    seg.segment(*cloud_out);

    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(cloud_in);
    icp.setInputTarget(cloud_target);
    icp.setMaximumIterations(1);
    icp.setTransformationEpsilon(1e-3);
    icp.setEuclideanFitnessEpsilon(0.1);

    pcl::PointCloud<PointT> Final;
    icp.align(Final);

    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
                 icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;


    pcl::visualization::PCLVisualizer viewer("3D Viewer");
    viewer.addCoordinateSystem(1.0f);
    viewer.addPointCloud(cloud_out, "our point cloud");
    viewer.spin();
#endif
    pcl::PointCloud<PointT>::Ptr cloud_in (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_target (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_out (new pcl::PointCloud<PointT>);

    pcl::io::loadPCDFile<PointT>("/home/thomas/DA/shared_docker_host/data/scenes_change/oneObj.pcd", *cloud_in);
    pcl::io::loadPCDFile<PointT>("/home/thomas/DA/shared_docker_host/data/scenes_change/twoObj.pcd", *cloud_target);


    std::string test_file;
    std::string recognizer_config = "cfg/multipipeline_config.xml";

    po::options_description desc("Single-View Object Instance Recognizer\n======================================\n**Allowed options");
    desc.add_options()
            ("help,h", "produce help message")
            ("test_folder,t", po::value<std::string>(&test_file)->required(), "Directory with test scenes stored as point clouds (.pcd). The camera pose is taken directly from the pcd header fields \"sensor_orientation_\" and \"sensor_origin_\" (if the test directory contains subdirectories, each subdirectory is considered as seperate sequence for multiview recognition)")
            ("recognizer_config", po::value<std::string>(&recognizer_config)->default_value(recognizer_config), "Config XML of the multi-pipeline recognizer")
            ;
    po::variables_map vm;
    po::parsed_options parsed = po::command_line_parser(argc, argv).options(desc).allow_unregistered().run();
    std::vector<std::string> to_pass_further = po::collect_unrecognized(parsed.options, po::include_positional);
    po::store(parsed, vm);
    if (vm.count("help")) { std::cout << desc << std::endl; to_pass_further.push_back("-h"); }
    try { po::notify(vm); }
    catch(std::exception& e) { std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl;  }

    v4r::apps::ObjectRecognizerParameter recognizer_param(recognizer_config);
    boost::shared_ptr<v4r::apps::ObjectRecognizer<PointT> > recognizer(new v4r::apps::ObjectRecognizer<PointT>(recognizer_param));

    recognizer->initialize(to_pass_further);

    std::vector<typename v4r::ObjectHypothesis<PointT>::Ptr > verified_hypotheses_from_old_scene;
    std::vector<v4r::ObjectHypothesesGroup<PointT> > generated_hyp;

    //reset view point - otherwise this messes up PCL's visualization (this does not affect recognition results)
    cloud_in->sensor_orientation_ = Eigen::Quaternionf::Identity();
    cloud_in->sensor_origin_ = Eigen::Vector4f::Zero(4);

    verified_hypotheses_from_old_scene = recognizer->recognize(cloud_in);
    generated_hyp = recognizer->getGeneratedObjectHypothesis();
    std::cout << "size of generated_hyp:" << generated_hyp.size() << std::endl;
    std::cout << "size of generated_hyp ohs_:" << generated_hyp[0].ohs_.size() << std::endl;
    std::cout << "size of generated_hyp ohs_:" << generated_hyp[1].ohs_.size() << std::endl;

    for ( const v4r::ObjectHypothesesGroup<PointT> &gohg : generated_hyp )
    {
        for ( const v4r::ObjectHypothesis<PointT>::Ptr &goh : gohg.ohs_ )
        {
            std::cout << "generated hyp:" << goh->model_id_ << std::endl;
        }
    }

    std::vector<typename v4r::ObjectHypothesis<PointT>::Ptr > verified_hypotheses_final;

    v4r::apps::ChangeDetector<PointT> detector;
    detector.init(recognizer_param, recognizer);
    verified_hypotheses_final = detector.hypotheses_verification(verified_hypotheses_from_old_scene, cloud_in, cloud_target);

    return (0);
}
