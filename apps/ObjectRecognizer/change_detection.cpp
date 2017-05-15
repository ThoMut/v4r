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

using ms = std::chrono::milliseconds;
using get_time = std::chrono::steady_clock ;

typedef pcl::PointXYZRGB PoinT;

namespace po = boost::program_options;

int
main (int argc, char** argv)
{

#if 0
    pcl::SegmentDifferences<PoinT> seg;
    seg.setInputCloud(cloud_in);
    seg.setTargetCloud(cloud_target);
    seg.setDistanceThreshold (0.002);
    seg.segment(*cloud_out);

    pcl::IterativeClosestPoint<PoinT, PoinT> icp;
    icp.setInputSource(cloud_in);
    icp.setInputTarget(cloud_target);
    icp.setMaximumIterations(1);
    icp.setTransformationEpsilon(1e-3);
    icp.setEuclideanFitnessEpsilon(0.1);

    pcl::PointCloud<PoinT> Final;
    icp.align(Final);

    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
                 icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;


    pcl::visualization::PCLVisualizer viewer("3D Viewer");
    viewer.addCoordinateSystem(1.0f);
    viewer.addPointCloud(cloud_out, "our point cloud");
    viewer.spin();
#endif
    pcl::PointCloud<PoinT>::Ptr cloud_in (new pcl::PointCloud<PoinT>);
    pcl::PointCloud<PoinT>::Ptr cloud_target (new pcl::PointCloud<PoinT>);
    pcl::PointCloud<PoinT>::Ptr cloud_out (new pcl::PointCloud<PoinT>);

    pcl::io::loadPCDFile<PoinT>("/home/thomas/DA/shared_docker_host/data/scenes_change/noObj.pcd", *cloud_in);
    pcl::io::loadPCDFile<PoinT>("/home/thomas/DA/shared_docker_host/data/scenes_change/twoObj.pcd", *cloud_target);

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

    v4r::apps::ObjectRecognizerParameter param(recognizer_config);
    v4r::apps::ObjectRecognizer<PoinT> recognizer (param);
    recognizer.initialize(to_pass_further);

    std::vector<typename v4r::ObjectHypothesis<PoinT>::Ptr > verified_hypotheses;
//    pcl::PointCloud<PoinT>::Ptr cloud_in(new pcl::PointCloud<PoinT>());
    std::vector<v4r::ObjectHypothesesGroup<PoinT> > generated_hyp;

    //reset view point - otherwise this messes up PCL's visualization (this does not affect recognition results)
    cloud_in->sensor_orientation_ = Eigen::Quaternionf::Identity();
    cloud_in->sensor_origin_ = Eigen::Vector4f::Zero(4);

    verified_hypotheses = recognizer.recognize(cloud_in);
    generated_hyp = recognizer.getGeneratedObjectHypothesis();
    std::cout << "size of generated_hyp:" << generated_hyp.size() << std::endl;
    std::cout << "size of generated_hyp ohs_:" << generated_hyp[0].ohs_.size() << std::endl;
    std::cout << "size of generated_hyp ohs_:" << generated_hyp[1].ohs_.size() << std::endl;

    for ( const v4r::ObjectHypothesesGroup<PoinT> &gohg : generated_hyp )
    {
        for ( const v4r::ObjectHypothesis<PoinT>::Ptr &goh : gohg.ohs_ )
        {
            std::cout << "generated hyp:" << goh->model_id_ << std::endl;
        }
    }

    /// change_detection
    std::cout << "#################### start change detection ################" << std::endl;

    std::vector<v4r::ObjectHypothesesGroup<PoinT> > hypotheses_input_group;
    std::vector<typename v4r::ObjectHypothesis<PoinT>::Ptr > hypotheses_output;

#if 0
 //normal calculation not needed because used from recognizer class
    std::string hv_config_xml_ = "cfg/hv_config.xml";
    std::string camera_config_xml_ = "cfg/camera.xml";
    std::string models_dir_ = recognizer.getModelsDir();

    typename v4r::Camera::Ptr camera_;
    typename v4r::Source<PoinT>::Ptr model_database_;
#endif
 //calculate normals
    pcl::ScopeTime t("Generation of object hypotheses");
    auto time_before = t.getTime();

    int normal_computation_method_ = 2; //NormalEstimatorType::PCL_INTEGRAL_NORMAL;
    typename v4r::NormalEstimator<PoinT>::Ptr normal_estimator_;
    normal_estimator_ = v4r::initNormalEstimator<PoinT> ( normal_computation_method_, to_pass_further );

    pcl::PointCloud<pcl::Normal>::Ptr normals;
    normal_estimator_->setInputCloud( cloud_in );
    normals = normal_estimator_->compute();

     std::cout << "normals calculation: " << t.getTime() - time_before << "ms" << std::endl;
    time_before = t.getTime();

    //#################################### Next Scene ##########################################

#if 0
    camera_.reset (new v4r::Camera(camera_config_xml_) );


    typename v4r::HV_Parameter paramHV;
    paramHV.load (hv_config_xml_);
    hv_.reset (new v4r::HypothesisVerification<PoinT, PoinT> (camera_, paramHV) );
    model_database_.reset ( new v4r::Source<PoinT> (models_dir_) );

#endif
    //hypotheses of last recognition redefine as new input for hypotheses verification
    typename v4r::ObjectHypothesis<PoinT>::Ptr empty;
    for(unsigned int i=0; i<verified_hypotheses.size(); ++i)
    {
        typename v4r::ObjectHypothesesGroup<PoinT> hyp_;
        hyp_.ohs_.push_back(empty);
        hyp_.ohs_[0] = verified_hypotheses[i];
        hypotheses_input_group.push_back(hyp_);
    }

std::cout << "################ start hypotheses verification ######################"<< std::endl;
    typename v4r::HypothesisVerification<PoinT, PoinT>::Ptr hv_;
    hv_ = recognizer.getHypothesesVerification();

    //hv_->setModelDatabase(model_database_);
    hv_->setSceneCloud( cloud_target );
    hv_->setNormals( normals );
    hv_->setHypotheses( hypotheses_input_group );
    hv_->verify();
    hypotheses_output = hv_->getVerifiedHypotheses();

    std::cout << "hypotheses input:" << std::endl;
    for ( const v4r::ObjectHypothesesGroup<PoinT> &gohg : hypotheses_input_group )
    {
        for ( const v4r::ObjectHypothesis<PoinT>::Ptr &goh : gohg.ohs_ )
        {
            std::cout << goh->model_id_ << std::endl;
        }
    }

    pcl::PointCloud<PoinT>::Ptr pRecognizedModels (new pcl::PointCloud<PoinT>());

    std::cout << "hypotheses output:" << std::endl;
    for ( const v4r::ObjectHypothesis<PoinT>::Ptr &voh : hypotheses_output )
    {
        std::cout << voh->model_id_ << std::endl;
        typename pcl::PointCloud<PoinT>::ConstPtr model_cloud = recognizer.getModel( voh->model_id_, 5 );
        typename pcl::PointCloud<PoinT>::Ptr model_aligned (new pcl::PointCloud<PoinT>);
        pcl::transformPointCloud (*model_cloud, *model_aligned, voh->transform_);
        *pRecognizedModels += *model_aligned;
    }

    std::cout << "hypothesis verification: " << t.getTime() - time_before << "ms" << std::endl;

//    pcl::io::savePCDFileASCII ("cloud_in_ascii.pcd", *cloud_in);
//    pcl::io::savePCDFileASCII ("cloud_target_ascii.pcd", *cloud_target);

    time_before = t.getTime();

// nur tiefendaten vergleichen:
    cloud_out->width = cloud_in->width;
    cloud_out->height = cloud_in->height;

    unsigned int number_points = cloud_in->width * cloud_in->height;
    cloud_out->points.resize(number_points);

    float z_target;
    for(unsigned int i=0; i<number_points; ++i)
    {
        z_target = cloud_target->points[i].z;
        if(std::fabs(z_target - cloud_in->points[i].z) >= 0.02*z_target*z_target)
        {
            cloud_out->points[i] = cloud_target->points[i];
        }
    }

    std::cout << "tiefendaten vergleichen: " << t.getTime() - time_before << "ms" << std::endl;

    //pcl::io::savePCDFileASCII ("cloud_out_ascii.pcd", *cloud_out);

    time_before = t.getTime();
//self implemented outlier removal

    //moving a window over the organized point cloud, only patches size of window_size*window_size are accepted
  pcl::PointCloud<PoinT>::Ptr cloud_filtered (new pcl::PointCloud<PoinT>);
  cloud_filtered->width = cloud_in->width;
  cloud_filtered->height = cloud_in->height;

  unsigned int number_points1 = cloud_in->width * cloud_in->height;
  cloud_filtered->points.resize(number_points1);

  int window_size = 5;
  //moving window over organized point cloud
  for(unsigned int i=0; i<cloud_out->width - window_size; ++i)
  {
  for(unsigned int jj=0; jj<cloud_out->height - window_size; ++jj)
  {
      //if data of actual point is != 0 the whole patch (window_size*window_size) is checkt
      if(cloud_out->at(i,jj).rgb != 0)
      {
          bool store = true;
          for(int k=0; k<=window_size; ++k)
          {
          for(int mm=0; mm<=window_size; ++mm)
          {
              if(cloud_out->at(i+k,jj+mm).rgb == 0)
              {
                  k =window_size+1;
                  mm= window_size+1;
                  store = false;
              }
          }
          }

          //if the whole patch is filled with data, it gets accepted
          if(store == true)
          {
              for(int k=0; k<=window_size; ++k)
              {
              for(int mm=0; mm<=window_size; ++mm)
              {
                  cloud_filtered->at(i+k,jj+mm) = cloud_out->at(i+k,jj+mm);
              }
              }
              jj=jj+window_size;
          }
      }
  }
  }

  std::cout << "outlier removal: " << t.getTime() - time_before << "ms" << std::endl;

//    std::vector<int> indices;
//    pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, indices);
//    pcl::removeNaNFromPointCloud(*cloud_target, *cloud_target, indices);

    time_before = t.getTime();
    verified_hypotheses = recognizer.recognize(cloud_filtered);
    verified_hypotheses.insert(verified_hypotheses.end(), hypotheses_output.begin(), hypotheses_output.end());
    std::cout << "recognizer mit gefilterten szene: " << t.getTime() - time_before << "ms" << std::endl;

    time_before = t.getTime();
    recognizer.recognize(cloud_out);
    std::cout << "recognizer mit reduzierter szene: " << t.getTime() - time_before << "ms" << std::endl;


    std::cout << "final result:" << std::endl;
    for ( const v4r::ObjectHypothesis<PoinT>::Ptr &voh : verified_hypotheses )
    {
        std::cout << voh->model_id_ << std::endl;
    }

    pcl::visualization::PCLVisualizer viewer("3D Viewer");
    viewer.addCoordinateSystem(1.0f);
    viewer.addPointCloud(cloud_filtered, "our point cloud");
    viewer.spin();

    return (0);
}
