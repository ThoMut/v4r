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

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <v4r/io/filesystem.h>


using ms = std::chrono::milliseconds;
using get_time = std::chrono::steady_clock ;

typedef pcl::PointXYZRGB PT;

namespace po = boost::program_options;

int
main (int argc, char** argv)
{

#if 0
    pcl::SegmentDifferences<PT> seg;
    seg.setInputCloud(cloud_in);
    seg.setTargetCloud(cloud_target);
    seg.setDistanceThreshold (0.002);
    seg.segment(*cloud_out);

    pcl::IterativeClosestPoint<PT, PT> icp;
    icp.setInputSource(cloud_in);
    icp.setInputTarget(cloud_target);
    icp.setMaximumIterations(1);
    icp.setTransformationEpsilon(1e-3);
    icp.setEuclideanFitnessEpsilon(0.1);

    pcl::PointCloud<PT> Final;
    icp.align(Final);

    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
                 icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;


    pcl::visualization::PCLVisualizer viewer("3D Viewer");
    viewer.addCoordinateSystem(1.0f);
    viewer.addPointCloud(cloud_out, "our point cloud");
    viewer.spin();
#endif
    pcl::PointCloud<PT>::Ptr cloud_in (new pcl::PointCloud<PT>);
    pcl::PointCloud<PT>::Ptr cloud_target (new pcl::PointCloud<PT>);
    pcl::PointCloud<PT>::Ptr cloud_out (new pcl::PointCloud<PT>);

    pcl::io::loadPCDFile<PT>("/home/thomas/DA/shared_docker_host/data/scenes_change/noObj.pcd", *cloud_in);
    pcl::io::loadPCDFile<PT>("/home/thomas/DA/shared_docker_host/data/scenes_change/twoObj.pcd", *cloud_target);

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
    v4r::apps::ObjectRecognizer<PT> recognizer (param);
    recognizer.initialize(to_pass_further);

    std::vector<typename v4r::ObjectHypothesis<PT>::Ptr > verified_hypotheses;
//    pcl::PointCloud<PT>::Ptr cloud_in(new pcl::PointCloud<PT>());
    std::vector<v4r::ObjectHypothesesGroup<PT> > generated_hyp;

    for(int i=13; i<14; ++i)
    {
        if(i==10) {continue;}

        std::string test_file1 = test_file;

        test_file1.append("photo" + std::to_string(i) + ".pcd");
        LOG(INFO) << "Recognizing file " << test_file1;
//        pcl::io::loadPCDFile( test_file1, *cloud_in);

        //reset view point - otherwise this messes up PCL's visualization (this does not affect recognition results)
        cloud_in->sensor_orientation_ = Eigen::Quaternionf::Identity();
        cloud_in->sensor_origin_ = Eigen::Vector4f::Zero(4);

        verified_hypotheses = recognizer.recognize(cloud_in);
        generated_hyp = recognizer.getGeneratedObjectHypothesis();
        std::cout << "size of generated_hyp:" << generated_hyp.size() << std::endl;
        std::cout << "size of generated_hyp ohs_:" << generated_hyp[0].ohs_.size() << std::endl;
        std::cout << "size of generated_hyp ohs_:" << generated_hyp[1].ohs_.size() << std::endl;

        for ( const v4r::ObjectHypothesesGroup<PT> &gohg : generated_hyp )
        {
            for ( const v4r::ObjectHypothesis<PT>::Ptr &goh : gohg.ohs_ )
            {
                std::cout << "generated hyp:" << goh->model_id_ << std::endl;
            }
        }
    }

    //change_detection
    std::cout << "#################### start change detection ################" << std::endl;

    std::string hv_config_xml_ = "cfg/hv_config.xml";
    std::string camera_config_xml_ = "cfg/camera.xml";
    std::string models_dir_ = recognizer.getModelsDir();
    std::vector<v4r::ObjectHypothesesGroup<PT> > hypotheses_input_group;
    std::vector<typename v4r::ObjectHypothesis<PT>::Ptr > hypotheses_output;
    std::vector<typename v4r::ObjectHypothesis<PT>::Ptr > hypotheses_input;
    int normal_computation_method_ = 2; //NormalEstimatorType::PCL_INTEGRAL_NORMAL;

    typename v4r::Camera::Ptr camera_;
    typename v4r::Source<PT>::Ptr model_database_;
    typename v4r::HypothesisVerification<PT, PT>::Ptr hv_;
    typename v4r::NormalEstimator<PT>::Ptr normal_estimator_;

    pcl::ScopeTime t("Generation of object hypotheses");
    auto time_before = t.getTime();

    normal_estimator_ = v4r::initNormalEstimator<PT> ( normal_computation_method_, to_pass_further );

    pcl::PointCloud<pcl::Normal>::Ptr normals;
    normal_estimator_->setInputCloud( cloud_in );
    normals = normal_estimator_->compute();

     std::cout << "normals calculation: " << t.getTime() - time_before << "ms" << std::endl;

    time_before = t.getTime();

    camera_.reset (new v4r::Camera(camera_config_xml_) );

    typename v4r::ObjectHypothesis<PT>::Ptr empty;

    for(unsigned int i=0; i<verified_hypotheses.size(); ++i)
    {
        typename v4r::ObjectHypothesesGroup<PT> hyp_;
        hyp_.ohs_.push_back(empty);
        hyp_.ohs_[0] = verified_hypotheses[i];
        hypotheses_input_group.push_back(hyp_);
    }

 //#################################### Next Scene ##########################################


    typename v4r::HV_Parameter paramHV;
    paramHV.load (hv_config_xml_);
    hv_.reset (new v4r::HypothesisVerification<PT, PT> (camera_, paramHV) );
    model_database_.reset ( new v4r::Source<PT> (models_dir_) );



    std::cout << "################ start hypotheses verification ######################"<< std::endl;

    hv_->setModelDatabase(model_database_);
    hv_->setSceneCloud( cloud_target );
    hv_->setNormals( normals );
    hv_->setHypotheses( hypotheses_input_group );
    hv_->verify();
    hypotheses_output = hv_->getVerifiedHypotheses();

    std::cout << "hypotheses input:" << std::endl;
    for ( const v4r::ObjectHypothesesGroup<PT> &gohg : hypotheses_input_group )
    {
        for ( const v4r::ObjectHypothesis<PT>::Ptr &goh : gohg.ohs_ )
        {
            std::cout << goh->model_id_ << std::endl;
        }
    }

    pcl::PointCloud<PT>::Ptr pRecognizedModels (new pcl::PointCloud<PT>());


    std::cout << "hypotheses output:" << std::endl;
    for ( const v4r::ObjectHypothesis<PT>::Ptr &voh : hypotheses_output )
    {
        std::cout << voh->model_id_ << std::endl;
        typename pcl::PointCloud<PT>::ConstPtr model_cloud = recognizer.getModel( voh->model_id_, 5 );
        typename pcl::PointCloud<PT>::Ptr model_aligned (new pcl::PointCloud<PT>);
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
  pcl::PointCloud<PT>::Ptr cloud_filtered (new pcl::PointCloud<PT>);
  cloud_filtered->width = cloud_in->width;
  cloud_filtered->height = cloud_in->height;

  unsigned int number_points1 = cloud_in->width * cloud_in->height;
  cloud_filtered->points.resize(number_points1);

  int window_size = 6;
  int window=0;

  for(unsigned int i=window_size/2; i<cloud_out->width - window_size/2; ++i)
  {
  for(unsigned int jj=window_size/2; jj<cloud_out->height - window_size/2; ++jj)
  {
      if(cloud_out->at(i,jj).rgb != 0)
      {
          for(int k=-window_size/2; k<=window_size/2; ++k)
          {
          for(int mm=-window_size/2; mm<=window_size/2; ++mm)
          {
              if(cloud_out->at(i+k,jj+mm).rgb != 0)
                  window++;
          }
          }
          if(window == 0)
                jj=jj+window_size/2;

          if(window >= window_size*window_size)
          {
              for(int k=-window_size/2; k<=window_size/2; ++k)
              {
              for(int mm=-window_size/2; mm<=window_size/2; ++mm)
              {
                  cloud_filtered->at(i+k,jj+mm) = cloud_out->at(i+k,jj+mm);
              }
              }
              jj=jj+window_size/2;
          }
          window = 0;
      }
  }
  }

  std::cout << "outlier removal: " << t.getTime() - time_before << "ms" << std::endl;


//radius outlier removal:
//    pcl::RadiusOutlierRemoval<PT> outrem;
//    outrem.setInputCloud(cloud_out);
//    outrem.setRadiusSearch(0.8);
//    outrem.setMinNeighborsInRadius (2);
//    outrem.filter (*cloud_out);

//statistical outlier removal:

//    pcl::StatisticalOutlierRemoval<PT> sor;
//    sor.setInputCloud (cloud_target);
//    sor.setMeanK (50);
//    sor.setStddevMulThresh (1.0);
//    sor.filter (*cloud_target);

//std::cout << "statistical outlier removal: " << t.getTime() - time_before << "ms" << std::endl;

//    std::vector<int> indices;
//    pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, indices);
//    pcl::removeNaNFromPointCloud(*cloud_target, *cloud_target, indices);


//    pcl::SegmentDifferences<PT> seg;
//    seg.setInputCloud(cloud_in);
//    seg.setTargetCloud(cloud_target);
//    seg.setDistanceThreshold (0.0005);
//    seg.segment(*cloud_out);

//    std::cout << "cloud_in->isOrganized(): " << cloud_in->isOrganized() << std::endl;
//    std::cout << "cloud_target->isOrganized(): " << cloud_target->isOrganized() << std::endl;
//    std::cout << "cloud_out->isOrganized(): " << cloud_out->isOrganized() << std::endl;

    time_before = t.getTime();
    verified_hypotheses = recognizer.recognize(cloud_out);
    verified_hypotheses.insert(verified_hypotheses.end(), hypotheses_output.begin(), hypotheses_output.end());
    std::cout << "recognizer mit reduzierter szene: " << t.getTime() - time_before << "ms" << std::endl;

    time_before = t.getTime();
    recognizer.recognize(cloud_filtered);
    std::cout << "recognizer mit gefilterter szene: " << t.getTime() - time_before << "ms" << std::endl;


    std::cout << "final result:" << std::endl;
    for ( const v4r::ObjectHypothesis<PT>::Ptr &voh : verified_hypotheses )
    {
        std::cout << voh->model_id_ << std::endl;
    }

    pcl::visualization::PCLVisualizer viewer("3D Viewer");
    viewer.addCoordinateSystem(1.0f);
    viewer.addPointCloud(cloud_filtered, "our point cloud");
    viewer.spin();

    return (0);

    //Lock at this:

//    pcl::FilterIndices< PT > Class Template Referenceabstract
//    Module filters


//    #include <pcl/filters/filter_indices.h>
}
