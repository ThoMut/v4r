#include <v4r/apps/ObjectRecognizer.h>

#include <iostream>
#include <sstream>

#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <glog/logging.h>

#include <pcl/common/time.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/impl/instantiate.hpp>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/registration/icp.h>

#include <v4r/common/camera.h>
#include <v4r/common/miscellaneous.h>
#include <v4r/common/normals.h>
#include <v4r/common/graph_geometric_consistency.h>
#include <v4r/common/pcl_visualization_utils.h>
#include <v4r/features/esf_estimator.h>
#include <v4r/features/global_simple_shape_estimator.h>
#include <v4r/features/global_concatenated.h>
#include <v4r/features/shot_local_estimator.h>
#include <v4r/features/sift_local_estimator.h>
#include <v4r/features/rops_local_estimator.h>
#include <v4r/keypoints/all_headers.h>
#include <v4r/io/filesystem.h>
#include <v4r/ml/all_headers.h>
#include <v4r/recognition/hypotheses_verification.h>
#include <v4r/recognition/global_recognition_pipeline.h>
#include <v4r/segmentation/all_headers.h>


#include <v4r/segmentation/plane_utils.h>
#include <v4r/segmentation/segmentation_utils.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>

#include <sys/time.h>
#include <sys/resource.h>

namespace po = boost::program_options;

namespace v4r
{

namespace apps
{

template<typename PointT>
void
ObjectRecognizer<PointT>::refinePose( const typename pcl::PointCloud<PointT>::ConstPtr &scene)
{
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputTarget(scene);
    icp.setMaximumIterations(param_.icp_iterations_);
    icp.setMaxCorrespondenceDistance( 0.05f );
//    icp.setSearchMethodTarget(kdtree_scene_, true);

    generated_object_hypotheses_refined_.clear();
    generated_object_hypotheses_refined_.resize( generated_object_hypotheses_.size() );

//    static pcl::visualization::PCLVisualizer vis;

    for(size_t ohg_id=0; ohg_id<generated_object_hypotheses_.size(); ohg_id++)
    {
        ObjectHypothesesGroup<PointT> &ohg_refined = generated_object_hypotheses_refined_[ohg_id];
        ohg_refined.global_hypotheses_ = generated_object_hypotheses_[ohg_id].global_hypotheses_;
        ohg_refined.ohs_.reserve( generated_object_hypotheses_[ohg_id].ohs_.size() );

        for(size_t i=0; i<generated_object_hypotheses_[ohg_id].ohs_.size(); i++)
        {
            pcl::ScopeTime t("ICP");
            typename ObjectHypothesis<PointT>::Ptr oh (new ObjectHypothesis<PointT>(*(generated_object_hypotheses_[ohg_id].ohs_[i])));
            bool found;
            typename Model<PointT>::ConstPtr m = model_database_->getModelById(oh->class_id_, oh->model_id_, found);
            typename pcl::PointCloud<PointT>::Ptr model_aligned ( new pcl::PointCloud<PointT>() );
            typename pcl::PointCloud<PointT>::ConstPtr model_cloud = m->getAssembled(3);
            pcl::transformPointCloud( *model_cloud, *model_aligned, oh->transform_);

            icp.setInputSource( model_aligned );
            pcl::PointCloud<PointT> aligned_visible_model;
            icp.align(aligned_visible_model);

            Eigen::Matrix4f refined_tf = Eigen::Matrix4f::Identity();
            if(icp.hasConverged())
                refined_tf = icp.getFinalTransformation();
            else
               std::cout << "ICP did not converge." << std::endl;

            oh->transform_ = refined_tf * oh->transform_;

            ohg_refined.ohs_.push_back(oh);
        }
    }
}

template<typename PointT>
void ObjectRecognizer<PointT>::initialize(const std::vector<std::string> &command_line_arguments)
{
    bool visualize_hv_go_cues = false;
    bool visualize_hv_model_cues = false;
    bool visualize_hv_pairwise_cues = false;
    bool visualize_keypoints = false;
    bool visualize_global_results = false;
    bool retrain = false;

    po::options_description desc("Single-View Object Instance Recognizer\n======================================\n**Allowed options");
    desc.add_options()
            ("help,h", "produce help message")
            ("model_dir,m", po::value<std::string>(&models_dir_)->required(), "Models directory")
            ("visualize,v", po::bool_switch(&visualize_), "visualize recognition results")
            ("skip_verification", po::bool_switch(&skip_verification_), "if true, skips verification (only hypotheses generation)")
            ("hv_vis_cues", po::bool_switch(&visualize_hv_go_cues), "If set, visualizes cues computated at the hypothesis verification stage such as inlier, outlier points. Mainly used for debugging.")
            ("hv_vis_model_cues", po::bool_switch(&visualize_hv_model_cues), "If set, visualizes the model cues. Useful for debugging")
            ("hv_vis_pairwise_cues", po::bool_switch(&visualize_hv_pairwise_cues), "If set, visualizes the pairwise cues. Useful for debugging")
            ("rec_visualize_keypoints", po::bool_switch(&visualize_keypoints), "If set, visualizes detected keypoints.")
            ("rec_visualize_global_pipeline", po::bool_switch(&visualize_global_results), "If set, visualizes segments and results from global pipeline.")
            ("retrain", po::bool_switch(&retrain), "If set, retrains the object models no matter if they already exists.")
            ("recognizer_remove_planes", po::value<bool>(&param_.remove_planes_)->default_value(param_.remove_planes_), "if enabled, removes the dominant plane in the input cloud (given thera are at least N inliers)")
            ("camera_xml", po::value<std::string>(&param_.camera_config_xml_), "camera config file")
            ("sift_config_xml", po::value<std::string>(&param_.sift_config_xml_), "sift config file")
            ("shot_config_xml", po::value<std::string>(&param_.shot_config_xml_), "shot config file")
            ("esf_config_xml", po::value<std::string>(&param_.global_recognition_pipeline_config_[0]), "esf config file")
            ("hv_config_xml", po::value<std::string>(&param_.hv_config_xml_), "hv config file")
            ;
    po::variables_map vm;
    po::parsed_options parsed = po::command_line_parser(command_line_arguments).options(desc).allow_unregistered().run();
    std::vector<std::string> to_pass_further = po::collect_unrecognized(parsed.options, po::include_positional);
    po::store(parsed, vm);
    if (vm.count("help")) { std::cout << desc << std::endl; to_pass_further.push_back("-h"); }
    try { po::notify(vm); }
    catch(std::exception& e) { std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl;  }

    // ====== DEFINE CAMERA =======
    camera_.reset (new Camera(param_.camera_config_xml_) );

    cv::Mat_<uchar> img_mask = cv::imread(param_.depth_img_mask_, CV_LOAD_IMAGE_GRAYSCALE);
    if( img_mask.data )
        camera_->setCameraDepthRegistrationMask( img_mask );
    else
        LOG(WARNING) << "No camera depth registration mask provided. Assuming all pixels have valid depth.";


    // ====== DEFINE VISUALIZATION PARAMETER =======
    PCLVisualizationParams::Ptr vis_param (new PCLVisualizationParams);
    vis_param->no_text_ = false;
    vis_param->bg_color_ = Eigen::Vector3i(255, 255, 255);
    vis_param->text_color_ = Eigen::Vector3f(0.f, 0.f, 0.f);
    vis_param->fontsize_ = 12;
    vis_param->coordinate_axis_scale_ = 0.2f;


    // ==== Fill object model database ==== ( assumes each object is in a seperate folder named after the object and contains and "views" folder with the training views of the object)
    model_database_.reset ( new Source<PointT> (models_dir_) );

    normal_estimator_ = v4r::initNormalEstimator<PointT> ( param_.normal_computation_method_, to_pass_further );


    // ====== SETUP MULTI PIPELINE RECOGNIZER ======
    mrec_.reset( new v4r::MultiRecognitionPipeline<PointT> );
    local_recognition_pipeline_.reset(new LocalRecognitionPipeline<PointT>);
    {
        // ====== SETUP LOCAL RECOGNITION PIPELINE =====
        if(param_.do_sift_ || param_.do_shot_)
        {
            local_recognition_pipeline_->setModelDatabase( model_database_ );

            if(param_.use_graph_based_gc_grouping_)
            {
                GraphGeometricConsistencyGroupingParameter gcparam;
                gcparam.gc_size_ = param_.cg_size_;
                gcparam.gc_threshold_ = param_.cg_thresh_;
                to_pass_further = gcparam.init(to_pass_further);
                GraphGeometricConsistencyGrouping<pcl::PointXYZ, pcl::PointXYZ>::Ptr gc_clusterer
                        (new GraphGeometricConsistencyGrouping<pcl::PointXYZ, pcl::PointXYZ>);
                local_recognition_pipeline_->setCGAlgorithm( gc_clusterer );
            }
            else
            {
                boost::shared_ptr< pcl::GeometricConsistencyGrouping<pcl::PointXYZ, pcl::PointXYZ> > gc_clusterer
                        (new pcl::GeometricConsistencyGrouping<pcl::PointXYZ, pcl::PointXYZ>);
                gc_clusterer->setGCSize( param_.cg_size_ );
                gc_clusterer->setGCThreshold( param_.cg_thresh_ );
                local_recognition_pipeline_->setCGAlgorithm( gc_clusterer );
            }

            if(param_.do_sift_)
            {
                LocalRecognizerParameter sift_param(param_.sift_config_xml_);
                typename LocalFeatureMatcher<PointT>::Ptr sift_rec (new LocalFeatureMatcher<PointT>(sift_param));
                typename SIFTLocalEstimation<PointT>::Ptr sift_est (new SIFTLocalEstimation<PointT>);
                sift_est->setMaxDistance(std::numeric_limits<float>::max());
                sift_rec->addFeatureEstimator( sift_est );
                local_recognition_pipeline_->addLocalFeatureMatcher(sift_rec);
            }
            if(param_.do_shot_)
            {

                LocalRecognizerParameter shot_pipeline_param(param_.shot_config_xml_);
                typename LocalFeatureMatcher<PointT>::Ptr shot_rec (new LocalFeatureMatcher<PointT>(shot_pipeline_param));
                std::vector<typename v4r::KeypointExtractor<PointT>::Ptr > keypoint_extractor = initKeypointExtractors<PointT>( param_.shot_keypoint_extractor_method_, to_pass_further );

                for( typename v4r::KeypointExtractor<PointT>::Ptr ke : keypoint_extractor)
                    shot_rec->addKeypointExtractor( ke );

                for(float support_radius : param_.keypoint_support_radii_)
                {
                    SHOTLocalEstimationParameter shot_param;
                    shot_param.support_radius_ = support_radius;
//                    shot_param.init( to_pass_further );
                    typename SHOTLocalEstimation<PointT>::Ptr shot_est (new SHOTLocalEstimation<PointT> (shot_param) );

    //                ROPSLocalEstimationParameter rops_param;
    //                rops_param.init( to_pass_further );
    //                typename ROPSLocalEstimation<PointT>::Ptr rops_est (new ROPSLocalEstimation<PointT> (rops_param) );


                    shot_rec->addFeatureEstimator( shot_est );
                }
                shot_rec->setVisualizeKeypoints(visualize_keypoints);
                local_recognition_pipeline_->addLocalFeatureMatcher(shot_rec);
            }

            typename RecognitionPipeline<PointT>::Ptr rec_pipeline_tmp = boost::static_pointer_cast<RecognitionPipeline<PointT> > (local_recognition_pipeline_);
            mrec_->addRecognitionPipeline(rec_pipeline_tmp);
        }

        // ====== SETUP GLOBAL RECOGNITION PIPELINE =====

        if( !param_.global_feature_types_.empty() )
        {
            typename GlobalRecognitionPipeline<PointT>::Ptr global_recognition_pipeline (new GlobalRecognitionPipeline<PointT>);
            typename v4r::Segmenter<PointT>::Ptr segmenter = v4r::initSegmenter<PointT>( param_.segmentation_method_, to_pass_further);
            global_recognition_pipeline->setSegmentationAlgorithm( segmenter );

            for(size_t global_pipeline_id = 0; global_pipeline_id < param_.global_feature_types_.size(); global_pipeline_id++)
            {
                    GlobalConcatEstimatorParameter p;
                    p.feature_type = param_.global_feature_types_[global_pipeline_id];
                    typename GlobalConcatEstimator<PointT>::Ptr global_concat_estimator (new GlobalConcatEstimator<PointT>(to_pass_further, p));

//                    typename OURCVFHEstimator<PointT>::Ptr ourcvfh_estimator (new OURCVFHEstimator<PointT>);
                    Classifier::Ptr classifier = initClassifier( param_.classification_methods_[global_pipeline_id], to_pass_further);

                    GlobalRecognizerParameter global_rec_param ( param_.global_recognition_pipeline_config_[global_pipeline_id] );
                    typename GlobalRecognizer<PointT>::Ptr global_r (new GlobalRecognizer<PointT>( global_rec_param ));
                    global_r->setFeatureEstimator( global_concat_estimator );
                    global_r->setClassifier( classifier );
                    global_recognition_pipeline->addRecognizer( global_r );
            }

            global_recognition_pipeline->setVisualizeClusters( visualize_global_results );

            typename RecognitionPipeline<PointT>::Ptr rec_pipeline_tmp = boost::static_pointer_cast<RecognitionPipeline<PointT> > (global_recognition_pipeline);
            mrec_->addRecognitionPipeline( rec_pipeline_tmp );
        }

        mrec_->setModelDatabase( model_database_ );
        mrec_->setNormalEstimator( normal_estimator_ );
        mrec_->setVisualizationParameter(vis_param);
        mrec_->initialize( models_dir_, retrain );
    }


    if(!skip_verification_)
    {
        // ====== SETUP HYPOTHESES VERIFICATION =====
        HV_Parameter paramHV;
        paramHV.load (param_.hv_config_xml_);
        hv_.reset (new HypothesisVerification<PointT, PointT> (camera_, paramHV) );

        if( visualize_hv_go_cues )
            hv_->visualizeCues(vis_param);
        if( visualize_hv_model_cues )
            hv_->visualizeModelCues(vis_param);
        if( visualize_hv_pairwise_cues )
            hv_->visualizePairwiseCues(vis_param);

        hv_->setModelDatabase(model_database_);
    }

    if (param_.remove_planes_)
    {
        // --plane_extraction_method 8 -z 2 --remove_points_below_selected_plane 1 --remove_planes 0 --plane_extractor_maxStepSize 0.1 --use_highest_plane 1 --min_plane_inliers 10000
       //"--min_plane_inliers", "10000",
        std::vector<std::string> additional_cs_arguments = {};

        to_pass_further.insert(to_pass_further.end(), additional_cs_arguments.begin(), additional_cs_arguments.end());
        v4r::apps::CloudSegmenterParameter cs_param;
        to_pass_further = cs_param.init( to_pass_further );
        cloud_segmenter_.reset( new v4r::apps::CloudSegmenter<PointT> (cs_param) );
        cloud_segmenter_->initialize(to_pass_further);
    }

    if(visualize_)
    {
        rec_vis_.reset( new v4r::ObjectRecognitionVisualizer<PointT>);
        rec_vis_->setModelDatabase(model_database_);
    }
}

template<typename PointT>
std::vector<typename ObjectHypothesis<PointT>::Ptr >
ObjectRecognizer<PointT>::recognize(const typename pcl::PointCloud<PointT>::ConstPtr &cloud)
{
    typename pcl::PointCloud<PointT>::Ptr processed_cloud (new pcl::PointCloud<PointT>(*cloud));
    //reset view point - otherwise this messes up PCL's visualization (this does not affect recognition results)
    processed_cloud->sensor_orientation_ = Eigen::Quaternionf::Identity();
    processed_cloud->sensor_origin_ = Eigen::Vector4f::Zero(4);

    verified_hypotheses_.clear();

    std::vector<double> elapsed_time;

    pcl::PointCloud<pcl::Normal>::Ptr normals;
    if( mrec_->needNormals() || hv_ )
    {
        pcl::ScopeTime t("Computing normals");
        normal_estimator_->setInputCloud( processed_cloud );
        normals = normal_estimator_->compute();
        mrec_->setSceneNormals( normals );
        elapsed_time.push_back( t.getTime() );
    }

    if(param_.remove_planes_)
    {
        cloud_segmenter_->setNormals( normals );
        cloud_segmenter_->segment( processed_cloud );
        processed_cloud = cloud_segmenter_->getProcessedCloud();
        const Eigen::Vector4f chosen_plane = cloud_segmenter_->getSelectedPlane();
        mrec_->setTablePlane( chosen_plane );
    }

    // ==== FILTER POINTS BASED ON DISTANCE =====
    for(PointT &p : processed_cloud->points)
    {
        if (pcl::isFinite(p) && p.getVector3fMap().norm() > param_.chop_z_)
            p.x = p.y = p.z = std::numeric_limits<float>::quiet_NaN();
    }

    {
        pcl::ScopeTime t("Generation of object hypotheses");
        mrec_->setInputCloud ( processed_cloud );
        mrec_->recognize();
        generated_object_hypotheses_ = mrec_->getObjectHypothesis();
        elapsed_time.push_back( t.getTime() );
    }


    if(param_.icp_iterations_)
    {
        refinePose(processed_cloud);
    }


    if(!skip_verification_)
    {
        pcl::ScopeTime t("Verification of object hypotheses");
        hv_->setSceneCloud( cloud );
        hv_->setNormals( normals );
        hv_->setHypotheses( generated_object_hypotheses_ );
        hv_->verify();
        verified_hypotheses_ = hv_->getVerifiedHypotheses();
        elapsed_time.push_back( t.getTime() );
    }

    for ( const typename ObjectHypothesis<PointT>::Ptr &voh : verified_hypotheses_ )
    {
        const std::string &model_id = voh->model_id_;
        const Eigen::Matrix4f &tf = voh->transform_;
        float confidence = voh->confidence_;
        LOG(INFO) << "********************" << model_id << " (confidence: " << confidence << ") " << std::endl << tf << std::endl << std::endl;
    }

    if ( visualize_ )
    {
        const std::map<std::string, typename LocalObjectModel::ConstPtr> lomdb = local_recognition_pipeline_->getLocalObjectModelDatabase();
        rec_vis_->setCloud( cloud );
        rec_vis_->setProcessedCloud( processed_cloud ); //comment_out
        rec_vis_->setNormals(normals);

        rec_vis_->setGeneratedObjectHypotheses( generated_object_hypotheses_ ); //comment_out
        rec_vis_->setRefinedGeneratedObjectHypotheses( generated_object_hypotheses_refined_ ); //comment_out
        rec_vis_->setLocalModelDatabase(lomdb);
        rec_vis_->setVerifiedObjectHypotheses( verified_hypotheses_ );
        //rec_vis_->visualize_simple();
        rec_vis_->visualize();
    }

    return verified_hypotheses_;
}


template class V4R_EXPORTS ObjectRecognizer<pcl::PointXYZRGB>;

}

}
