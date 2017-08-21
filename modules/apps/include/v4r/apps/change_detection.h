#include <boost/serialization/vector.hpp>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>

#include <pcl/segmentation/segment_differences.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <chrono>
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

#include <v4r/recognition/hypotheses_verification.h>
#include <v4r/apps/ObjectRecognizer.h>

#pragma once

namespace v4r
{

namespace apps
{
template<typename PointT>
class V4R_EXPORTS ChangeDetector
{
private:
    boost::shared_ptr<ObjectRecognizer<PointT> > recognizer_;
    ObjectRecognizerParameter recognizer_param_;
    typename v4r::NormalEstimator<PointT>::Ptr normal_estimator_;
    typename v4r::HypothesisVerification<PointT, PointT>::Ptr hv_;

public:

    void
    init(ObjectRecognizerParameter &recognizer_param, boost::shared_ptr<ObjectRecognizer<PointT> > &recognizer)
    {
        recognizer_param_ = recognizer_param;
        recognizer_ = recognizer;

        int normal_computation_method_ = recognizer_param_.normal_computation_method_; //NormalEstimatorType::PCL_INTEGRAL_NORMAL;
        std::vector<std::string> empty_vector;
        normal_estimator_ = v4r::initNormalEstimator<PointT> ( normal_computation_method_, empty_vector );
        hv_ = recognizer_->getHypothesesVerification();
    }

    std::vector<typename ObjectHypothesis<PointT>::Ptr >
    hypotheses_verification(std::vector<typename v4r::ObjectHypothesis<PointT>::Ptr> verified_hypotheses,
                            typename pcl::PointCloud<PointT>::Ptr &cloud_in,
                            typename pcl::PointCloud<PointT>::Ptr &cloud_target)
    {

        LOG(INFO) << "# calculate difference point cloud #"<< std::endl;

        typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
        cloud_filtered = segment_difference(cloud_in, cloud_target);

        LOG(INFO) << "# start recognition of new scene #"<< std::endl;

        std::vector<v4r::ObjectHypothesesGroup<PointT> > generated_hypotheses_group;
        std::vector<typename v4r::ObjectHypothesis<PointT>::Ptr > dummy_hyp;
        //disable hypotheses verification inside recognizer to gain speed!
        recognizer_->set_skip_verification(true);
        dummy_hyp = recognizer_->recognize(cloud_filtered);
        generated_hypotheses_group = recognizer_->getGeneratedObjectHypothesis();

        //hypotheses of last recognition redefined as new input for hypotheses verification
        std::vector<v4r::ObjectHypothesesGroup<PointT> > hypotheses_input_group;
        std::vector<typename v4r::ObjectHypothesis<PointT>::Ptr > hypotheses_output;

        typename v4r::ObjectHypothesis<PointT>::Ptr empty;
        for(unsigned int i=0; i<verified_hypotheses.size(); ++i)
        {
            typename v4r::ObjectHypothesesGroup<PointT> hyp_;
            hyp_.ohs_.push_back(empty);
            hyp_.ohs_[0] = verified_hypotheses[i];
            hypotheses_input_group.push_back(hyp_);
        }

        //combine old and new hypotheses for input into hv

        LOG(INFO) << "hypotheses input from old scene:" << std::endl;
        for ( const typename v4r::ObjectHypothesesGroup<PointT> &gohg : hypotheses_input_group )
        {
            for ( const typename v4r::ObjectHypothesis<PointT>::Ptr &goh : gohg.ohs_ )
            {
                LOG(INFO) << goh->model_id_ << std::endl;
            }
        }

        hypotheses_input_group.insert(hypotheses_input_group.end(), generated_hypotheses_group.begin(), generated_hypotheses_group.end());

        LOG(INFO) << "hypotheses input :" << std::endl;
        for ( const typename v4r::ObjectHypothesesGroup<PointT> &gohg : hypotheses_input_group )
        {
            for ( const typename v4r::ObjectHypothesis<PointT>::Ptr &goh : gohg.ohs_ )
            {
                LOG(INFO) << goh->model_id_ << std::endl;
            }
        }

        LOG(INFO) << "# calculate normals of new scene #"<< std::endl;

        pcl::PointCloud<pcl::Normal>::Ptr normals;
        normal_estimator_->setInputCloud( cloud_target ); //cloud_target
        normals = normal_estimator_->compute();

        LOG(INFO) << "# start hypotheses verification on new scene #"<< std::endl;

        //hv_->setModelDatabase(model_database_); //done in recognizer
        hv_->setSceneCloud( cloud_target );
        hv_->setNormals( normals );
        hv_->setHypotheses( hypotheses_input_group );
        hv_->verify();
        hypotheses_output = hv_->getVerifiedHypotheses();

        LOG(INFO) << "final verified hypotheses:" << std::endl;
        for (const typename v4r::ObjectHypothesis<PointT>::Ptr &voh : hypotheses_output ) //verified_hypotheses
        {
            LOG(INFO) << voh->model_id_ << std::endl;
        }

//              pcl::visualization::PCLVisualizer viewer("3D Viewer");
//              viewer.addPointCloud(cloud_filtered, "our point cloud");
//              viewer.setBackgroundColor(1,1,1);
//              viewer.setCameraPosition(0, 0, 0, 0, 0, 1, 0, -1, 0);
//              viewer.resetCamera();
//              viewer.spin();

        return hypotheses_output; //verified_hypotheses
    }

    typename pcl::PointCloud<PointT>::Ptr
    segment_difference(typename pcl::PointCloud<PointT>::Ptr &cloud_in,
                    typename pcl::PointCloud<PointT>::Ptr &cloud_target)
    {
        // only compare depth data
        typename pcl::PointCloud<PointT>::Ptr cloud_out (new pcl::PointCloud<PointT>);

        cloud_out->width = cloud_in->width;
        cloud_out->height = cloud_in->height;

        LOG(INFO) << "cloud_in->width:" << cloud_in->width << std::endl;
        LOG(INFO) << "cloud_in->height:" << cloud_in->height << std::endl;

        unsigned int number_points = cloud_in->width * cloud_in->height;
        cloud_out->points.resize(number_points);

//        std::cout <<

        float z_target, r_target, g_target, b_target, z_in, r_in, g_in, b_in, dif_rgb, target_rgb, in_rgb;
        for(unsigned int i=0; i<number_points; ++i)
        {
            z_target = cloud_target->points[i].z;
            z_in = cloud_in->points[i].z;

            if(std::isnan(z_target))
                continue;

            if(std::isnan(z_in)) {
                cloud_out->points[i] = cloud_target->points[i];
                continue;
            }

            r_in = cloud_in->points[i].r;
            g_in = cloud_in->points[i].g;
            b_in = cloud_in->points[i].b;
            in_rgb = std::sqrt(std::pow(r_in,2) + std::pow(g_in,2) + std::pow(b_in,2));

            r_in = r_in/in_rgb;
            g_in = g_in/in_rgb;
            b_in = b_in/in_rgb;

            r_target = cloud_target->points[i].r;
            g_target = cloud_target->points[i].g;
            b_target = cloud_target->points[i].b;
            target_rgb = std::sqrt(std::pow(r_target,2) + std::pow(g_target,2) + std::pow(b_target,2));

            r_target = r_target/target_rgb;
            g_target = g_target/target_rgb;
            b_target = b_target/target_rgb;

            dif_rgb = std::sqrt(std::pow(r_target - r_in,2) + std::pow(g_target - g_in,2) + std::pow(b_target - b_in,2));

            //std::fabs(r_target - cloud_in->points[i].r) >= color_thres*r_target))
            if((std::fabs(z_target - z_in) >= 0.02*z_target*z_target) || dif_rgb > 0.3)
            {
                cloud_out->points[i] = cloud_target->points[i];
            }
        }
      //return cloud_out;

      /// outlier removal:
//      //moving a window over the organized point cloud, only patches size of window_size*window_size are accepted
//      typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
//      cloud_filtered->width = cloud_in->width;
//      cloud_filtered->height = cloud_in->height;

//      unsigned int number_points1 = cloud_in->width * cloud_in->height;
//      cloud_filtered->points.resize(number_points1);

//      int window_size = 5;
//      //moving window over organized point cloud
//      for(unsigned int i=0; i<cloud_out->width - window_size; ++i)
//      {
//      for(unsigned int jj=0; jj<cloud_out->height - window_size; ++jj)
//      {
//          //if data of actual point is != 0 the whole patch (window_size*window_size) is checkt
//          if(cloud_out->at(i,jj).rgb != 0)
//          {
//              bool store = true;
//              for(int k=0; k<=window_size; ++k)
//              {
//              for(int mm=0; mm<=window_size; ++mm)
//              {
//                  if(cloud_out->at(i+k,jj+mm).rgb == 0)
//                  {
//                      k =window_size+1;
//                      mm= window_size+1;
//                      store = false;
//                  }
//              }
//              }

//              //if the whole patch is filled with data, it gets accepted
//              if(store == true)
//              {
//                  for(int k=0; k<=window_size; ++k)
//                  {
//                  for(int mm=0; mm<=window_size; ++mm)
//                  {
//                      cloud_filtered->at(i+k,jj+mm) = cloud_out->at(i+k,jj+mm);
//                  }
//                  }
//                  jj=jj+window_size;
//              }
//          }
//      }
//      }

//      return cloud_filtered;
        return cloud_out;

    } //end init_empty_scene

    std::vector<typename ObjectHypothesis<PointT>::Ptr >
    init_workspace(typename pcl::PointCloud<PointT>::Ptr &cloud_original,
                   typename pcl::PointCloud<PointT>::Ptr &cloud_empty_workspace)
    {
        LOG(INFO) << "# calculate difference point cloud #"<< std::endl;

        typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
        cloud_filtered = segment_difference(cloud_empty_workspace, cloud_original);

        LOG(INFO) << "# start recognition of new scene #"<< std::endl;

        std::vector<v4r::ObjectHypothesesGroup<PointT> > generated_hypotheses_group;
        std::vector<typename v4r::ObjectHypothesis<PointT>::Ptr > dummy_hyp;
        //disable hypotheses verification inside recognizer to gain speed!
        recognizer_->set_skip_verification(true);
        dummy_hyp = recognizer_->recognize(cloud_filtered);
        generated_hypotheses_group = recognizer_->getGeneratedObjectHypothesis();

        LOG(INFO) << "hypotheses input :" << std::endl;
        for ( const typename v4r::ObjectHypothesesGroup<PointT> &gohg : generated_hypotheses_group )
        {
            for ( const typename v4r::ObjectHypothesis<PointT>::Ptr &goh : gohg.ohs_ )
            {
                LOG(INFO) << goh->model_id_ << std::endl;
            }
        }

        LOG(INFO) << "# calculate normals of new scene #"<< std::endl;

        pcl::PointCloud<pcl::Normal>::Ptr normals;
        normal_estimator_->setInputCloud( cloud_original ); //cloud_targe
        normals = normal_estimator_->compute();

        LOG(INFO) << "# start hypotheses verification on new scene #"<< std::endl;

        std::vector<typename v4r::ObjectHypothesis<PointT>::Ptr > hypotheses_output;
        //hv_->setModelDatabase(model_database_); //done in recognizer
        hv_->setSceneCloud( cloud_original );
        hv_->setNormals( normals );
        hv_->setHypotheses( generated_hypotheses_group );
        hv_->verify();
        hypotheses_output = hv_->getVerifiedHypotheses();

        LOG(INFO) << "final verified hypotheses:" << std::endl;
        for (const typename v4r::ObjectHypothesis<PointT>::Ptr &voh : hypotheses_output ) //verified_hypotheses
        {
            LOG(INFO) << voh->model_id_ << std::endl;
        }

        //      pcl::visualization::PCLVisualizer viewer("3D Viewer");
        //      viewer.addCoordinateSystem(1.0f);
        //      viewer.addPointCloud(cloud_filtered, "our point cloud");
        //      viewer.spin();

        return hypotheses_output; //verified_hypotheses
    }


};

}
}
