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

public:

    void
    init(ObjectRecognizerParameter &recognizer_param, boost::shared_ptr<ObjectRecognizer<PointT> > &recognizer)
    {
        recognizer_param_ = recognizer_param;
        recognizer_ = recognizer;
    }

    std::vector<typename ObjectHypothesis<PointT>::Ptr >
    hypotheses_verification(std::vector<typename v4r::ObjectHypothesis<PointT>::Ptr> verified_hypotheses,
                            typename pcl::PointCloud<PointT>::Ptr &cloud_in,
                            typename pcl::PointCloud<PointT>::Ptr &cloud_target)
    {

        LOG(INFO) << "################ calculate normals of new scene ######################"<< std::endl;

        //TODO:move to init
        int normal_computation_method_ = recognizer_param_.normal_computation_method_; //NormalEstimatorType::PCL_INTEGRAL_NORMAL;
        typename v4r::NormalEstimator<PointT>::Ptr normal_estimator_;
        std::vector<std::string> empty_vector;
        normal_estimator_ = v4r::initNormalEstimator<PointT> ( normal_computation_method_, empty_vector );
        //end TODO:move to init

        pcl::PointCloud<pcl::Normal>::Ptr normals;
        normal_estimator_->setInputCloud( cloud_in );
        normals = normal_estimator_->compute();

        std::vector<v4r::ObjectHypothesesGroup<PointT> > hypotheses_input_group;
        std::vector<typename v4r::ObjectHypothesis<PointT>::Ptr > hypotheses_output;

        //hypotheses of last recognition redefined as new input for hypotheses verification
        typename v4r::ObjectHypothesis<PointT>::Ptr empty;
        for(unsigned int i=0; i<verified_hypotheses.size(); ++i)
        {
            typename v4r::ObjectHypothesesGroup<PointT> hyp_;
            hyp_.ohs_.push_back(empty);
            hyp_.ohs_[0] = verified_hypotheses[i];
            hypotheses_input_group.push_back(hyp_);
        }

        LOG(INFO) << "################ start hypotheses verification on new scene ######################"<< std::endl;

        //TODO:move to init
        typename v4r::HypothesisVerification<PointT, PointT>::Ptr hv_;
        hv_ = recognizer_->getHypothesesVerification();
        //end TODO:move to init

        //hv_->setModelDatabase(model_database_); //done in recognizer
        hv_->setSceneCloud( cloud_target );
        hv_->setNormals( normals );
        hv_->setHypotheses( hypotheses_input_group );
        hv_->verify();
        hypotheses_output = hv_->getVerifiedHypotheses();

        LOG(INFO) << "hypotheses input from old scene:" << std::endl;
        for ( const typename v4r::ObjectHypothesesGroup<PointT> &gohg : hypotheses_input_group )
        {
            for ( const typename v4r::ObjectHypothesis<PointT>::Ptr &goh : gohg.ohs_ )
            {
                LOG(INFO) << goh->model_id_ << std::endl;
            }
        }

        LOG(INFO) << "hypotheses output for new scene:" << std::endl;
        for ( const typename v4r::ObjectHypothesis<PointT>::Ptr &voh : hypotheses_output )
        {
            LOG(INFO) << voh->model_id_ << std::endl;
        }

        // only compare depth data
        typename pcl::PointCloud<PointT>::Ptr cloud_out (new pcl::PointCloud<PointT>);

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

      /// outlier removal:
      //moving a window over the organized point cloud, only patches size of window_size*window_size are accepted
      typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
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

      verified_hypotheses = recognizer_->recognize(cloud_filtered);
      verified_hypotheses.insert(verified_hypotheses.end(), hypotheses_output.begin(), hypotheses_output.end());

      LOG(INFO) << "final verified hypotheses:" << std::endl;
      for (const typename v4r::ObjectHypothesis<PointT>::Ptr &voh : verified_hypotheses )
      {
          LOG(INFO) << voh->model_id_ << std::endl;
      }

      return verified_hypotheses;
    }
};

}
}
