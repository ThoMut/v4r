#include <v4r/apps/change_detection.h>

namespace v4r
{

namespace apps
{
//template<typename PointT>
//void
//ChangeDetection<PointT>::init(const typename recognizer)
//{

//}

template<typename PointT>
std::vector<typename ObjectHypothesis<PointT>::Ptr >
ChangeDetection<PointT>::hypotheses_verification(const typename v4r::ObjectHypothesis<PointT>::Ptr &verified_hypotheses,
                                                 const typename pcl::PointCloud<PointT>::ConstPtr &cloud_in,
                                                 const typename pcl::PointCloud<PointT>::ConstPtr &cloud_target)
{
    std::cout << "################ calculate normals of new scene ######################"<< std::endl;

    int normal_computation_method_ = recognizer_->normal_computation_method_; //NormalEstimatorType::PCL_INTEGRAL_NORMAL;
    typename v4r::NormalEstimator<PointT>::Ptr normal_estimator_;
    std::vector<std::string> empty_vector;
    normal_estimator_ = v4r::initNormalEstimator<PointT> ( normal_computation_method_, empty_vector );

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

    std::cout << "################ start hypotheses verification ######################"<< std::endl;
    typename v4r::HypothesisVerification<PointT, PointT>::Ptr hv_;
    hv_ = recognizer_->getHypothesesVerification();

    //hv_->setModelDatabase(model_database_); //done in recognizer
    hv_->setSceneCloud( cloud_target );
    hv_->setNormals( normals );
    hv_->setHypotheses( hypotheses_input_group );
    hv_->verify();
    hypotheses_output = hv_->getVerifiedHypotheses();

    std::cout << "hypotheses input:" << std::endl;
    for ( const typename v4r::ObjectHypothesesGroup<PointT> &gohg : hypotheses_input_group )
    {
        for ( const typename v4r::ObjectHypothesis<PointT>::Ptr &goh : gohg.ohs_ )
        {
            std::cout << goh->model_id_ << std::endl;
        }
    }

    std::cout << "hypotheses output:" << std::endl;
    for ( const typename v4r::ObjectHypothesis<PointT>::Ptr &voh : hypotheses_output )
    {
        std::cout << voh->model_id_ << std::endl;
    }


}

}
}
