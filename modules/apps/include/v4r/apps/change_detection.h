
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

namespace v4r
{

namespace apps
{
template<typename PointT>
class V4R_EXPORTS ChangeDetection
{
private:
    boost::shared_ptr<ObjectRecognizer<PointT> > recognizer_;

public:

    void
    init(ObjectRecognizer<PointT> &recognizer)
    {
        recognizer_ = recognizer;
    }

    std::vector<typename ObjectHypothesis<PointT>::Ptr >
    hypotheses_verification(const typename ObjectHypothesis<PointT>::Ptr &verified_hypotheses,
                            const typename pcl::PointCloud<PointT>::ConstPtr &cloud_in,
                            const typename pcl::PointCloud<PointT>::ConstPtr &cloud_target);


};

}
}
