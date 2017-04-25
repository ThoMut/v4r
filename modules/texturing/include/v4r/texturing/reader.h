/**
 * $Id$
 * 
 * Software License Agreement (GNU General Public License)
 *
 *  Copyright (C) 2015:
 *
 *    Johann Prankl, prankl@acin.tuwien.ac.at
 *    Aitor Aldoma, aldoma@acin.tuwien.ac.at
 *
 *      Automation and Control Institute
 *      Vienna University of Technology
 *      Gusshausstraße 25-29
 *      1170 Vienn, Austria
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @author Johann Prankl, Aitor Aldoma
 *
 */

#include <v4r/core/macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/filesystem.hpp>

namespace v4r
{
namespace texturing
{

class V4R_EXPORTS Reader
{
private:
    Eigen::Matrix4f readPose(std::string file);

    void getFilesInDirectory (std::string path_with_views,
                           std::vector<std::string> & view_filenames,
                           const std::string & pattern);

public:
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> readInputClouds(std::string path, std::string pattern, int step=1);
    std::vector<Eigen::Matrix4f> readPoses(std::string path, std::string pattern, int step=1);
    std::vector<std::vector<int> > readIndices(std::string path, std::string pattern, int step=1);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr readModel(std::string path, std::string filename);
};

}
}
