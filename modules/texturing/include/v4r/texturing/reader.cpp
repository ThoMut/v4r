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

#include "reader.h"

#include <boost/regex.hpp>

#include <pcl/io/pcd_io.h>

struct IndexPoint
{
    int idx;
};

POINT_CLOUD_REGISTER_POINT_STRUCT (IndexPoint,
                                   (int, idx, idx)
                                   )

std::vector<std::vector<int> > Reader::readIndices(std::string path, std::string pattern, int step)
{
    std::vector<std::string> files;
    getFilesInDirectory(path, files, pattern);

    std::vector<std::vector<int> > result;

    for (size_t i = 0; i < files.size (); i+=step)
    {
        printf("Load indices file: %s\n", files[i].c_str());

        pcl::PointCloud<IndexPoint> obj_indices_cloud;
        pcl::io::loadPCDFile (files[i], obj_indices_cloud);

        std::vector<int> indicesList;
        for(size_t k=0; k < obj_indices_cloud.points.size(); k++)
          indicesList.push_back(obj_indices_cloud.points[k].idx);

        result.push_back(indicesList);
    }

    return result;
}

std::vector<Eigen::Matrix4f> Reader::readPoses(std::string path, std::string pattern, int step)
{
    std::vector<std::string> files;
    getFilesInDirectory(path, files, pattern);


    std::vector<Eigen::Matrix4f> poses;

    for (size_t i = 0; i < files.size (); i+=step)
    {
        printf("Load pose file: %s\n", files[i].c_str());

        Eigen::Matrix4f pose = readPose(files[i]);

        poses.push_back(pose);
    }

    return poses;
}


pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr Reader::readModel(std::string path, std::string filename)
{
    if (path[path.size() - 1] != '/')
    {
        path.append("/");
    }

    path.append(filename);

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pointCloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

    std::cout << "Load model file: " << path.c_str() << std::endl;

    pcl::io::loadPCDFile (path, *pointCloud);

    return pointCloud;
}

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> Reader::readInputClouds(std::string path, std::string pattern, int step)
{
    std::vector<std::string> files;
    getFilesInDirectory(path, files, pattern);


    // load point clouds
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pointClouds;

    for (size_t i = 0; i < files.size (); i+=step)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud (new pcl::PointCloud<pcl::PointXYZRGB>);

        std::cout << "Load PCD file: " << files[i].c_str() << std::endl;

        pcl::io::loadPCDFile (files[i], *pointCloud);

        pointClouds.push_back(pointCloud);
    }

    return pointClouds;
}



Eigen::Matrix4f Reader::readPose(std::string file)
{
    Eigen::Matrix4f pose;

    std::ifstream in;
    in.open (file.c_str (), std::ifstream::in);

    char linebuf[1024];
    in.getline (linebuf, 1024);
    std::string line (linebuf);
    std::vector<std::string> strs_2;
    boost::split (strs_2, line, boost::is_any_of (" "));

    int c = 0;
    for (int i = 0; i < 16; i++, c++)
    {
        pose (c / 4, c % 4) = static_cast<float> (atof (strs_2[i].c_str ()));
    }

    return pose;
}

void Reader::getFilesInDirectory (std::string path,
                       std::vector<std::string> & view_filenames,
                       const std::string & pattern)
{


    if (path[path.size() - 1] != '/')
    {
        path.append("/");
    }

    boost::filesystem::path path_with_views = path;

  std::stringstream filter_str;
  filter_str << pattern;
  const boost::regex my_filter( filter_str.str() );

  boost::filesystem::directory_iterator end_itr;
  for (boost::filesystem::directory_iterator itr (path_with_views); itr != end_itr; ++itr)
  {
    if (!(boost::filesystem::is_directory (*itr)))
    {
      std::vector < std::string > strs;
      std::vector < std::string > strs_;

#if BOOST_FILESYSTEM_VERSION == 3
      std::string file = (itr->path ().filename ()).string();
#else
      std::string file = (itr->path ()).filename ();
#endif

      boost::smatch what;
      if( !boost::regex_match( file, what, my_filter ) ) continue;

#if BOOST_FILESYSTEM_VERSION == 3
        view_filenames.push_back ((itr->path ().filename ()).string());
#else
        view_filenames.push_back ((itr->path ()).filename ());
#endif
    }
  }

  std::cout << "Load pcd files from source dir: " << path << std::endl;

  for (size_t i = 0; i < view_filenames.size (); i++)
  {
      std::cout << "Load pcd file " << view_filenames[i] << std::endl;

      view_filenames[i].insert(0, path);
  }

  // sort files
  std::sort (view_filenames.begin (), view_filenames.end ());
}
