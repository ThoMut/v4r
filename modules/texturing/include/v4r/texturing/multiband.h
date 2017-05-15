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
#pragma once

#include <vector>
#include <string>
#include <set>

#include <v4r/core/macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/range_image/range_image_planar.h>

#include <pcl/Vertices.h>
#include <pcl/PolygonMesh.h>

#include <opencv2/core/core.hpp>

#include "TexturedMesh.h"

#include "renderer.h"

namespace v4r
{
namespace texturing
{

struct Polygon
{
    Eigen::Vector3f normal;
    Eigen::Vector3f point1;
    pcl::Vertices vertices;
    std::vector<int> viewingImageSet;

    bool *imageVisibility;
    double *angles;
    int targetTexture;
};

struct Image
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    Eigen::Matrix4f pose;
    Eigen::Affine3f fullPose;

    Eigen::Vector3f cam_vector;

    cv::Mat3b texture;
    cv::Mat1b texture_blending_state;
    std::vector<cv::Mat3f> gaussian;
    std::vector<cv::Mat3f> laplacian;

    pcl::RangeImagePlanar range_image;

    cv::Rect croppedSize;
};

class V4R_EXPORTS MultibandTexture
{
private:
    Renderer *renderer;
    int angleThreshDegree;
    bool projectNormals;
    double angleThresh;
    bool enableBlending;
    int pyramidDepth;
    int kernelSize;
    float sigma;
    int padding;
    float focal_length;
    //std::string texturePath;

public:
    MultibandTexture(Renderer *renderer, int angleThreshDegree, bool enableBlending, int padding, float focal_length);

    void generatePyramids(Image *image);

    float calculateWeight(double angle, int b, double worstAngle);

    void blendPixel(int x, int y, int min_x, int min_y, std::vector<Image> *inputImages, std::vector<Polygon> *polygons, int polyIndex, int imageIndex, bool isPadding);
    void setPixel(int x, int y, int min_x, int min_y, std::vector<Image> *inputImages, std::vector<Polygon> *polygons, int polyIndex, int imageIndex, bool isPadding);

    bool combination(std::vector<Polygon> *polygons, std::set<int> &imageSubset, int imageSize, std::set<int> &images, std::set<int> &combinations, int offset, int k);

    TexturedMesh::Ptr reconstruct(pcl::PolygonMesh::Ptr mesh, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds, std::vector<Eigen::Matrix4f> poses);

    void addTexture(pcl::PointCloud<pcl::PointXYZ>::Ptr meshCloud, TexturedMesh::Ptr result, std::vector<Image> *inputImages, int imageIndex, std::vector<Polygon> *polygons, std::set<int> imageSubset);

    Eigen::Vector3f calculateNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr meshCloud, pcl::Vertices vertices/*, Eigen::Matrix4f pose*/);

    pcl::RangeImagePlanar getMeshProjection(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Matrix4f pose);

    pcl::RangeImage getMeshProjection2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Matrix4f pose);

    Eigen::Vector3f getCamVector(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, Eigen::Matrix4f pose);

    Eigen::Vector3f get3dPoint(int x, int y, Image *inputImage, Polygon *p);

};

}
}
