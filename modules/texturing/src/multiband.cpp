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

#include <v4r/texturing/multiband.h>

#include <boost/lexical_cast.hpp>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/io/io.h>

#include <pcl/search/kdtree.h>

#include <math.h>

#include <pcl/io/file_io.h>
#include <pcl/io/png_io.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include <pcl/common/projection_matrix.h>

#include <v4r/texturing/hittingset.h>

namespace v4r
{
namespace texturing
{

MultibandTexture::MultibandTexture(Renderer *renderer, int angleThreshDegree, bool enableBlending, int padding, float focal_length) :
    angleThreshDegree(angleThreshDegree), enableBlending(enableBlending), padding(padding), focal_length(focal_length)
{
    this->renderer = renderer;
    pyramidDepth = 4;
    kernelSize = 5;
    sigma = 2.0f;
    //texturePath = std::string("/home/alex/foo/");
}

Eigen::Vector3f MultibandTexture::getCamVector(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, Eigen::Matrix4f pose)
{
    Eigen::Vector4f cam_vector = Eigen::Vector4f(0.0f, 0.0f, -1.0f, 0.0f);

    Eigen::Affine3f sensorPose = Eigen::Affine3f(Eigen::Translation3f(input->sensor_origin_[0],
                                 input->sensor_origin_[1],
                                 input->sensor_origin_[2])) *
                                 Eigen::Affine3f(input->sensor_orientation_);

    cam_vector = sensorPose * cam_vector;

    cam_vector = pose * cam_vector;

    return Eigen::Vector3f(cam_vector[0], cam_vector[1], cam_vector[2]);
    //return Eigen::Vector3f(0.0f, 0.0f, 1.0f);
}

bool MultibandTexture::combination(std::vector<Polygon> *polygons, std::set<int> &imageSubset, int imageSize, std::set<int> &images, std::set<int> &combinations, int offset, int k)
{
    /*
    std::cout << "Checking Combination: ";

    std::set<int>::iterator it;
    for (it = combinations.begin(); it != combinations.end(); ++it)
    {
        std::cout << *it << " ";
    }

    std::cout << std::endl;
    */

    if (k == 0) {

        // check if finished

        for (int polyIndex=0;polyIndex<(int)polygons->size();polyIndex++)
        {
            bool match = false;

            std::set<int>::iterator it;
            for (it = combinations.begin(); it != combinations.end(); ++it)
            {
                if (polygons->at(polyIndex).imageVisibility[*it] == true)
                {
                    match = true;
                    break;
                }
            }

            if (!match)
            {
                for (it = imageSubset.begin(); it != imageSubset.end(); ++it)
                {
                    if (polygons->at(polyIndex).imageVisibility[*it] == true)
                    {
                        match = true;
                        break;
                    }
                }
            }

            if (!match)
            {
                bool empty = true;

                for (int i=0;i<imageSize;i++)
                {
                    if (polygons->at(polyIndex).imageVisibility[i] == true)
                    {
                        empty = false;
                    }
                }

                if (!empty)
                {
                    return false;
                }
            }
        }

        std::cout << "MATCH FOUND !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;


        std::cout << "Combinations " << std::endl;

        std::set<int>::iterator it;
        for (it = combinations.begin(); it != combinations.end(); ++it)
        {
            std::cout << *it << " ";
        }

        std::cout << std::endl;

        std::cout << "Single matches " << std::endl;

        for (it = imageSubset.begin(); it != imageSubset.end(); ++it)
        {
            std::cout << *it << " ";
        }

        std::cout << std::endl;

        return true;
/*
        std::cout << "Combination: ";

        std::set<int>::iterator it;
        for (it = combinations.begin(); it != combinations.end(); ++it)
        {
            std::cout << *it << " ";
        }

        std::cout << std::endl;
*/
    }

    for (int i = offset; i <= (int)images.size() - k; ++i) {
        std::set<int>::iterator it = images.begin();
        std::advance(it, i);
        int x = *it;

        combinations.insert(x);
        bool match = combination(polygons, imageSubset, imageSize, images, combinations, i+1, k-1);

        if (match)
        {
            return true;
        }

        combinations.erase(x);
    }

    return false;
}

TexturedMesh::Ptr MultibandTexture::reconstruct(pcl::PolygonMesh::Ptr mesh, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds, std::vector<Eigen::Matrix4f> poses)
{
    std::cout << "processing pcl texture" << std::endl;

    std::cout << "mesh size " << mesh->polygons.size() << std::endl;

    std::cout << "texture size " << clouds.size() << std::endl;

    std::cout << "pose size " << poses.size() << std::endl;

    // init data structures
    int nrPolygons = mesh->polygons.size();
    int nrImages = clouds.size();

    pcl::PointCloud<pcl::PointXYZ>::Ptr meshCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2 (mesh->cloud, *meshCloud);

    std::vector<Polygon> polygons;

    // init polygons
    for (int polyIndex=0;polyIndex<nrPolygons;polyIndex++)
    {
        Eigen::Vector3f triangle_normal = calculateNormal(meshCloud, mesh->polygons[polyIndex]);
        triangle_normal.normalize();

        Polygon poly;

        pcl::PointXYZ p1 = meshCloud->points[mesh->polygons[polyIndex].vertices[0]];
        poly.point1 = Eigen::Vector3f(p1.x, p1.y, p1.z);
        poly.vertices = mesh->polygons[polyIndex];
        poly.normal = triangle_normal;
        poly.imageVisibility = new bool[nrImages];
        poly.angles = new double[nrImages];

        polygons.push_back(poly);
    }

    std::vector<Image> inputImages;

    // init input images
    for (int imgIndex=0;imgIndex<nrImages;imgIndex++)
    {
        Image image;

        Eigen::Vector3f cam_vector = getCamVector(clouds[imgIndex], poses[imgIndex]); //Eigen::Vector3f(0.0f, 0.0f, -1.0f);//cloud->sensor_orientation_.normalized().vec();
        cam_vector.normalize();

        image.cloud = clouds[imgIndex];
        image.pose = poses[imgIndex];
        image.cam_vector = cam_vector;


        cv::Mat3b cv_img(image.cloud->height, image.cloud->width, CV_8UC3);

        for (int i=0;i<(int)image.cloud->width;i++)
        {
            for (int j=0;j<(int)image.cloud->height;j++)
            {
                cv_img.at<cv::Vec3b>(j, i).val[0] = image.cloud->at(i, j).b;
                cv_img.at<cv::Vec3b>(j, i).val[1] = image.cloud->at(i, j).g;
                cv_img.at<cv::Vec3b>(j, i).val[2] = image.cloud->at(i, j).r;
            }
        }

        cv::Mat3b cv_img_filtered(image.cloud->height, image.cloud->width, CV_8UC3);
        //cv::fastNlMeansDenoisingColored(cv_img, cv_img_filtered);
        //cv::bilateralFilter(cv_img, cv_img_filtered, 9, 75.0, 75.0);
        //cv::medianBlur(cv_img, cv_img_filtered, 3);

        image.texture = cv_img;
        generatePyramids(&image);


        cv::Mat1b state(image.cloud->height, image.cloud->width);
        state.setTo(0);
        image.texture_blending_state = state;


        /*
        for (int b=0;b<pyramidDepth;b++)
        {
            std::string filename("/home/alex/temp/");
            filename.append(boost::lexical_cast<std::string>(imgIndex));
            filename.append("_");
            filename.append(boost::lexical_cast<std::string>(b));
            filename.append("_g.png");

            std::string filename2("/home/alex/temp/");
            filename2.append(boost::lexical_cast<std::string>(imgIndex));
            filename2.append("_");
            filename2.append(boost::lexical_cast<std::string>(b));
            filename2.append("_l.png");

            //std::cout << "filename: " << filename << std::endl;
            //std::cout << "filename: " << filename2 << std::endl;

            cv::imwrite(filename, image.gaussian[b]);
            cv::imwrite(filename2, image.laplacian[b]);
        }

        {
            std::cout << "output test" << std::endl;

            cv::Mat3f test(image.cloud->height, image.cloud->width, CV_32FC3);
            test.setTo(cv::Scalar(0,0,0));

            for (int b=0;b<pyramidDepth;b++)
            {
                test += image.laplacian[b];
            }

            std::string filename2("/home/alex/temp/");
            filename2.append("test_");
            filename2.append(boost::lexical_cast<std::string>(imgIndex));
            filename2.append(".png");

            cv::imwrite(filename2, test);
        }
*/

        Eigen::Affine3f sensorPose = Eigen::Affine3f(Eigen::Translation3f(image.cloud->sensor_origin_[0],
                                     image.cloud->sensor_origin_[1],
                                     image.cloud->sensor_origin_[2])) *
                                     Eigen::Affine3f(image.cloud->sensor_orientation_);

        sensorPose = sensorPose * image.pose;

        image.fullPose = sensorPose;


        image.range_image = getMeshProjection(meshCloud, image.pose);


        image.croppedSize.x = image.cloud->width;
        image.croppedSize.y = image.cloud->height;
        image.croppedSize.width = 0;
        image.croppedSize.height = 0;

        inputImages.push_back(image);
    }


    angleThresh = (angleThreshDegree * M_PI) / 180.0;
    float maxAngle = (80 * M_PI) / 180.0;


    std::cout << "Created mesh cloud" << std::endl;

    std::cout << "Created array" << std::endl;

    // check for image visibility based on viewing angle and threshold
    for (int polyIndex=0;polyIndex<nrPolygons;polyIndex++)
    {
        double bestAngle = 100.0;
        //int bestAngleIndex = -1;
        //bool imgFound = false;
        //std::vector<double> angles;

        for (int imgIndex=0;imgIndex<nrImages;imgIndex++)
        {
            double angle = inputImages[imgIndex].cam_vector.dot(polygons[polyIndex].normal);

            angle = fabs(acos(angle));
            //angles.push_back(angle);

            //std::cout << "angle " << angle << std::endl;

            if (angle < bestAngle)
            {
                bestAngle = angle;
                //bestAngleIndex = imgIndex;
            }

            /*
            if (angle < angleThresh)
            {
                imageVisibility[polyIndex][imgIndex] = true;
                imgFound = true;
            } else {
                imageVisibility[polyIndex][imgIndex] = false;
            }*/

            polygons[polyIndex].imageVisibility[imgIndex] = false;
            polygons[polyIndex].angles[imgIndex] = angle;
        }

        if (bestAngle < maxAngle) {
            for (int imgIndex=0;imgIndex<nrImages;imgIndex++)
            {
                if (fabs(bestAngle - polygons[polyIndex].angles[imgIndex]) < angleThresh)
                {
                    polygons[polyIndex].imageVisibility[imgIndex] = true;
                }
            }
        }

        /*
        if (!imgFound && bestAngle < maxAngle) // if no image below threshold, take best one
        {
            imageVisibility[polyIndex][bestAngleIndex] = true;
        }
        */
    }

    std::cout << "Checked image visibility" << std::endl;


    std::set<int> remainingImages;
    std::set<int> imageSubset;

    for (int i=0;i<(int)clouds.size();i++)
    {
        remainingImages.insert(i);
    }


    // process polygons that can only be seen by one image, and add image to subset
    for (int polyIndex=0;polyIndex<nrPolygons;polyIndex++)
    {
        int imageCounter = 0;
        int lastImageIndex = -1;

        for (int imgIndex=0;imgIndex<nrImages;imgIndex++)
        {
            if (polygons[polyIndex].imageVisibility[imgIndex])
            {
                imageCounter++;
                lastImageIndex = imgIndex;
            }
        }

        //std::cout << "image counter " << imageCounter << std::endl;

        /*
        if (imageCounter == 0)
        {
            std::cout << "WARNING !!!!!!!!!!!!" << std::endl;
        }
        */

        if (imageCounter == 1)
        {
            std::cout << "image counter " << lastImageIndex << std::endl;
            imageSubset.insert(lastImageIndex);
            remainingImages.erase(lastImageIndex);
        }
    }

    for (int i=0;i<nrImages;i++)
    {
        renderer->renderTextureStep(mesh, inputImages[i].cloud, getCamVector(inputImages[i].cloud, poses[i]));
    }

    std::cout << "Image subset: " << imageSubset.size() << " Remaining: " << remainingImages.size() << std::endl;


    HittingSet hittingSet;
    std::vector<std::vector<int> > subsets;
    std::vector<int> set;

    for (int i=0;i<(int)polygons.size();i++)
    {
        bool alreadyMatched = false;
        std::vector<int> subset;


        for (int k=0;k<nrImages;k++)
        {
            if (polygons[i].imageVisibility[k])
            {
                subset.push_back(k);

                if (imageSubset.find(k) != imageSubset.end())
                {
                    alreadyMatched = true;
                    break;
                }
            }
        }

        if (!alreadyMatched && subset.size() > 0)
        {
            subsets.push_back(subset);
        }
    }

    std::set<int>::iterator it2;
    for (it2=remainingImages.begin();it2!=remainingImages.end();it2++)
    {
        set.push_back(*it2);
    }

    std::set<int> combinations = hittingSet.solve(set, subsets);

    /*
    std::set<int> combinations;
    for (int i=1;i<=remainingImages.size();i++)
    {
        std::cout << "Start iteration " << i << std::endl;

        combinations.clear();
        bool match = combination(&polygons, imageSubset, nrImages, remainingImages, combinations, 0, i);

        if (match)
        {
            std::cout << "match for " << i << " images" << std::endl;
            break;
        }
    }
    */

    // Remove matched polygons to improve performance
    // TODO: k-approximation of k-hitting algorithm for large image subsets





    // apply texture to mesh


    std::cout << "render textures" << std::endl;

    TexturedMesh::Ptr result(new TexturedMesh());
    result->mesh = mesh;
    result->textureCoordinates.resize(meshCloud->size());
    result->textureIndex.resize(mesh->polygons.size());
    result->textureIndex2.resize(meshCloud->size());

    std::cout << "setup coords" << std::endl;

    for (int i=0;i<(int)meshCloud->size();i++)
    {
        result->textureCoordinates[i] = Eigen::Vector2f(0.0f, 0.0f);
        result->textureIndex2[i] = 0;
    }

    std::cout << "setup indices" << std::endl;

    for (int i=0;i<(int)mesh->polygons.size();i++)
    {
        result->textureIndex[i] = 0;
    }

    std::cout << "create textures" << std::endl;
    std::cout << "cloud size " << clouds.size() << std::endl;
    std::cout << "poses size " << poses.size() << std::endl;

    // create textures
    std::set<int>::iterator it;
    for (it = combinations.begin(); it != combinations.end(); ++it)
    {
        imageSubset.insert(*it);
    }

    // calculate texture coordinates

    int totalWidth = 0;
    int totalHeight = 0;
    for (it = imageSubset.begin(); it != imageSubset.end(); ++it)
    {
        addTexture(meshCloud, result, &inputImages, *it, &polygons, imageSubset);
        totalWidth += inputImages.at(*it).croppedSize.width - inputImages.at(*it).croppedSize.x;
        totalHeight = std::max(totalHeight, inputImages.at(*it).croppedSize.height - inputImages.at(*it).croppedSize.y);
    }

    std::cout << "Added textures" << std::endl;

    /*
    int totalWidth = 0;
    int totalHeight = 0;
    for (int imageIndex=0;imageIndex<imageSubset.size() ;imageIndex++)
    {
        totalWidth += result->textures[imageIndex].cols;
        totalHeight = std::max(totalHeight, result->textures[imageIndex].rows);
    }
    */

    std::cout << "create new texture" << totalWidth << " " << totalHeight << std::endl;
    cv::Mat3b texture(totalHeight, totalWidth, CV_8UC3);
    for (int x=0;x<totalWidth;x++)
    {
        for (int y=0;y<totalHeight;y++)
        {
            texture.at<cv::Vec3b>(y, x).val[0] = 0;
            texture.at<cv::Vec3b>(y, x).val[1] = 0;
            texture.at<cv::Vec3b>(y, x).val[2] = 0;
        }
    }

    int offsetX = 0;
    int imageIndex = 0;
    for (it = imageSubset.begin(); it != imageSubset.end(); ++it)
    //for (int imageIndex=0;imageIndex<imageSubset.size() ;imageIndex++)
    {
        cv::Rect cropSize = inputImages.at(*it).croppedSize;

        std::cout << "copy texture " << imageIndex << std::endl;
        // copy texture
        for (int x=cropSize.x;x<cropSize.width;x++)
        {
            for (int y=cropSize.y;y<cropSize.height;y++)
            {
                if (x >= 0 && y >= 0 && x < result->textures[imageIndex].cols && y < result->textures[imageIndex].rows)
                {
                    texture.at<cv::Vec3b>(y - cropSize.y, x + offsetX - cropSize.x).val[0] = result->textures[imageIndex].at<cv::Vec3b>(y, x).val[0];
                    texture.at<cv::Vec3b>(y - cropSize.y, x + offsetX - cropSize.x).val[1] = result->textures[imageIndex].at<cv::Vec3b>(y, x).val[1];
                    texture.at<cv::Vec3b>(y - cropSize.y, x + offsetX - cropSize.x).val[2] = result->textures[imageIndex].at<cv::Vec3b>(y, x).val[2];
                }
            }
        }

        std::cout << "adapt coords " << imageIndex << std::endl;

        for (int j=0;j<(int)meshCloud->points.size();j++)
        {
            int textureIndex = result->textureIndex2[j];
            Eigen::Vector2f coords = result->textureCoordinates[j];

            if (textureIndex < imageIndex)
            {
                int oldWidth = offsetX;
                int newWidth = offsetX + (cropSize.width - cropSize.x);

                coords.x() = (coords.x() * oldWidth) / newWidth;
            }

            if (textureIndex == imageIndex)
            {
                int oldWidth = result->textures[imageIndex].cols;
                int newWidth = offsetX + (cropSize.width - cropSize.x);

                float x = coords.x() * oldWidth; //absolute in big source image
                x -= cropSize.x;

                float y = coords.y() * result->textures[imageIndex].rows;
                y -= cropSize.y;

                coords.x() = (x + offsetX) / newWidth;
                coords.y() = (y) / totalHeight;
            }

            result->textureCoordinates[j] = coords;
        }

        offsetX += (cropSize.width - cropSize.x);
        imageIndex++;
    }

    /*
    cv::Mat ycrcb;
    cv::cvtColor(texture,ycrcb,CV_RGB2YCrCb);

    cv::vector<cv::Mat> channels;
    cv::split(ycrcb,channels);

    cv::equalizeHist(channels[0], channels[0]);

    cv::Mat optimized;
    cv::merge(channels,ycrcb);

    cv::cvtColor(ycrcb, optimized, CV_YCrCb2RGB);
    */

    result->textures.clear();
    result->textures.push_back(texture);

    std::cout << "complete" << std::endl;


    /*
    cv::Mat copy;
    cv::cvtColor(texture, copy, CV_BGR2RGB);
    std::string file_name;
    file_name.append(texturePath);
    file_name.append("/texture.png");
    cv::imwrite(file_name, copy);

*/




/*

    //targetImages.resize(mesh->polygons.size());
    for (unsigned int triangleIndex=0;triangleIndex<mesh->polygons.size();triangleIndex++)
    {
        targetImages.push_back(-1);
    }

    //int forcedImage = 0;

    //hack

    for (int j=0;j<clouds.size();j++) // get the one that is used most
    {
        std::cout << "image counter " << imageCounter[j] << std::endl;
    }

    std::cout << "mesh polygon size " << mesh->polygons.size();

    for (int i=0;i<clouds.size();i++) // get best weights
    {
        int best = 0;
        int bestValue = 0;
        for (int j=0;j<clouds.size();j++) // get the one that is used most
        {
            if (imageCounter[j] > bestValue)
            {
                best = j;
                bestValue = imageCounter[j];
            }
        }

        //std::cout << "best image " << best << std::endl;

        if (imageCounter[best] != 0)
        {
            for (unsigned int triangleIndex=0;triangleIndex<mesh->polygons.size();triangleIndex++)
            {
                if(std::find(visibleImages[triangleIndex].begin(), visibleImages[triangleIndex].end(), best) != visibleImages[triangleIndex].end())
                {
                    //std::cout << "adding image for triangle " << triangleIndex << std::endl;
                    targetImages.at(triangleIndex) = best;

                    for (int j=0;j<visibleImages[triangleIndex].size();j++)
                    {
                        int index = visibleImages[triangleIndex][j];
                        imageCounter[index]--;
                    }
                    visibleImages[triangleIndex].clear();

                }
            }
        }
    }

    std::cout << "--------------------" << std::endl;

    for (int j=0;j<clouds.size();j++) // get the one that is used most
    {
        std::cout << "image counter " << imageCounter[j] << std::endl;
    }

    std::cout << "render textures" << std::endl;

    output::TexturedMesh::Ptr result(new output::TexturedMesh());
    result->mesh = mesh;
    result->textureCoordinates.resize(meshCloud->size());
    result->textureIndex.resize(mesh->polygons.size());
    result->textureIndex2.resize(meshCloud->size());

    std::cout << "setup coords" << std::endl;

    for (int i=0;i<meshCloud->size();i++)
    {
        result->textureCoordinates[i] = Eigen::Vector2f(0.0f, 0.0f);
        result->textureIndex2[i] = 0;
    }

    std::cout << "setup indices" << std::endl;

    for (int i=0;i<mesh->polygons.size();i++)
    {
        result->textureIndex[i] = 0;
    }

    std::cout << "create textures" << std::endl;
    std::cout << "cloud size " << clouds.size() << std::endl;
    std::cout << "poses size " << poses.size() << std::endl;

    // create textures
    for (int imageIndex=0;imageIndex<clouds.size() ;imageIndex++)
    {
        addTexture(meshCloud, result, clouds[imageIndex], poses[imageIndex], imageIndex, targetImages);
    }

    int totalWidth = 0;
    int totalHeight = 0;
    for (int imageIndex=0;imageIndex<clouds.size() ;imageIndex++)
    {
        totalWidth += result->textures[imageIndex].cols;
        totalHeight = std::max(totalHeight, result->textures[imageIndex].rows);
    }

    std::cout << "create new texture" << std::endl;
    cv::Mat3b texture(totalHeight, totalWidth, CV_8UC3);
    for (int x=0;x<totalWidth;x++)
    {
        for (int y=0;y<totalHeight;y++)
        {
            texture.at<cv::Vec3b>(y, x).val[0] = 0;
            texture.at<cv::Vec3b>(y, x).val[1] = 0;
            texture.at<cv::Vec3b>(y, x).val[2] = 0;
        }
    }

    int offsetX = 0;
    for (int imageIndex=0;imageIndex<clouds.size() ;imageIndex++)
    {
        std::cout << "copy texture " << imageIndex << std::endl;
        // copy texture
        for (int x=0;x<result->textures[imageIndex].cols;x++)
        {
            for (int y=0;y<result->textures[imageIndex].rows;y++)
            {
                texture.at<cv::Vec3b>(y, x + offsetX).val[0] = result->textures[imageIndex].at<cv::Vec3b>(y, x).val[0];
                texture.at<cv::Vec3b>(y, x + offsetX).val[1] = result->textures[imageIndex].at<cv::Vec3b>(y, x).val[1];
                texture.at<cv::Vec3b>(y, x + offsetX).val[2] = result->textures[imageIndex].at<cv::Vec3b>(y, x).val[2];
            }
        }

        std::cout << "adapt coords " << imageIndex << std::endl;

        for (int j=0;j<meshCloud->points.size();j++)
        {
            int textureIndex = result->textureIndex2[j];
            Eigen::Vector2f coords = result->textureCoordinates[j];

            if (textureIndex < imageIndex)
            {
                int oldWidth = offsetX;
                int newWidth = offsetX + result->textures[imageIndex].cols;

                coords.x() = (coords.x() * oldWidth) / newWidth;
            }

            if (textureIndex == imageIndex)
            {
                int oldWidth = result->textures[imageIndex].cols;
                int newWidth = offsetX + result->textures[imageIndex].cols;

                coords.x() = ((coords.x() * oldWidth) + offsetX) / newWidth;
                coords.y() = (coords.y() * result->textures[imageIndex].rows) / totalHeight;
            }

            result->textureCoordinates[j] = coords;
        }

        offsetX += result->textures[imageIndex].cols;
    }

    result->textures.clear();
    result->textures.push_back(texture);

    std::cout << "complete" << std::endl;
    */

    return result;
}

float MultibandTexture::calculateWeight(double angle, int b, double bestAngle)
{
    //return 1.0f;

    if (b == 0)
    {
        //std::cout << angle << " " << bestAngle << std::endl;

        if (angle < bestAngle + 0.0001f && angle > bestAngle - 0.0001f)
        {
            //std::cout << "take best angle" << std::endl;
            return M_PI / 2.0 - angle;
        }
        else
        {
            return 0.0f;
        }
    }
    else
    {
        //std::cout << "b > 0" << angle << " " << ((M_PI * b) / (2 * (pyramidDepth - 1))) << std::endl;

        if (angle < (M_PI * b) / (2 * (pyramidDepth - 1)))
        {
            return M_PI / 2.0 - angle;
        }
        else
        {
            return 0.0f;
        }
    }
}


void MultibandTexture::generatePyramids(Image *image)
{
    std::cout << "Create pyramid" << std::endl;

    for (int b=0;b<pyramidDepth;b++)
    {
        if (b == 0)
        {
            std::cout << "Create gaussian pyramid 0" << std::endl;

            cv::Mat3b copy;
            image->texture.copyTo(copy);

            cv::Mat3f copy_large;
            copy.convertTo(copy_large, CV_32FC3);
            image->gaussian.push_back(copy_large);
        }
        else
        {
            std::cout << "Create gaussian pyramid higher" << std::endl;

            cv::Mat3f blurred;
            cv::GaussianBlur(image->gaussian[b-1], blurred, cv::Size(kernelSize, kernelSize), std::pow(sigma, b+1));
            image->gaussian.push_back(blurred);
        }
    }

    for (int i=0;i<pyramidDepth;i++)
    {
        if (i<pyramidDepth-1)
        {
            std::cout << "Create laplacian pyramid 0" << std::endl;

            cv::Mat3f lap;
            lap = image->gaussian[i] - image->gaussian[i+1];
            image->laplacian.push_back(lap);
        }
        else
        {
            std::cout << "Create laplacian pyramid higher" << std::endl;

            image->laplacian.push_back(image->gaussian[i]);
        }
    }

    std::cout << "Create pyramid complete" << std::endl;
}

Eigen::Vector3f MultibandTexture::get3dPoint(int screen_x, int screen_y, Image *inputImage, Polygon *p)
{
    float angle_x, angle_y;
    //float range = 1.0f;
    inputImage->range_image.getAnglesFromImagePoint(screen_x, screen_y, angle_x, angle_y);

    //std::cout << image_x<<","<<image_y<<","<<range;
    //float cosY = cosf (angle_y);
    //Eigen::Vector3f point = Eigen::Vector3f (range * sinf (angle_x) * cosY, range * sinf (angle_y), range * cosf (angle_x)*cosY);
    //point = to_world_system_ * point;

    Eigen::Vector3f p1;
    inputImage->range_image.calculate3DPoint(screen_x, screen_y, 1.0f, p1);
    Eigen::Vector3f p2;
    inputImage->range_image.calculate3DPoint(screen_x, screen_y, 10.0f, p2);


    //std::cout << "S: " << point <<  std::endl;
    //std::cout << "T: " << point2 <<  std::endl;

    Eigen::Vector3f ray_start = p1;
    Eigen::Vector3f ray_end = p2;



    // intersection
    Eigen::Vector3f target;
    //Eigen::Vector3f cam_dir(camera.focal[0] - camera.pos[0], camera.focal[1] - camera.pos[1], camera.focal[2] - camera.pos[2]);
    //cam_dir.normalize();

    Eigen::ParametrizedLine<float, 3> line(ray_start, ray_start - ray_end);
    Eigen::Hyperplane<float, 3> plane(p->normal, p->point1);

    float d = line.intersection(plane);
    target = ray_start + (ray_start - ray_end) * d;

    return target;

    // get angles
}

void MultibandTexture::blendPixel(int x, int y, int min_x, int min_y, std::vector<Image> *inputImages, std::vector<Polygon> *polygons, int polyIndex, int imageIndex, bool isPadding)
{
    cv::Vec3f value(0.0f,0.0f,0.0f);//inputImages->at(imageIndex).gaussian[0].at<cv::Vec3f>(y, x);

    Polygon p = polygons->at(polyIndex);

    //int count = 1;

    //float targetAngle = p.angles[imageIndex];

    Eigen::Vector3f point3d;

    point3d = get3dPoint(x, y, &inputImages->at(imageIndex), &p);

    float bestAngle = 100.0f;

    for (int i=0;i<(int)inputImages->size();i++)
    {
        if (p.imageVisibility[i] == true)
        {
            if (p.angles[i] < bestAngle)
            {
                bestAngle = p.angles[i];
            }
        }
    }

    //bestAngle = p.angles[imageIndex];

    for (int b=0;b<pyramidDepth;b++)
    {
        cv::Vec3f levelValue(0.0f,0.0f,0.0f);
        float weightSum = 0.0f;

        for (int i=0;i<(int)inputImages->size();i++)
        {
            if (p.imageVisibility[i] == true)
            {
                pcl::RangeImagePlanar &ri_dst = inputImages->at(i).range_image;

                float x_new, y_new, r;
                ri_dst.getImagePoint(point3d, x_new, y_new, r);

                if (imageIndex == i)
                {
                    x_new = x;
                    y_new = y;
                }

                if (x_new >= 0 && x_new < inputImages->at(i).gaussian[0].cols
                        && y_new >= 0 && y_new < inputImages->at(i).gaussian[0].rows)
                {
                    //if (fabs(targetAngle - p.angles[i]) < (10 * M_PI) / 180.0)
                    //{
                        //std::cout << "Image: " << i << " old: " << x << " " << y << " New: " << (int) x_new << " " << (int) y_new << std::endl;

                    //std::cout << "angles: " << p.angles[i] << " " << bestAngle << std::endl;

                    float weight = calculateWeight(p.angles[i]/* - bestAngle*/, b, bestAngle);

                    cv::Mat patch;
                    cv::getRectSubPix(inputImages->at(i).laplacian[b], cv::Size(1,1), cv::Point2f(x_new, y_new), patch);

                    //cv::Vec3f value2 = inputImages->at(i).laplacian[b].at<cv::Vec3f>((int) y_new, (int) x_new);
                    cv::Vec3f value2 = patch.at<cv::Vec3f>(0,0);
                        //std::cout << "Value dest " << i << ": " << value2[0] << " " << value2[1] << " " << value2[2] << std::endl;

                    levelValue += weight * value2;
                    weightSum += weight;
                        //count++;
                    //}

                }
            }
        }

        //std::cout << "value bef: " << value << std::endl;
        //std::cout << "levl val: " << levelValue << std::endl;
        //std::cout << "weighted sum: " << weightSum << std::endl;

        if (weightSum > 0.0f)
        {
            value += levelValue / weightSum;
        }

        //std::cout << "value after: " << value << std::endl;
    }

    //std::cout << "Value2: " << value << std::endl;

    //value /= (float) count;

    /*
    if (value.val[0] > 255.0f || value.val[1] > 255.0f || value.val[2] > 255.0f)
    {
        std::cout << "Less than zero: " << value << std::endl;
        value.val[0] = 0;
        value.val[1] = 255;
        value.val[2] = 0;
    }
    */

    if (value.val[0] < 0.0f) value.val[0] = 0.0f;
    if (value.val[1] < 0.0f) value.val[1] = 0.0f;
    if (value.val[2] < 0.0f) value.val[2] = 0.0f;

    if (value.val[0] > 255.0f) value.val[0] = 255.0f;
    if (value.val[1] > 255.0f) value.val[1] = 255.0f;
    if (value.val[2] > 255.0f) value.val[2] = 255.0f;

    if (inputImages->at(imageIndex).texture_blending_state.at<uchar>(y, x) == 0) // not mapped yet
    {
        if (isPadding)
        {
            inputImages->at(imageIndex).texture.at<cv::Vec3b>(y - min_y, x - min_x).val[0] = value.val[2];
            inputImages->at(imageIndex).texture.at<cv::Vec3b>(y - min_y, x - min_x).val[1] = value.val[1];
            inputImages->at(imageIndex).texture.at<cv::Vec3b>(y - min_y, x - min_x).val[2] = value.val[0];
        }
        else
        {
            inputImages->at(imageIndex).texture_blending_state.at<uchar>(y, x) = 1;

            inputImages->at(imageIndex).texture.at<cv::Vec3b>(y - min_y, x - min_x).val[0] = value.val[2];
            inputImages->at(imageIndex).texture.at<cv::Vec3b>(y - min_y, x - min_x).val[1] = value.val[1];
            inputImages->at(imageIndex).texture.at<cv::Vec3b>(y - min_y, x - min_x).val[2] = value.val[0];
        }
    }
    else
    {
        if (isPadding)
        {
            // do nothing
        }
        else
        {
            // do nothing
        }
    }
}

void MultibandTexture::setPixel(int x, int y, int min_x, int min_y, std::vector<Image> *inputImages, std::vector<Polygon> *polygons, int polyIndex, int imageIndex, bool isPadding)
{
    cv::Vec3f value = inputImages->at(imageIndex).gaussian[0].at<cv::Vec3f>((int) y, (int) x);

    if (inputImages->at(imageIndex).texture_blending_state.at<uchar>(y, x) == 0) // not mapped yet
    {
        if (isPadding)
        {
            inputImages->at(imageIndex).texture.at<cv::Vec3b>(y - min_y, x - min_x).val[0] = value.val[2];
            inputImages->at(imageIndex).texture.at<cv::Vec3b>(y - min_y, x - min_x).val[1] = value.val[1];
            inputImages->at(imageIndex).texture.at<cv::Vec3b>(y - min_y, x - min_x).val[2] = value.val[0];
        }
        else
        {
            inputImages->at(imageIndex).texture_blending_state.at<uchar>(y, x) = 1;

            inputImages->at(imageIndex).texture.at<cv::Vec3b>(y - min_y, x - min_x).val[0] = value.val[2];
            inputImages->at(imageIndex).texture.at<cv::Vec3b>(y - min_y, x - min_x).val[1] = value.val[1];
            inputImages->at(imageIndex).texture.at<cv::Vec3b>(y - min_y, x - min_x).val[2] = value.val[0];
        }
    }
}

void MultibandTexture::addTexture(pcl::PointCloud<pcl::PointXYZ>::Ptr meshCloud, TexturedMesh::Ptr result, std::vector<Image> *inputImages, int imageIndex, std::vector<Polygon> *polygons, std::set<int> imageSubset)
{
    std::cout << "create texture for image " << imageIndex << std::endl;

    int max_x = inputImages->at(imageIndex).cloud->width;
    int max_y = inputImages->at(imageIndex).cloud->height;
    int min_x = 0;
    int min_y = 0;

    for (int i=0;i<(int)inputImages->at(imageIndex).cloud->width;i++)
    {
        for (int j=0;j<(int)inputImages->at(imageIndex).cloud->height;j++)
        {
            if (!inputImages->at(imageIndex).range_image.isInImage(i, j) || !inputImages->at(imageIndex).range_image.isValid(i, j))
            {
                //pcl::PointXYZRGB &p = inputImages->at(imageIndex).cloud->at(i, j);
                //p.r = 0;
                //p.g = 255;
                //p.b = 0;
            } else {
                //if (i < min_x) min_x = i;
                //if (j < min_y) min_y = j;
                //if (i > max_x) max_x = i;
                //if (j > max_y) max_y = j;
            }
        }
    }

    max_x += (max_x - min_x) % 8;
    max_y += (max_y - min_y) % 8;

    //min_x = 0;
    //max_x = image->width;
    //min_y = 0;
    //max_y = image->height;


    std::cout << "create texture " << min_x << " " << max_x << " " << min_y << " " << max_y << std::endl;

    cv::Mat3b texture(max_y - min_y, max_x - min_x, CV_8UC3);
    texture.setTo(cv::Scalar(0,0,0));

    //cv::bilateralFilter(texture, texture_filtered, -1, 20.0, 20.0);

    result->textures.push_back(texture);
    inputImages->at(imageIndex).texture = texture;

    std::cout << "calc texture coords" << std::endl;



    int size = result->mesh->polygons.size();
    for (int polyIndex=0;polyIndex<size;polyIndex++)
    {
        pcl::Vertices vertices = result->mesh->polygons[polyIndex];

        int targetImage = -1;
        double bestAngle = 100.0;
        double worstAngle = 0.0;
        std::set<int>::iterator it;

        for (it = imageSubset.begin(); it != imageSubset.end(); ++it)
        {
            if (polygons->at(polyIndex).imageVisibility[*it])
            {

                // calculate normal of polygon
                //pcl::Vertices vertices = mesh->polygons[polyIndex];
                //Eigen::Vector3f triangle_normal = calculateNormal(meshCloud, vertices/*, poses[*it]*/);
                //triangle_normal.normalize();

                // camera orientation
                //Eigen::Vector3f cam_vector = getCamVector(image, poses[*it]); //Eigen::Vector3f(0.0f, 0.0f, -1.0f);//cloud->sensor_orientation_.normalized().vec();
                //cam_vector.normalize();

                double angle = polygons->at(polyIndex).angles[*it]; //cam_vector.dot(triangle_normal);

                //angle = fabs(acos(angle));

                if (angle < bestAngle)
                {
                    bestAngle = angle;
                    targetImage = *it;
                }

                if (angle > worstAngle)
                {
                    worstAngle = angle;
                }
            }
        }

        polygons->at(polyIndex).targetTexture = targetImage;

        if (targetImage == imageIndex) {

            std::vector<Eigen::Vector2f> pointsInImage;

            for (int vert_i=0;vert_i<(int)vertices.vertices.size();vert_i++)
            {
                pcl::PointXYZ pclPoint = meshCloud->points[vertices.vertices[vert_i]];
                Eigen::Vector3f p(pclPoint.x, pclPoint.y, pclPoint.z);

                float u,v,r;
                inputImages->at(imageIndex).range_image.getImagePoint(p, u, v, r);

                pointsInImage.push_back(Eigen::Vector2f(u, v));

                //std::cout << "Triangle point: " << u << " / " << v << " - " << pclPoint.x << " / " << pclPoint.y << " / " << pclPoint.z << std::endl;
            }

            // sort by x value
            if (pointsInImage[0][0] > pointsInImage[1][0]) {
                Eigen::Vector2f t = pointsInImage[0];
                pointsInImage[0] = pointsInImage[1];
                pointsInImage[1] = t;
            }

            if (pointsInImage[1][0] > pointsInImage[2][0]) {
                Eigen::Vector2f t = pointsInImage[1];
                pointsInImage[1] = pointsInImage[2];
                pointsInImage[2] = t;
            }

            if (pointsInImage[0][0] > pointsInImage[1][0]) {
                Eigen::Vector2f t = pointsInImage[0];
                pointsInImage[0] = pointsInImage[1];
                pointsInImage[1] = t;
            }

            // get triangle bounds
            int t_min_x = floor(pointsInImage[0][0]);
            int t_max_x = ceil(pointsInImage[2][0]);

            float t_min = 99999.0;
            float t_max = 0.0;

            for (int i=0;i<(int)pointsInImage.size();i++)
            {
                if (pointsInImage[i][1] < t_min)
                {
                    t_min = pointsInImage[i][1];
                }

                if (pointsInImage[i][1] > t_max)
                {
                    t_max = pointsInImage[i][1];
                }
            }

            int t_min_y = floor(t_min);
            int t_max_y = ceil(t_max);

            // check if pixel is in triangle
            float k_up_left = (pointsInImage[1][1] - pointsInImage[0][1]) / ((pointsInImage[1][0] - pointsInImage[0][0]));
            float k_down_left = (pointsInImage[2][1] - pointsInImage[0][1]) / ((pointsInImage[2][0] - pointsInImage[0][0]));

            float k_up_right = (pointsInImage[2][1] - pointsInImage[1][1]) / ((pointsInImage[2][0] - pointsInImage[1][0]));


            //std::cout << "Proj: " << pointsInImage[0] << " " << pointsInImage[1] << " " << pointsInImage[2] << " K: " << k_up_left << " " << k_down_left << " " << k_up_right << std::endl;

            //std::cout << "min: " << t_min_x << " " << t_max_x << " " << t_min_y << " " << t_max_y << std::endl;

            t_min_x = std::max(t_min_x-padding, min_x);
            t_min_y = std::max(t_min_y-padding, min_y);

            t_max_x = std::min(t_max_x+padding, inputImages->at(imageIndex).texture.cols + min_x - 1);
            t_max_y = std::min(t_max_y+padding, inputImages->at(imageIndex).texture.rows + min_y - 1);

            for (int x=t_min_x;x<=t_max_x;x++)
            {
                for (int y=t_min_y;y<=t_max_y;y++)
                {
                    float top=0;
                    float bottom=0;

                    if (x < pointsInImage[1][0])
                    {
                        top = pointsInImage[0][1] + (x - pointsInImage[0][0]) * k_up_left;
                        bottom = pointsInImage[0][1] + (x - pointsInImage[0][0]) * k_down_left;
                    }
                    if (x > pointsInImage[1][0])
                    {
                        top = pointsInImage[1][1] + (x - pointsInImage[1][0]) * k_up_right;
                        bottom = pointsInImage[0][1] + (x - pointsInImage[0][0]) * k_down_left;
                    }
                    if (x == pointsInImage[1][0])
                    {
                        top = pointsInImage[1][1];
                        bottom = pointsInImage[0][1] + (x - pointsInImage[0][0]) * k_down_left;
                    }

                    if (top > bottom) {
                        float t = top;
                        top = bottom;
                        bottom = t;
                    }

                    if (x + padding > pointsInImage[0][0] && x - padding < pointsInImage[2][0]
                            && y + padding > top && y - padding < bottom) {

                        //std::cout << "   match: " << x << " " << y << " " << top << " " << bottom << " !!!" << std::endl;

                        bool isNotPadding = x > pointsInImage[0][0] && x < pointsInImage[2][0]
                                && y > top && y < bottom;

                        if (enableBlending)
                        {
                            blendPixel(x, y, min_x, min_y, inputImages, polygons, polyIndex, imageIndex, !isNotPadding);
                        } else {
                            setPixel(x, y, min_x, min_y, inputImages, polygons, polyIndex, imageIndex, !isNotPadding);
                        }

                        if (inputImages->at(imageIndex).croppedSize.x > x) inputImages->at(imageIndex).croppedSize.x = x;
                        if (inputImages->at(imageIndex).croppedSize.y > y) inputImages->at(imageIndex).croppedSize.y = y;
                        if (inputImages->at(imageIndex).croppedSize.width < x) inputImages->at(imageIndex).croppedSize.width = x;
                        if (inputImages->at(imageIndex).croppedSize.height < y) inputImages->at(imageIndex).croppedSize.height = y;

                    } else {

                        //std::cout << "no match: " << x << " " << y << " " << top << " " << bottom << std::endl;
                    }
                }
            }
        }


        for (int vert_i=0;vert_i<(int)vertices.vertices.size();vert_i++)
        {
            // multiple texture coordinates for one vertex?
            //int targetImage = bestImages[triangleIndex];


            //std::cout << "best image: " << targetImage << std::endl;

            if (targetImage == imageIndex || targetImage == -1)
            {
                //std::cout << "adding coordinates" << std::endl;
                pcl::PointXYZ pclPoint = meshCloud->points[vertices.vertices[vert_i]];
                Eigen::Vector3f p(pclPoint.x, pclPoint.y, pclPoint.z);
                float u,v,r;

                inputImages->at(imageIndex).range_image.getImagePoint(p, u, v, r);
                u -= min_x;
                v -= min_y;

                if (targetImage == -1)
                {
                    u = 0;
                    v = texture.rows + 40;
                }

                if (result->textureCoordinates[vertices.vertices[vert_i]].x() != 0.0f || result->textureCoordinates[vertices.vertices[vert_i]].y() != 0.0f)
                {
                    if (result->textureIndex2[vertices.vertices[vert_i]] == (int)result->textures.size() - 1) // same texture patch
                    {
                        //std::cout << "Same texture patch: " << result->textureCoordinates[vertices.vertices[vert_i]].x() << " - " << u / texture.cols << std::endl;
                    }
                    else
                    {

                        if (targetImage != -1)
                        {
                            //std::cout << "clouds size " << meshCloud->size();
                            // duplicate point
                            meshCloud->push_back(pclPoint);
                            result->textureCoordinates.push_back(Eigen::Vector2f(Eigen::Vector2f(u / texture.cols, v / texture.rows)));
                            result->textureIndex2.push_back(result->textures.size() - 1);
                            unsigned int newIndex = meshCloud->size() - 1;
                            //std::cout << "new index " << newIndex << " tex coord size " << result->textureCoordinates.size() << std::endl;
                            //result->mesh->polygons[polyIndex].vertices[vert_i] = newIndex;

                            //pcl::Vertices newPoly;
                            //newPoly.vertices = vertices.vertices;
                            //result->mesh->polygons.push_back(newPoly);

                            result->mesh->polygons[polyIndex].vertices[vert_i] = newIndex;
                        }
                    }
                }
                else
                {
                    result->textureCoordinates[vertices.vertices[vert_i]] = Eigen::Vector2f(u / texture.cols, v / texture.rows);

                    result->textureIndex2[vertices.vertices[vert_i]] = result->textures.size() - 1;
                }

                result->textureIndex[polyIndex] = result->textures.size() - 1;
            }
        }
    }

    std::cout << "Mesh cloud size after texturing " << meshCloud->size() << std::endl;

    pcl::toPCLPointCloud2(*meshCloud, result->mesh->cloud);

    /*
    std::string file_name;
    file_name.append("/home/alex/Arbeitsflaeche/test/texture_");
    file_name.append(boost::lexical_cast<std::string>(imageIndex));
    file_name.append(".png");
    pcl::io::savePNGFile(file_name, *image, "rgb");
    */

    inputImages->at(imageIndex).croppedSize.x -= 4;
    inputImages->at(imageIndex).croppedSize.y -= 4;
    inputImages->at(imageIndex).croppedSize.width += 4;
    inputImages->at(imageIndex).croppedSize.height += 4;

    inputImages->at(imageIndex).croppedSize.width += (inputImages->at(imageIndex).croppedSize.width - inputImages->at(imageIndex).croppedSize.x) % 8;
    inputImages->at(imageIndex).croppedSize.height += (inputImages->at(imageIndex).croppedSize.height - inputImages->at(imageIndex).croppedSize.y) % 8;
}

pcl::RangeImagePlanar MultibandTexture::getMeshProjection(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Matrix4f pose)
{

    // Image size. Both Kinect and Xtion work at 640x480.
    int imageSizeX = 640;
    int imageSizeY = 480;
    // Center of projection. here, we choose the middle of the image.
    float centerX = 640.0f / 2.0f;
    float centerY = 480.0f / 2.0f;
    // Focal length. The value seen here has been taken from the original depth images.
    // It is safe to use the same value vertically and horizontally.
    float focalLengthX = focal_length;//, focalLengthY = focalLengthX;
    // Sensor pose. Thankfully, the cloud includes the data.

    Eigen::Affine3f sensorPose = Eigen::Affine3f(Eigen::Translation3f(cloud->sensor_origin_[0],
                                 cloud->sensor_origin_[1],
                                 cloud->sensor_origin_[2])) *
                                 Eigen::Affine3f(cloud->sensor_orientation_);
    sensorPose = sensorPose * pose;

    // Noise level. If greater than 0, values of neighboring points will be averaged.
    // This would set the search radius (i.e., 0.03 == 3cm).
    float noiseLevel = 0.0f;
    // Minimum range. If set, any point closer to the sensor than this will be ignored.
    float minimumRange = 0.0f;

    // Planar range image object.
    pcl::RangeImagePlanar rangeImagePlanar;
    rangeImagePlanar.createFromPointCloudWithFixedSize(*cloud, imageSizeX, imageSizeY,
            centerX, centerY, focalLengthX, focalLengthX,
            sensorPose, pcl::RangeImage::CAMERA_FRAME,
            noiseLevel, minimumRange);

    return rangeImagePlanar;
}

Eigen::Vector3f MultibandTexture::calculateNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr meshCloud, pcl::Vertices vertices/*, Eigen::Matrix4f pose*/)
{
    /*
    pcl::PointXYZRGBNormal p1 = meshCloud->points[vertices.vertices[0]];
    pcl::PointXYZRGBNormal p2 = meshCloud->points[vertices.vertices[1]];
    pcl::PointXYZRGBNormal p3 = meshCloud->points[vertices.vertices[2]];

    Eigen::Vector3f v1 = Eigen::Vector3f( p1.getNormalVector3fMap() );
    Eigen::Vector3f v2 = Eigen::Vector3f( p2.getNormalVector3fMap() );
    Eigen::Vector3f v3 = Eigen::Vector3f( p3.getNormalVector3fMap() );



    return (v1 + v2 + v3).normalized();
    */

    pcl::PointXYZ p1 = meshCloud->points[vertices.vertices[0]];
    pcl::PointXYZ p2 = meshCloud->points[vertices.vertices[1]];
    pcl::PointXYZ p3 = meshCloud->points[vertices.vertices[2]];

    /*
    Eigen::Matrix4f inv = pose.inverse();

    p1 = pcl::transformPoint(p1, Eigen::Affine3f(inv));
    p2 = pcl::transformPoint(p2, Eigen::Affine3f(inv));
    p3 = pcl::transformPoint(p3, Eigen::Affine3f(inv));
    */

    Eigen::Vector3f v1 = Eigen::Vector3f( p1.getArray3fMap() );
    Eigen::Vector3f v2 = Eigen::Vector3f( p2.getArray3fMap() );
    Eigen::Vector3f v3 = Eigen::Vector3f( p3.getArray3fMap() );

    v2 = v1 - v2;
    v3 = v1 - v3;
    v2.normalize();
    v3.normalize();

    return v2.cross(v3);
}





/*

#include "texturing/pclTexture.h"

#include <boost/lexical_cast.hpp>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/io/io.h>

#include <pcl/search/kdtree.h>

#include <math.h>

#include <pcl/io/file_io.h>
#include <pcl/io/png_io.h>

#include <opencv2/core/core.hpp>

namespace object_modeller
{
namespace texturing
{

PclTexture::PclTexture(std::string config_name) : InOutModule(config_name)
{
    angleThresh = M_PI;
}

output::TexturedMesh::Ptr PclTexture::process(boost::tuples::tuple<pcl::PolygonMesh::Ptr, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>, std::vector<Eigen::Matrix4f> > input)
{
    std::cout << "processing pcl texture" << std::endl;

    pcl::PolygonMesh::Ptr mesh = boost::tuples::get<0>(input);
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds = boost::tuples::get<1>(input);
    std::vector<Eigen::Matrix4f> poses = boost::tuples::get<2>(input);

    std::cout << "got input" << std::endl;

    std::cout << "mesh size " << mesh->polygons.size() << std::endl;

    // contains for each polygon a list of images, that can see this polygon
    std::vector<int> visibleImages[mesh->polygons.size()];
    //std::vector<std::vector<int> > visibleImages;
    //visibleImages.resize(mesh->polygons.size());

    std::cout << "created vector" << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr meshCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2 (mesh->cloud, *meshCloud);

    std::cout << "copied cloud" << std::endl;

    for (int triangleIndex=0;triangleIndex<mesh->polygons.size();triangleIndex++)
    {
        //std::cout << "iterate triangles " << triangleIndex << " of " << mesh->polygons.size() << std::endl;
        double bestAngle = 100.0;
        int bestAngleIndex;

        for (int imageIndex=0;imageIndex<clouds.size();imageIndex++)
        {
            //std::cout << "iterate images " << imageIndex << " of " << clouds.size() << std::endl;

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = clouds[imageIndex];

            pcl::Vertices vertices = mesh->polygons[triangleIndex];

            Eigen::Vector3f triangle_normal = calculateNormal(meshCloud, vertices);

            Eigen::Vector3f cam_vector = cloud->sensor_orientation_.normalized().vec();

            double angle = cam_vector.dot(triangle_normal);

            if (angle < bestAngle)
            {
                bestAngle = angle;
                bestAngleIndex = imageIndex;
            }

            if (angle < angleThresh)
            {
                // take the best at the moment
                //visibleImages[triangleIndex].push_back(imageIndex);
            }

            //double test = cloud->sensor_orientation_.dot(normal);

        }

        if (visibleImages[triangleIndex].size() == 0)
        {
            visibleImages[triangleIndex].push_back(bestAngleIndex);
        }
    }

    std::cout << "render textures" << std::endl;

    output::TexturedMesh::Ptr result(new output::TexturedMesh());
    result->mesh = mesh;
    result->textureCoordinates.resize(mesh->cloud.data.size());
    result->textureIndex.resize(mesh->polygons.size());

    std::cout << "setup coords" << std::endl;

    for (int i=0;i<mesh->cloud.data.size();i++)
    {
        result->textureCoordinates[i] = Eigen::Vector2f(0.0f, 0.0f);
    }

    std::cout << "setup indices" << std::endl;

    for (int i=0;i<mesh->polygons.size();i++)
    {
        result->textureIndex[i] = 0;
    }

    std::cout << "create textures" << std::endl;
    std::cout << "cloud size " << clouds.size() << std::endl;
    std::cout << "poses size " << poses.size() << std::endl;

    // create textures
    for (int imageIndex=0;imageIndex<1;imageIndex++)
    {
        addTexture(result, clouds[imageIndex], poses[imageIndex], imageIndex, visibleImages);
    }

    std::cout << "complete" << std::endl;

    return result;
}

void PclTexture::addTexture(output::TexturedMesh::Ptr result, pcl::PointCloud<pcl::PointXYZRGB>::Ptr image, Eigen::Matrix4f pose, int imageIndex, std::vector<int> *bestImages)
{
    std::cout << "create texture" << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr meshCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2 (result->mesh->cloud, *meshCloud);

    pcl::RangeImagePlanar rangeImage = getMeshProjection(meshCloud, image, pose);

    int min_x = image->width;
    int min_y = image->height;
    int max_x = 0;
    int max_y = 0;

    for (int i=0;i<image->width;i++)
    {
        for (int j=0;j<image->height;j++)
        {
            if (!rangeImage.isInImage(i, j) || !rangeImage.isValid(i, j))
            {
                pcl::PointXYZRGB &p = image->at(i, j);
                p.r = 0;
                p.g = 0;
                p.b = 0;
            } else {
                if (i < min_x) min_x = i;
                if (j < min_y) min_y = j;
                if (i > max_x) max_x = i;
                if (j > max_y) max_y = j;
            }
        }
    }

    max_x += (max_x - min_x) % 8;
    max_y += (max_y - min_y) % 8;

    //min_x = 0;
    //max_x = image->width;
    //min_y = 0;
    //max_y = image->height;


    std::cout << "create texture " << min_x << " " << max_x << " " << min_y << " " << max_y << std::endl;

    cv::Mat3b texture(max_y - min_y, max_x - min_x, CV_8UC3);

    for (int i=min_x;i<max_x;i++)
    {
        for (int j=min_y;j<max_y;j++)
        {
            texture.at<cv::Vec3b>(j - min_y, i - min_x).val[0] = image->at(i, j).r;
            texture.at<cv::Vec3b>(j - min_y, i - min_x).val[1] = image->at(i, j).g;
            texture.at<cv::Vec3b>(j - min_y, i - min_x).val[2] = image->at(i, j).b;
        }
    }


    result->textures.push_back(texture);


    std::cout << "calc texture coords" << std::endl;


    // calculate texture coordinates

    for (int triangleIndex=0;triangleIndex<result->mesh->polygons.size();triangleIndex++)
    {
        pcl::Vertices vertices = result->mesh->polygons[triangleIndex];

        for (int vert_i=0;vert_i<vertices.vertices.size();vert_i++)
        {
            // multiple texture coordinates for one vertex?
            int targetImage = bestImages[triangleIndex][0];

            if (targetImage == imageIndex)
            {
                pcl::PointXYZ pclPoint = meshCloud->points[vertices.vertices[vert_i]];
                Eigen::Vector3f p(pclPoint.x, pclPoint.y, pclPoint.z);
                float u,v,r;

                rangeImage.getImagePoint(p, u, v, r);
                u -= min_x;
                v -= min_y;

                //std::cout << "image point " << u << " - " << v << std::endl;

                result->textureCoordinates[vertices.vertices[vert_i]] = Eigen::Vector2f(u / texture.cols, v / texture.rows);

                //std::cout << "coords " << "\n" << result->textureCoordinates[vertices.vertices[vert_i]] << std::endl;

                result->textureIndex[triangleIndex] = targetImage;
            }
        }
    }
}

pcl::RangeImagePlanar PclTexture::getMeshProjection(pcl::PointCloud<pcl::PointXYZ>::Ptr meshCloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Eigen::Matrix4f pose)
{

    // Image size. Both Kinect and Xtion work at 640x480.
    int imageSizeX = 640;
    int imageSizeY = 480;
    // Center of projection. here, we choose the middle of the image.
    float centerX = 640.0f / 2.0f;
    float centerY = 480.0f / 2.0f;
    // Focal length. The value seen here has been taken from the original depth images.
    // It is safe to use the same value vertically and horizontally.
    float focalLengthX = 525.0f, focalLengthY = focalLengthX;
    // Sensor pose. Thankfully, the cloud includes the data.
    Eigen::Affine3f sensorPose = Eigen::Affine3f(Eigen::Translation3f(cloud->sensor_origin_[0],
                                 cloud->sensor_origin_[1],
                                 cloud->sensor_origin_[2])) *
                                 Eigen::Affine3f(cloud->sensor_orientation_);

    sensorPose = sensorPose * pose;
    // Noise level. If greater than 0, values of neighboring points will be averaged.
    // This would set the search radius (i.e., 0.03 == 3cm).
    float noiseLevel = 0.0f;
    // Minimum range. If set, any point closer to the sensor than this will be ignored.
    float minimumRange = 0.0f;

    // Planar range image object.
    pcl::RangeImagePlanar rangeImagePlanar;
    rangeImagePlanar.createFromPointCloudWithFixedSize(*meshCloud, imageSizeX, imageSizeY,
            centerX, centerY, focalLengthX, focalLengthX,
            sensorPose, pcl::RangeImage::CAMERA_FRAME,
            noiseLevel, minimumRange);

    return rangeImagePlanar;
}

Eigen::Vector3f PclTexture::calculateNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr meshCloud, pcl::Vertices vertices)
{
    pcl::PointXYZ p1 = meshCloud->points[vertices.vertices[0]];
    pcl::PointXYZ p2 = meshCloud->points[vertices.vertices[1]];
    pcl::PointXYZ p3 = meshCloud->points[vertices.vertices[2]];

    Eigen::Vector3f v1 = Eigen::Vector3f( p1.getArray3fMap() );
    Eigen::Vector3f v2 = Eigen::Vector3f( p2.getArray3fMap() );
    Eigen::Vector3f v3 = Eigen::Vector3f( p3.getArray3fMap() );

    v2 = v2 - v1;
    v3 = v3 - v1;

    return v2.cross(v3);
}

}
}



  */

}
}
