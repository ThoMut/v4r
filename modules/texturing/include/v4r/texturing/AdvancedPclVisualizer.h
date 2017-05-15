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

#include <pcl/visualization/pcl_visualizer.h>

#include <opencv2/opencv.hpp>

#include "TexturedMesh.h"

namespace v4r
{
namespace texturing
{


/** \author Khai Tran */
  struct TexMaterial
  {
    TexMaterial () : tex_name (), tex_Ka (), tex_Kd (), tex_Ks (), tex_d (), tex_Ns (), tex_illum () {}

    struct RGB
    {
      float r;
      float g;
      float b;
    }; //RGB

    cv::Mat3b data;

    /** \brief Texture name. */
    std::string tex_name;

    /** \brief Texture file. */
    //std::string tex_file;

    /** \brief Defines the ambient color of the material to be (r,g,b). */
    RGB         tex_Ka;

    /** \brief Defines the diffuse color of the material to be (r,g,b). */
    RGB         tex_Kd;

    /** \brief Defines the specular color of the material to be (r,g,b). This color shows up in highlights. */
    RGB         tex_Ks;

    /** \brief Defines the transparency of the material to be alpha. */
    float       tex_d;

    /** \brief Defines the shininess of the material to be s. */
    float       tex_Ns;

    /** \brief Denotes the illumination model used by the material.
      *
      * illum = 1 indicates a flat material with no specular highlights, so the value of Ks is not used.
      * illum = 2 denotes the presence of specular highlights, and so a specification for Ks is required.
      */
    int         tex_illum;
  }; // TexMaterial


  /** \author Khai Tran */
  struct TextureMesh
  {
    TextureMesh () :
      cloud (), tex_polygons (), tex_coordinates (), tex_materials () {}

    pcl::PCLPointCloud2  cloud;
    pcl::PCLHeader  header;


    std::vector<std::vector<pcl::Vertices> >    tex_polygons;     // polygon which is mapped with specific texture defined in TexMaterial
    std::vector<std::vector<Eigen::Vector2f> >  tex_coordinates;  // UV coordinates
    std::vector<TexMaterial>               tex_materials;    // define texture material

    public:
      typedef boost::shared_ptr<TextureMesh> Ptr;
      typedef boost::shared_ptr<TextureMesh const> ConstPtr;
   }; // struct TextureMesh


class AdvancedPclVisualizer : public pcl::visualization::PCLVisualizer
{
public:
    AdvancedPclVisualizer(int &argc, char **argv, const std::string &name, pcl::visualization::PCLVisualizerInteractorStyle* style, const bool create_interactor)
        : PCLVisualizer (argc, argv, name, style, create_interactor) {}

    int textureFromTexMaterial (const TexMaterial& tex_mat, vtkTexture* vtk_tex) const;
    bool addTextureMesh (TexturedMesh::Ptr mesh, const std::string &id, int viewport);
    void addActorToRenderer2 (const vtkSmartPointer<vtkProp> &actor, int viewport);
private:
    bool addTextureMesh (const TextureMesh &mesh, const std::string &id, int viewport);
};

}
}
