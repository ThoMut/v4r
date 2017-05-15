
/**
 * $Id$
 *
 * Software License Agreement (GNU General Public License)
 *
 *  Copyright (C) 2015:
 *
 *    Johann Prankl, prankl@acin.tuwien.ac.at
 *    Aitor Aldoma Buchaca, aldoma@acin.tuwien.ac.at
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
 * @author Johann Prankl, Aitor Aldoma Buchaca
 *
 */

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string>

#include <pcl/common/transforms.h>

#include <pcl/console/parse.h>

#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>

#include <v4r/texturing/reader.h>
#include <v4r/texturing/renderer.h>
#include <v4r/texturing/surface.h>
#include <v4r/texturing/multiband.h>

#include <v4r/texturing/export.h>
#include <v4r/io/filesystem.h>


// parameters

// General
std::string base_path; //("/home/thomas/DA/data/texture_models");
//std::string sub_dir("");
//std::string sub_dir_models("");
//std::string sub_dir_text_meshes("textured_meshes");
std::string pattern_cloud(".*cloud.*.pcd");
std::string pattern_poses(".*pose.*.txt");
std::string model_name("3D_model.pcd");
int step = 1;

// poisson
int poisson_depth = 10;
int poisson_nr_samples = 20;
bool crop_model = true;

// texturing
int tex_angle_tresh = 25;
bool tex_enable_blending = true;
int tex_padding = 4;
float focal_length = 525.0f;

void printUsage(int argc, char **argv)
{
    std::cout << std::endl;
    std::cout << std::endl;
    std::cout << "Usage: " << argv[0] << " base-path <options>" << std::endl;
    std::cout << std::endl;
    std::cout << "base-path where models and the recognition_structure are located" << std::endl;
    std::cout << std::endl;
    std::cout << "Options: " << std::endl;
    std::cout << std::endl;
    std::cout << "-pattern-cloud = Pattern for source cloud files (default: .*cloud.*.pcd)" << std::endl;
    std::cout << "-pattern-poses = Pattern for pose files (default: .*pose.*.txt)" << std::endl;
    std::cout << "-model-name = File name of the model file (default: modelname.pcd)" << std::endl;
    std::cout << std::endl;
    std::cout << "-poisson-depth = Depth of the Poisson reconstruction octree (higher -> more detail, lower -> faster, default=10)" << std::endl;
    std::cout << "-poisson-nr-samples = Nr of samples per node in the octree, smoothes the output (1 = no smoothing, higher values -> more smoothing, default=20)" << std::endl;
    std::cout << "-poisson-crop = Crop mesh using convex hull" << std::endl;
    std::cout << std::endl;

}

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        printUsage(argc, argv);
        return -1;
    }

    base_path = argv[1];

    pcl::console::parse_argument(argc, argv, "-step", step);
    pcl::console::parse_argument(argc, argv, "-pattern-cloud", pattern_cloud);
    pcl::console::parse_argument(argc, argv, "-pattern-poses", pattern_poses);
    pcl::console::parse_argument(argc, argv, "-model-name", model_name);

    pcl::console::parse_argument(argc, argv, "-poisson-depth", poisson_depth);
    pcl::console::parse_argument(argc, argv, "-poisson-nr-samples", poisson_nr_samples);
    pcl::console::parse_argument(argc, argv, "-poisson-crop", crop_model);

    pcl::console::parse_argument(argc, argv, "-tex-angle-thresh", tex_angle_tresh);
    pcl::console::parse_argument(argc, argv, "-tex-enable-blending", tex_enable_blending);
    pcl::console::parse_argument(argc, argv, "-tex-padding", tex_padding);
    pcl::console::parse_argument(argc, argv, "-tex-focal-length", focal_length);

    v4r::texturing::Renderer renderer;

    std::vector< std::string> training_dir_names;   // equal to class names
    std::string training_dir;
    training_dir.append(base_path);
    //training_dir.append("/");
    //training_dir.append(sub_dir);
    training_dir_names = v4r::io::getFoldersInDirectory(training_dir);

    if(training_dir_names.empty())
    {
        std::cerr << "No subfolders in directory " << base_path << ". " << std::endl;
        return -1;
    }

    std::vector< std::string> model_names;   // equal to class names
    std::string model_path;
    model_path.append(base_path);
    //model_path.append("/");
    //model_path.append(sub_dir_models);
    //model_names = v4r::io::getFilesInDirectory(model_path, ".*\\.(pcd|PCD)", true);
    model_names = training_dir_names;

    if( model_names.empty() )
    {
        std::cerr << "No files in directory " << model_path << ". " << std::endl;
        return -1;
    }

    assert ( model_names.size() == training_dir_names.size() );

    std::string textured_meshes;
    textured_meshes.append(base_path);

    for(size_t folder_id=0; folder_id < training_dir_names.size(); folder_id++)
    {
        std::string textured_meshes_model;
        textured_meshes_model.append(textured_meshes);
        textured_meshes_model.append("/");
        textured_meshes_model.append(training_dir_names[folder_id]);
        textured_meshes_model.append("/mesh");

        boost::filesystem::path textured_meshes_model_p (textured_meshes_model);
        if (boost::filesystem::is_directory(textured_meshes_model_p)) {
            std::cout << "Mesh exists already" << textured_meshes_model << std::endl;
          continue;
        }


        v4r::texturing::Reader reader;
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr model = reader.readModel(model_path+"/"+training_dir_names[folder_id], "3D_model.pcd");

        cout<<"crop_model="<<crop_model<<endl;

        v4r::texturing::Surface surface(poisson_depth, poisson_nr_samples, crop_model);
        pcl::PolygonMesh::Ptr mesh = surface.reconstruct(model);

        cout<<endl<<"Press ..."<<endl;
        cout<<"  [r] to activate rotation mode"<<endl;
        cout<<"  [e] to continue with object texturing"<<endl<<endl;

        renderer.renderMesh(mesh);


        if (!boost::filesystem::is_directory(textured_meshes_model_p))
        {
            if (boost::filesystem::create_directory(textured_meshes_model_p))
            {
                std::cout << "Created directory " << textured_meshes_model << std::endl;
            }
            else
            {
                std::cerr << "Cannnot create directory " <<textured_meshes_model << std::endl;
                return -1;
            }
        }

        pcl::io::savePolygonFileVTK(textured_meshes_model + "/mesh.vtk", *mesh);
        pcl::io::savePolygonFilePLY(textured_meshes_model + "/mesh.ply", *mesh);
        pcl::io::savePolygonFileSTL(textured_meshes_model + "/mesh.stl", *mesh);

    }

    return 0;
}
