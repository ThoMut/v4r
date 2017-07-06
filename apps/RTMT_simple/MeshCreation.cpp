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

#ifndef Q_MOC_RUN
#include "MeshCreation.h"
//#include <v4r/keypoints/impl/toString.hpp>
#endif

using namespace std;

/**
 * @brief MeshCreation::MeshCreation
 */
MeshCreation::MeshCreation() :
  cmd(UNDEF), m_run(false)
{
}

/**
 * @brief MeshCreation::~MeshCreation
 */
MeshCreation::~MeshCreation()
{
  stop();
}

/******************************** public *******************************/

/**
 * @brief MeshCreation::start
 * @param cam_id
 */
void MeshCreation::start()
{
  QThread::start();
}

/**
 * @brief MeshCreation::stop
 */
void MeshCreation::stop()
{
  if(m_run)
  {
    m_run = false;
    this->wait();
  }
}

/**
 * @brief MeshCreation::isRunning
 * @return
 */
bool MeshCreation::isRunning()
{
  return m_run;
}

/**
 * @brief MeshCreation::calculateMesh
 * @param _cloud
 */
void MeshCreation::calculateMesh(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &_model_cloud)
{

  model_cloud = _model_cloud;

  cmd = CREATE_MESH;
  start();
}


/********************************** private ****************************************/

/**
 * @brief MeshCreation::run
 * main loop
 */
void MeshCreation::run()
{
  m_run=true;

  switch (cmd)
  {
  case CREATE_MESH:
  {
    emit printStatus("Status: Creating Mesh ... Please be patient...");

    calculateMesh();

    break;
  }
  default:
    break;
  }

  cmd = UNDEF;
  m_run=false;

  //while(m_run)
  //{
  //}
}

/**
 * @brief MeshCreation::calculateMesh
 */
void MeshCreation::calculateMesh()
{
//  if (model_cloud == 0)
//    return;

    // parameters

    // General
//    std::string base_path("/home/thomas/DA/shared_docker_host/data/texture_test");

    // poisson
    bool crop_model = true;

    //pcl::io::loadPCDFile ("/home/thomas/DA/shared_docker_host/data/texture_test/cleaning_agent/3D_model.pcd", *model_cloud);

    v4r::texturing::Renderer renderer;

    cout<<"crop_model="<<crop_model<<endl;

    v4r::texturing::Surface surface(mesh_params.poisson_depth, mesh_params.poisson_nr_samples, crop_model);
    mesh.reset(new pcl::PolygonMesh);
    mesh = surface.reconstruct(model_cloud);

    //pcl::io::saveVTKFile("/home/thomas/Desktop/test/mesh.vtk", *mesh);
//    pcl::io::savePolygonFileVTK("/home/thomas/Desktop/test/mesh.vtk", *mesh);
//    pcl::PolygonMesh mesh_test;
//    pcl::io::loadPolygonFileVTK("/home/thomas/Desktop/test/mesh.vtk", *mesh);

    cout<<endl<<"Press ..."<<endl;
    cout<<"  [r] to activate rotation mode"<<endl;
    cout<<"  [e] to continue with object texturing"<<endl<<endl;

//    boost::filesystem::path textured_meshes_model_p (textured_meshes_model);
//    if (!boost::filesystem::is_directory(textured_meshes_model_p))
//    {
//        if (boost::filesystem::create_directory(textured_meshes_model_p))
//        {
//            std::cout << "Created directory " << textured_meshes_model << std::endl;
//        }
//        else
//        {
//            std::cerr << "Cannnot create directory " <<textured_meshes_model << std::endl;
//            return;
//        }
//    }

    //emit update_visualization();

    std::string txt = std::string("Status: Mesh created");
    emit finishedCreateMesh();
    emit printStatus(txt);
    renderer.renderMesh(mesh);
}

void MeshCreation::mesh_creation_parameter_changed(const MeshCreationParameter &_mesh_params)
{
    mesh_params = _mesh_params;
}

bool MeshCreation::saveMesh(const std::string &_folder, const std::string &_modelname)
{
    boost::filesystem::create_directories(_folder + "/models/" + _modelname + "/mesh" );

    pcl::io::savePolygonFileVTK(_folder + "/models/" + _modelname + "/mesh" + "/mesh.vtk", *mesh);
    pcl::io::savePolygonFilePLY(_folder + "/models/" + _modelname + "/mesh" + "/mesh.ply", *mesh);
    pcl::io::savePolygonFileSTL(_folder + "/models/" + _modelname + "/mesh" + "/mesh.stl", *mesh);
    //PROBLEM: Numbers in File are stored with comma instead of point!
    return true;
}

