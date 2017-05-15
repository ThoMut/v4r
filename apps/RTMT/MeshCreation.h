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

#ifndef _MESH_CREATION_H
#define _MESH_CREATION_H

#ifndef Q_MOC_RUN
#include <QThread>
#include <QMutex>
#include <queue>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/shared_ptr.hpp>
#include <pcl/common/transforms.h>
#include "params.h"
#include "sensor.h"
#include <v4r/keypoints/impl/Object.hpp>
#include <v4r/common/impl/SmartPtr.hpp>
#include <v4r/keypoints/impl/invPose.hpp>
#include <v4r/reconstruction/ProjBundleAdjuster.h>

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string>


#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <v4r/texturing/reader.h>
#include <v4r/texturing/renderer.h>
#include <v4r/texturing/surface.h>
#include <v4r/texturing/multiband.h>
//#include <v4r/texturing/export.h>
#include <v4r/io/filesystem.h>
#endif



class MeshCreation : public QThread
{
  Q_OBJECT

public:
  enum Command
  {
    CREATE_MESH,
    MAX_COMMAND,
    UNDEF = MAX_COMMAND
  };

  MeshCreation();
  ~MeshCreation();

  void start();
  void stop();
  bool isRunning();

  void calculateMesh(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &_model_cloud);
  bool saveMesh(const std::string &_folder, const std::string &_modelname);

public slots:
  void mesh_creation_parameter_changed(const MeshCreationParameter &_mesh_params);

signals:
  void printStatus(const std::string &_txt);
  void finishedCreateMesh();

private:
  void run();
  void calculateMesh();

  Command cmd;
  bool m_run;
  MeshCreationParameter mesh_params;

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr model_cloud;
  pcl::PolygonMesh::Ptr mesh;

};

#endif
