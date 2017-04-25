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

#include <v4r/texturing/AdvancedPclVisualizer.h>


#include <vtkOpenGLHardwareSupport.h>
#include <vtkOpenGLRenderWindow.h>
#include <vtkImageData.h>
#include <vtkImageImport.h>

namespace v4r
{
namespace texturing
{

bool AdvancedPclVisualizer::addTextureMesh (TexturedMesh::Ptr mesh, const std::string &id, int viewport)
{
    std::cout << "converting textured mesh" << std::endl;

    TextureMesh m;
    m.cloud = mesh->mesh->cloud;
    m.header = mesh->mesh->header;

    //m.tex_materials.reserve(mesh->textures.size());
    m.tex_coordinates.resize(1);
    m.tex_polygons.resize(1);

    TexMaterial mat;
    mat.data = mesh->textures[0];
    m.tex_materials.push_back(mat);

    std::cout << "created materials" << std::endl;

    for (unsigned int i=0;i<mesh->mesh->polygons.size();i++)
    {
        //int textureIndex = mesh->textureIndex[i];
        int textureIndex = 0;

        pcl::Vertices v = mesh->mesh->polygons[i];
        m.tex_polygons.at(textureIndex).push_back(v);

        /*
        for (int j=0;j<v.vertices.size();j++)
        {
            //std::cout << "get coordinates for " << v.vertices[j] << std::endl;
            Eigen::Vector2f coords = mesh->textureCoordinates[v.vertices[j]];

            //m.tex_coordinates.at(textureIndex).push_back(coords);
        }
        */
    }

    for (unsigned int i=0;i<mesh->textureCoordinates.size();i++)
    {
        Eigen::Vector2f coords = mesh->textureCoordinates[i];
        m.tex_coordinates.at(0).push_back(coords);
    }

    return addTextureMesh(m, id, viewport);
}


int AdvancedPclVisualizer::textureFromTexMaterial (const TexMaterial& tex_mat, vtkTexture* vtk_tex) const
{
    vtkSmartPointer<vtkImageImport> importer = vtkSmartPointer<vtkImageImport>::New();

    int width = tex_mat.data.cols;
    int height = tex_mat.data.rows;

    /*
    cv::Mat3b texture(width, height, CV_8UC3);

    for (int i=0;i<width;i++)
    {
        for (int j=0;j<height;j++)
        {
            texture.at<cv::Vec3b>(i, j).val[0] = i % 255; //image->at(i, j).b;
            texture.at<cv::Vec3b>(i, j).val[1] = 0; //image->at(i, j).g;
            texture.at<cv::Vec3b>(i, j).val[2] = 0; //image->at(i, j).r;
        }
    }
    */

    importer->SetDataSpacing( 1, 1, 1 );
    importer->SetDataOrigin( 0, 0, 0 );
    importer->SetWholeExtent(   0, width-1, 0, height-1, 0, 0 );
    importer->SetDataExtentToWholeExtent();
    importer->SetDataScalarTypeToUnsignedChar();
    importer->SetNumberOfScalarComponents( 3 );
    importer->SetImportVoidPointer( tex_mat.data.data );
    importer->Update();

    vtk_tex->SetInputConnection(importer->GetOutputPort());
    /*
    vtkSmartPointer<vtkJPEGReader> jpeg_reader = vtkSmartPointer<vtkJPEGReader>::New ();
    jpeg_reader->SetFileName (full_path.string ().c_str ());
    jpeg_reader->Update ();
    vtk_tex->SetInputConnection (jpeg_reader->GetOutputPort ());
    */

  return (0);
}

bool AdvancedPclVisualizer::addTextureMesh (const TextureMesh &mesh, const std::string &id, int viewport)
{
    std::cout << "rendering textured mesh" << std::endl;

  pcl::visualization::CloudActorMap::iterator am_it = getCloudActorMap()->find (id);
  if (am_it != getCloudActorMap()->end ())
  {
    PCL_ERROR ("[PCLVisualizer::addTextureMesh] A shape with id <%s> already exists!"
               " Please choose a different id and retry.\n",
               id.c_str ());
    return (false);
  }
  // no texture materials --> exit
  if (mesh.tex_materials.size () == 0)
  {
    PCL_ERROR("[PCLVisualizer::addTextureMesh] No textures found!\n");
    return (false);
  }
  // polygons are mapped to texture materials
  if (mesh.tex_materials.size () != mesh.tex_polygons.size ())
  {
    PCL_ERROR("[PCLVisualizer::addTextureMesh] Materials number %lu differs from polygons number %lu!\n",
              mesh.tex_materials.size (), mesh.tex_polygons.size ());
    return (false);
  }
  // each texture material should have its coordinates set
  if (mesh.tex_materials.size () != mesh.tex_coordinates.size ())
  {
    PCL_ERROR("[PCLVisualizer::addTextureMesh] Coordinates number %lu differs from materials number %lu!\n",
              mesh.tex_coordinates.size (), mesh.tex_materials.size ());
    return (false);
  }
  // total number of vertices
  std::size_t nb_vertices = 0;
  for (std::size_t i = 0; i < mesh.tex_polygons.size (); ++i)
    nb_vertices+= mesh.tex_polygons[i].size ();
  // no vertices --> exit
  if (nb_vertices == 0)
  {
    PCL_ERROR("[PCLVisualizer::addTextureMesh] No vertices found!\n");
    return (false);
  }
  // total number of coordinates
  std::size_t nb_coordinates = 0;
  for (std::size_t i = 0; i < mesh.tex_coordinates.size (); ++i)
    nb_coordinates+= mesh.tex_coordinates[i].size ();
  // no texture coordinates --> exit
  if (nb_coordinates == 0)
  {
    PCL_ERROR("[PCLVisualizer::addTextureMesh] No textures coordinates found!\n");
    return (false);
  }

  // Create points from mesh.cloud
  vtkSmartPointer<vtkPoints> poly_points = vtkSmartPointer<vtkPoints>::New ();
  vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New ();
  bool has_color = false;
  vtkSmartPointer<vtkMatrix4x4> transformation = vtkSmartPointer<vtkMatrix4x4>::New ();
  if ((pcl::getFieldIndex(mesh.cloud, "rgba") != -1) ||
      (pcl::getFieldIndex(mesh.cloud, "rgb") != -1))
  {
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromPCLPointCloud2(mesh.cloud, cloud);
    if (cloud.points.size () == 0)
    {
      PCL_ERROR("[PCLVisualizer::addTextureMesh] Cloud is empty!\n");
      return (false);
    }
    convertToVtkMatrix (cloud.sensor_origin_, cloud.sensor_orientation_, transformation);
    has_color = true;
    colors->SetNumberOfComponents (3);
    colors->SetName ("Colors");

    poly_points->SetNumberOfPoints (cloud.size ());
    for (std::size_t i = 0; i < cloud.points.size (); ++i)
    {
      const pcl::PointXYZRGB &p = cloud.points[i];
      poly_points->InsertPoint (i, p.x, p.y, p.z);
      const unsigned char color[3] = {p.r, p.g, p.b};
      colors->InsertNextTupleValue(color);
    }
  }
  else
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::fromPCLPointCloud2 (mesh.cloud, *cloud);
    // no points --> exit
    if (cloud->points.size () == 0)
    {
      PCL_ERROR("[PCLVisualizer::addTextureMesh] Cloud is empty!\n");
      return (false);
    }
    convertToVtkMatrix (cloud->sensor_origin_, cloud->sensor_orientation_, transformation);
    poly_points->SetNumberOfPoints (cloud->points.size ());
    for (std::size_t i = 0; i < cloud->points.size (); ++i)
    {
      const pcl::PointXYZ &p = cloud->points[i];
      poly_points->InsertPoint (i, p.x, p.y, p.z);
    }
  }

  //create polys from polyMesh.tex_polygons
  vtkSmartPointer<vtkCellArray> polys = vtkSmartPointer<vtkCellArray>::New ();
  for (std::size_t i = 0; i < mesh.tex_polygons.size (); i++)
  {
    for (std::size_t j = 0; j < mesh.tex_polygons[i].size (); j++)
    {
      std::size_t n_points = mesh.tex_polygons[i][j].vertices.size ();
      polys->InsertNextCell (int (n_points));
      for (std::size_t k = 0; k < n_points; k++)
        polys->InsertCellPoint (mesh.tex_polygons[i][j].vertices[k]);
    }
  }
  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
  polydata->SetPolys (polys);
  polydata->SetPoints (poly_points);
  if (has_color)
    polydata->GetPointData()->SetScalars(colors);

  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New ();
#if VTK_MAJOR_VERSION < 6
    mapper->SetInput (polydata);
#else
    mapper->SetInputData (polydata);
#endif

  vtkSmartPointer<vtkLODActor> actor = vtkSmartPointer<vtkLODActor>::New ();
  vtkOpenGLHardwareSupport* hardware = vtkOpenGLRenderWindow::SafeDownCast (getRenderWindow())->GetHardwareSupport ();
  bool supported = hardware->GetSupportsMultiTexturing ();
  // Check if hardware support multi texture
  std::size_t texture_units (hardware->GetNumberOfFixedTextureUnits ());
  if ((mesh.tex_materials.size () > 1) && supported && (texture_units > 1))
  {
    if (texture_units < mesh.tex_materials.size ())
      PCL_WARN ("[PCLVisualizer::addTextureMesh] GPU texture units %d < mesh textures %d!\n",
                texture_units, mesh.tex_materials.size ());
    // Load textures
    std::size_t last_tex_id = std::min (mesh.tex_materials.size (), texture_units);
    int tu = vtkProperty::VTK_TEXTURE_UNIT_0;
    std::size_t tex_id = 0;
    while (tex_id < last_tex_id)
    {
      vtkSmartPointer<vtkTexture> texture = vtkSmartPointer<vtkTexture>::New ();
      if (textureFromTexMaterial (mesh.tex_materials[tex_id], texture))
      {
        PCL_WARN ("[PCLVisualizer::addTextureMesh] Failed to load texture %s, skipping!\n",
                  mesh.tex_materials[tex_id].tex_name.c_str ());
        continue;
      }
      // the first texture is in REPLACE mode others are in ADD mode
      if (tex_id == 0)
        texture->SetBlendingMode(vtkTexture::VTK_TEXTURE_BLENDING_MODE_REPLACE);
      else
        texture->SetBlendingMode(vtkTexture::VTK_TEXTURE_BLENDING_MODE_ADD);
      // add a texture coordinates array per texture
      vtkSmartPointer<vtkFloatArray> coordinates = vtkSmartPointer<vtkFloatArray>::New ();
      coordinates->SetNumberOfComponents (2);
      std::stringstream ss; ss << "TCoords" << tex_id;
      std::string this_coordinates_name = ss.str ();
      coordinates->SetName (this_coordinates_name.c_str ());

      for (std::size_t t = 0 ; t < mesh.tex_coordinates.size (); ++t)
        if (t == tex_id)
          for (std::size_t tc = 0; tc < mesh.tex_coordinates[t].size (); ++tc)
            coordinates->InsertNextTuple2 (mesh.tex_coordinates[t][tc][0],
                                           mesh.tex_coordinates[t][tc][1]);
        else
          for (std::size_t tc = 0; tc < mesh.tex_coordinates[t].size (); ++tc)
            coordinates->InsertNextTuple2 (-1.0, -1.0);

      mapper->MapDataArrayToMultiTextureAttribute(tu,
                                                  this_coordinates_name.c_str (),
                                                  vtkDataObject::FIELD_ASSOCIATION_POINTS);
      polydata->GetPointData ()->AddArray (coordinates);
      actor->GetProperty ()->SetTexture(tu, texture);
      ++tex_id;
      ++tu;
    }
  } // end of multi texturing
  else
  {
    if (!supported || texture_units < 2)
      PCL_WARN ("[PCLVisualizer::addTextureMesh] Your GPU doesn't support multi texturing. "
                "Will use first one only!\n");

    vtkSmartPointer<vtkTexture> texture = vtkSmartPointer<vtkTexture>::New ();
    // fill vtkTexture from pcl::TexMaterial structure
    if (textureFromTexMaterial (mesh.tex_materials[0], texture))
      PCL_WARN ("[PCLVisualizer::addTextureMesh] Failed to create vtkTexture from %s!\n",
                mesh.tex_materials[0].tex_name.c_str ());

    // set texture coordinates
    vtkSmartPointer<vtkFloatArray> coordinates = vtkSmartPointer<vtkFloatArray>::New ();
    coordinates->SetNumberOfComponents (2);
    coordinates->SetNumberOfTuples (mesh.tex_coordinates[0].size ());
    for (std::size_t tc = 0; tc < mesh.tex_coordinates[0].size (); ++tc)
    {
      const Eigen::Vector2f &uv = mesh.tex_coordinates[0][tc];
      coordinates->SetTuple2 (tc, uv[0], uv[1]);
    }
    coordinates->SetName ("TCoords");
    polydata->GetPointData ()->SetTCoords(coordinates);
    // apply texture
    actor->SetTexture (texture);
  } // end of one texture

  // set mapper
  actor->SetMapper (mapper);
  addActorToRenderer2 (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  (*getCloudActorMap())[id].actor = actor;

  // Save the viewpoint transformation matrix to the global actor map
  (*getCloudActorMap())[id].viewpoint_transformation_ = transformation;

  std::cout << "finished rendering" << std::endl;

  return (true);
}

void AdvancedPclVisualizer::addActorToRenderer2 (const vtkSmartPointer<vtkProp> &actor, int viewport)
{
  // Add it to all renderers
  getRendererCollection()->InitTraversal ();
  vtkRenderer* renderer = NULL;
  int i = 0;
  while ((renderer = getRendererCollection()->GetNextItem ()) != NULL)
  {
    // Should we add the actor to all renderers?
    if (viewport == 0)
    {
      renderer->AddActor (actor);
    }
    else if (viewport == i)               // add the actor only to the specified viewport
    {
      renderer->AddActor (actor);
    }
    ++i;
  }
}

}
}
