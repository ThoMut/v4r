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

namespace v4r
{
namespace texturing
{

int saveObjFileImpl (const std::string &file_name,
                      const pcl::TextureMesh &tex_mesh, unsigned precision)
{
  if (tex_mesh.cloud.data.empty ())
  {
    PCL_ERROR ("[pcl::io::saveOBJFile] Input point cloud has no data!\n");
    return (-1);
  }
  // Open file
  std::ofstream fs;
  fs.precision (precision);
  fs.open (file_name.c_str ());

  // Define material file
  std::string mtl_file_name = file_name.substr (0, file_name.find_last_of (".")) + ".mtl";
  // Strip path for "mtllib" command
  std::string mtl_file_name_nopath = mtl_file_name;
  mtl_file_name_nopath.erase (0, mtl_file_name.find_last_of ('/') + 1);

  /* Write 3D information */
  // number of points
  unsigned nr_points  = tex_mesh.cloud.width * tex_mesh.cloud.height;
  unsigned point_size = static_cast<unsigned> (tex_mesh.cloud.data.size () / nr_points);

  // mesh size
  unsigned nr_meshes = static_cast<unsigned> (tex_mesh.tex_polygons.size ());
  // number of faces for header
  unsigned nr_faces = 0;
  for (unsigned m = 0; m < nr_meshes; ++m)
    nr_faces += static_cast<unsigned> (tex_mesh.tex_polygons[m].size ());

  // Write the header information
  fs << "####" << std::endl;
  fs << "# OBJ dataFile simple version. File name: " << file_name << std::endl;
  fs << "# Vertices: " << nr_points << std::endl;
  fs << "# Faces: " <<nr_faces << std::endl;
  fs << "# Material information:" << std::endl;
  fs << "mtllib " << mtl_file_name_nopath << std::endl;
  fs << "####" << std::endl;

  // Write vertex coordinates
  fs << "# Vertices" << std::endl;
  for (unsigned i = 0; i < nr_points; ++i)
  {
    int xyz = 0;
    // "v" just be written one
    bool v_written = false;
    for (size_t d = 0; d < tex_mesh.cloud.fields.size (); ++d)
    {
      int count = tex_mesh.cloud.fields[d].count;
      if (count == 0)
        count = 1;          // we simply cannot tolerate 0 counts (coming from older converter code)
      int c = 0;
      // adding vertex
      if ((tex_mesh.cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) && (
          tex_mesh.cloud.fields[d].name == "x" ||
          tex_mesh.cloud.fields[d].name == "y" ||
          tex_mesh.cloud.fields[d].name == "z"))
      {
        if (!v_written)
        {
           // write vertices beginning with v
          fs << "v ";
          v_written = true;
        }
        float value;
        memcpy (&value, &tex_mesh.cloud.data[i * point_size + tex_mesh.cloud.fields[d].offset + c * sizeof (float)], sizeof (float));
        fs << value;
        if (++xyz == 3)
          break;
        fs << " ";
      }
    }
    if (xyz != 3)
    {
      PCL_ERROR ("[pcl::io::saveOBJFile] Input point cloud has no XYZ data!\n");
      return (-2);
    }
    fs << std::endl;
  }
  fs << "# "<< nr_points <<" vertices" << std::endl;

  // Write vertex normals
  for (unsigned i = 0; i < nr_points; ++i)
  {
    int xyz = 0;
    // "vn" just be written one
    bool v_written = false;
    for (size_t d = 0; d < tex_mesh.cloud.fields.size (); ++d)
    {
      int count = tex_mesh.cloud.fields[d].count;
      if (count == 0)
        count = 1;          // we simply cannot tolerate 0 counts (coming from older converter code)
      int c = 0;
      // adding vertex
      if ((tex_mesh.cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) && (
          tex_mesh.cloud.fields[d].name == "normal_x" ||
          tex_mesh.cloud.fields[d].name == "normal_y" ||
          tex_mesh.cloud.fields[d].name == "normal_z"))
      {
          if (!v_written)
          {
          // write vertices beginning with vn
          fs << "vn ";
          v_written = true;
          }
        float value;
        memcpy (&value, &tex_mesh.cloud.data[i * point_size + tex_mesh.cloud.fields[d].offset + c * sizeof (float)], sizeof (float));
        fs << value;
        if (++xyz == 3)
          break;
        fs << " ";
      }
    }
    if (xyz != 3)
    {
      PCL_ERROR ("[pcl::io::saveOBJFile] Input point cloud has no normals!\n");
      return (-2);
    }
    fs << std::endl;
  }
  // Write vertex texture with "vt" (adding latter)

  for (unsigned m = 0; m < nr_meshes; ++m)
  {
    fs << "# " << tex_mesh.tex_coordinates[m].size() << " vertex textures in submesh " << m <<  std::endl;
    for (size_t i = 0; i < tex_mesh.tex_coordinates[m].size (); ++i)
    {
      fs << "vt ";

      if (tex_mesh.tex_coordinates[m][i][0] < 0.0f || tex_mesh.tex_coordinates[m][i][0] > 1.0f ||
              tex_mesh.tex_coordinates[m][i][1] < 0.0f || tex_mesh.tex_coordinates[m][i][1] > 1.0f)
      {
          fs <<  0.0f << " " << 0.0f << std::endl;
      } else {
        fs <<  tex_mesh.tex_coordinates[m][i][0] << " " << (1.0f - tex_mesh.tex_coordinates[m][i][1]) << std::endl;
      }
      //std::cout << "tex: " << tex_mesh.tex_coordinates[m][i][0] << " " << tex_mesh.tex_coordinates[m][i][1] << std::endl;
    }
  }

  unsigned f_idx = 0;

  // int idx_vt =0;
  for (unsigned m = 0; m < nr_meshes; ++m)
  {
    if (m > 0) f_idx += static_cast<unsigned> (tex_mesh.tex_polygons[m-1].size ());

    fs << "# The material will be used for mesh " << m << std::endl;
    fs << "usemtl " <<  tex_mesh.tex_materials[m].tex_name << std::endl;
    fs << "# Faces" << std::endl;

    for (size_t i = 0; i < tex_mesh.tex_polygons[m].size(); ++i)
    {
      // Write faces with "f"
      fs << "f";
      size_t j = 0;
      // There's one UV per vertex per face, i.e., the same vertex can have
      // different UV depending on the face.
      for (j = 0; j < tex_mesh.tex_polygons[m][i].vertices.size (); ++j)
      {
        uint32_t idx = tex_mesh.tex_polygons[m][i].vertices[j] + 1;
        fs << " " << idx
           << "/" << idx //tex_mesh.tex_polygons[m][i].vertices.size () * (i+f_idx) +j+1
           << "/" << idx; // vertex index in obj file format starting with 1
      }
      fs << std::endl;
    }
    fs << "# "<< tex_mesh.tex_polygons[m].size() << " faces in mesh " << m << std::endl;
  }
  fs << "# End of File";



  // Close obj file
  fs.close ();

  /* Write material defination for OBJ file*/
  // Open file

  std::ofstream m_fs;
  m_fs.precision (precision);
  m_fs.open (mtl_file_name.c_str ());

  // default
  m_fs << "#" << std::endl;
  m_fs << "# Wavefront material file" << std::endl;
  m_fs << "#" << std::endl;
  for(unsigned m = 0; m < nr_meshes; ++m)
  {
    m_fs << "newmtl " << tex_mesh.tex_materials[m].tex_name << std::endl;
    m_fs << "Ka "<< tex_mesh.tex_materials[m].tex_Ka.r << " " << tex_mesh.tex_materials[m].tex_Ka.g << " " << tex_mesh.tex_materials[m].tex_Ka.b << std::endl; // defines the ambient color of the material to be (r,g,b).
    m_fs << "Kd "<< tex_mesh.tex_materials[m].tex_Kd.r << " " << tex_mesh.tex_materials[m].tex_Kd.g << " " << tex_mesh.tex_materials[m].tex_Kd.b << std::endl; // defines the diffuse color of the material to be (r,g,b).
    m_fs << "Ks "<< tex_mesh.tex_materials[m].tex_Ks.r << " " << tex_mesh.tex_materials[m].tex_Ks.g << " " << tex_mesh.tex_materials[m].tex_Ks.b << std::endl; // defines the specular color of the material to be (r,g,b). This color shows up in highlights.
    m_fs << "d " << tex_mesh.tex_materials[m].tex_d << std::endl; // defines the transparency of the material to be alpha.
    m_fs << "Ns "<< tex_mesh.tex_materials[m].tex_Ns  << std::endl; // defines the shininess of the material to be s.
    m_fs << "illum "<< tex_mesh.tex_materials[m].tex_illum << std::endl; // denotes the illumination model used by the material.
                                            // illum = 1 indicates a flat material with no specular highlights, so the value of Ks is not used.
                                            // illum = 2 denotes the presence of specular highlights, and so a specification for Ks is required.
    m_fs << "map_Kd " << tex_mesh.tex_materials[m].tex_file << std::endl;
    m_fs << "###" << std::endl;
  }

  m_fs.close ();


  return (0);
}

void saveObjFile(std::string filename, TexturedMesh::Ptr mesh, std::string texture_file)
{
    std::cout << "converting textured mesh" << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr meshCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr meshCloudWithNormals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::fromPCLPointCloud2 (mesh->mesh->cloud, *meshCloud);

    pcl::copyPointCloud(*meshCloud, *meshCloudWithNormals);
    pcl::toPCLPointCloud2(*meshCloudWithNormals, mesh->mesh->cloud);

    pcl::TextureMesh m;
    m.cloud = mesh->mesh->cloud;
    m.header = mesh->mesh->header;

    //m.tex_materials.reserve(mesh->textures.size());
    m.tex_coordinates.resize(1);
    m.tex_polygons.resize(1);

    pcl::TexMaterial mat;
    //mat.data = mesh->textures[0];
    mat.tex_file = texture_file;
    mat.tex_name = "texture";
    m.tex_materials.push_back(mat);

    std::cout << "created materials" << std::endl;

    std::cout << "mesh polygons size: " << mesh->mesh->polygons.size() << std::endl;
    std::cout << "cloud size: " << meshCloudWithNormals->size() << std::endl;

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

    std::cout << "mesh tex coordinates size: " << mesh->textureCoordinates.size() << std::endl;

    for (unsigned int i=0;i<mesh->textureCoordinates.size();i++)
    {
        Eigen::Vector2f coords = mesh->textureCoordinates[i];
        m.tex_coordinates.at(0).push_back(coords);
    }

    saveObjFileImpl(filename, m, 10);

}

}
}
