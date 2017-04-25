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

#include <v4r/texturing/renderer.h>

namespace v4r
{
namespace texturing
{


void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
    if (event.keyDown())
    {
        Renderer *renderer = static_cast<Renderer*> (viewer_void);
        std::cout << event.getKeySym() << std::endl;

        if (event.getKeySym () == "space") renderer->cont();
    }
}

Renderer::Renderer()
{
  //TEST: wieder einblenden wenn man renderer will
    pcl::visualization::PCLVisualizerInteractorStyle *style = new pcl::visualization::PCLVisualizerInteractorStyle();
    int argc = 0;
    vis.reset(new AdvancedPclVisualizer(argc, NULL, "", style, true));

    vis->registerKeyboardCallback(keyboardEventOccurred, (void*) this);
}

void Renderer::cont()
{
    vis->spinOnce();
}

void Renderer::renderTextureStep(pcl::PolygonMesh::Ptr mesh,  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Eigen::Vector3f cam_vector)
{
    /*
    vis->removeAllPointClouds();
    vis->removeAllShapes();

    std::cout << "Cam vector: " << cam_vector << std::endl;


    pcl::PointCloud<pcl::PointXYZ> meshCloud;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr colored(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::fromPCLPointCloud2(mesh->cloud, meshCloud);
    pcl::copyPointCloud(meshCloud, *colored);


    for (int i=0;i<mesh->polygons.size();i++)
    {
        pcl::PointXYZRGBNormal p1 = colored->points[mesh->polygons[i].vertices[0]];
        pcl::PointXYZRGBNormal p2 = colored->points[mesh->polygons[i].vertices[1]];
        pcl::PointXYZRGBNormal p3 = colored->points[mesh->polygons[i].vertices[2]];

        Eigen::Vector3f v1 = Eigen::Vector3f( p1.getArray3fMap() );
        Eigen::Vector3f v2 = Eigen::Vector3f( p2.getArray3fMap() );
        Eigen::Vector3f v3 = Eigen::Vector3f( p3.getArray3fMap() );

        v2 = v1 - v2;
        v3 = v1 - v3;
        v2.normalize();
        v3.normalize();

        Eigen::Vector3f normal = v2.cross(v3);

        double angle = acos(normal.dot(cam_vector)) * 180.0f / M_PI;

        //if (angle < 0.0f)
        //    angle = 0.0f;

        angle = angle / 90.0;

        if (angle > 1.0f)
        {
            angle = 1.0f;
        }

        for (int j=0;j<mesh->polygons[i].vertices.size();j++)
        {
            colored->points[mesh->polygons[i].vertices[j]].r = 255 * angle;
            colored->points[mesh->polygons[i].vertices[j]].g = 255 - 255 * angle;
            colored->points[mesh->polygons[i].vertices[j]].b = 0;
        }
    }

    pcl::toPCLPointCloud2(*colored, mesh->cloud);


    vis->addCoordinateSystem(1.0);
    vis->addArrow(pcl::PointXYZ(0.0f, 0.0f, 0.0f), pcl::PointXYZ(cam_vector.x(), cam_vector.y(), cam_vector.z()), 1.0, 1.0, 1.0, true, "cam", 0);
    vis->addPointCloud(cloud, "cloud");
    vis->addPolygonMesh(*mesh);

    vis->spin();
    */
}

void Renderer::renderPointClouds(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds)
{
    vis->removeAllPointClouds();

    for (int i=0;i<(int)clouds.size();i++)
    {
        std::stringstream name;
        name << "cloud " << i;

        vis->addPointCloud(clouds[i], name.str());
    }

    vis->spin();
}

void Renderer::renderMesh(pcl::PolygonMesh::Ptr mesh)
{
    vis->removeAllPointClouds();

    vis->addPolygonMesh(*mesh, "mesh");

    vis->spin();
}

void Renderer::renderTexturedMesh(TexturedMesh::Ptr mesh)
{
    vis->removeAllPointClouds();

    vis->addTextureMesh(mesh, std::string("mesh"), 0);

    vis->spin();
}

void Renderer::renderPointCloudWithNormals(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr model)
{
    vis->removeAllPointClouds();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr points(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    pcl::copyPointCloud(*model, *points);
    pcl::copyPointCloud(*model, *normals);

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> rgb(model);
    vis->addPointCloud<pcl::PointXYZRGBNormal>(model, rgb, "cloud");

    //vis.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(points, normals, 10, 0.05, "cloud2");

    vis->spin();
}

void Renderer::renderPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr model)
{
    vis->removeAllPointClouds();

    vis->addPointCloud(model, "cloud");

    vis->spin();
}

}
}
