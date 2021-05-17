
#include <ignition/math/Pose3.hh>
#include <functional>
#include "gazebo/common/Plugin.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/transport/Node.hh"
#include "gazebo/gazebo.hh"
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>

#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <shape_msgs/Mesh.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cloud_to_mesh.h>
#include <mesh_conversions.h>
#include <pcl/surface/organized_fast_mesh.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/TextureMesh.h>
#include <pcl/surface/texture_mapping.h>
#include <pcl/surface/mls.h>
#include <pcl/io/obj_io.h>
#include <pcl/filters/filter.h>


template<typename ... Args>
std::string format_string(const std::string& format, Args ... args)
{
  size_t size = snprintf(nullptr, 0, format.c_str(), args ...) + 1;
  std::unique_ptr<char[]> buffer(new char[size]);
  snprintf(buffer.get(), size, format.c_str(), args ...);
  return std::string(buffer.get(), buffer.get() + size - 1);
}

namespace gazebo
{
  class PointcloudGazebo : public WorldPlugin  
  {

    private:transport::NodePtr node;
            physics::WorldPtr my_world;


            pcl::OrganizedFastMesh<pcl::PointXYZ> ofm;
            std::string file_name_;



            // ros::Publisher marker_pub_;
            // ros::Publisher shape_pub_;
            // sensor_msgs::PointCloud2 cloud_out_;
            // sensor_msgs::PointCloud2 cloud_self_filtered_out;
            // int p_cloud_queue_size_;
            // CloudToMesh<pcl::PointXYZ, pcl::PointNormal> cloud_to_mesh_;
      

    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override
    {

      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_dst (new pcl::PointCloud<pcl::PointXYZ>);

      my_world = _world;
      
      // Create a new transport node
      this->node = transport::NodePtr(new transport::Node());

      // Initialize the node with the world name
      this->node->Init(my_world->Name());

      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client",
            ros::init_options::NoSigintHandler);
      }


      std::cout << "Ros init complete\n";
      
      //* load the file
        if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/jskimlab/gazebo_plugin_pc/PcdFile/test_cat.pcd", *cloud_dst) == -1) 
        {
          PCL_ERROR ("Couldn't read pcd file \n");
          return ;
        }

        

        cloud_dst->width = 5;
        cloud_dst->height = 10;
        cloud_dst->points.resize (cloud_dst->width * cloud_dst->height);

        std::cout << "Loaded "
                  << cloud_dst->width * cloud_dst->height
                  << " data points from pcd with the following fields: "
                  << std::endl;

        // int npoints = 0;
        // for (std::size_t i = 0; i < cloud_dst->height; i++)
        // {
        //   for (std::size_t j = 0; j < cloud_dst->width; j++)
        //   {
        //     (*cloud_dst)[npoints].x = static_cast<float> (i);
        //     (*cloud_dst)[npoints].y = static_cast<float> (j);
        //     (*cloud_dst)[npoints].z = static_cast<float> (cloud_dst->size ()); // to avoid shadowing
        //     npoints++;
        //   }
        // }
        // int nan_idx = cloud_dst->width*cloud_dst->height - 2*cloud_dst->width + 1;
        // (*cloud_dst)[nan_idx].x = std::numeric_limits<float>::quiet_NaN ();
        // (*cloud_dst)[nan_idx].y = std::numeric_limits<float>::quiet_NaN ();
        // (*cloud_dst)[nan_idx].z = std::numeric_limits<float>::quiet_NaN ();







        // pcl::PolygonMesh triangles;
        // ofm.setInputCloud(cloud_dst);
        // ofm.setMaxEdgeLength (1.5);
        // ofm.setTrianglePixelSize (1);
        // ofm.setTriangulationType (pcl::OrganizedFastMesh<pcl::PointXYZ>::TRIANGLE_ADAPTIVE_CUT);
        // ofm.reconstruct(triangles);

        // EXPECT_EQ (triangles.cloud.width, cloud_dst->width);
        // EXPECT_EQ (triangles.cloud.height, cloud_dst->height);
        // EXPECT_EQ (int (triangles.polygons.size ()), 2*(triangles.cloud.width-1)*(triangles.cloud.height-1) - 4);
        // EXPECT_EQ (int (triangles.polygons.at (0).vertices.size ()), 3);
        // EXPECT_EQ (int (triangles.polygons.at (0).vertices.at (0)), 0);
        // EXPECT_EQ (int (triangles.polygons.at (0).vertices.at (1)), triangles.cloud.width+1);
        // EXPECT_EQ (int (triangles.polygons.at (0).vertices.at (2)), 1);

        // std::stringstream ss;
        
        // ss << "pointcloud_mesh.stl";
        
        // ROS_INFO("Writing... %s", ss.str().c_str());
        // pcl::io::savePolygonFileSTL(ss.str(),triangles);

        // ROS_INFO("Finished saving stl file");





        sdf::SDF modelSDF;
        std::string text;

        // made collision of models 0, if it is not 0 and if two sphere poses so similar, they will move their pose to avoid each other.
        text = format_string(
          "<sdf version ='1.6'>\
              <model name='my_mesh'>\
                <pose>0 0 0 0 0 0</pose>\
                <link name='body'>\
                  <visual name='visual'>\
                    <pose>0 0 0 0 0 0</pose>\
                    <geometry>\
                      <mesh><uri>file:///home/jskimlab/gazebo_plugin_pc/MySrc/duck.dae</uri></mesh>\
                    </geometry>\
                  </visual>\
                  <gravity>0</gravity>\
                </link>\
              </model>\
          </sdf>");  // to make point density higher, modify more than 5
        modelSDF.SetFromString(text);

        // insert each point in gazebo
        _world->InsertModelSDF(modelSDF); 

        ROS_INFO("Inserting compelete");







        // ros::NodeHandle pnh("~");

        // marker_pub_ = pnh.advertise<visualization_msgs::Marker>("mesh_marker", 1, true);
        // shape_pub_  = pnh.advertise<shape_msgs::Mesh>("mesh_shape", 1, true);

        // cloud_to_mesh_.setVoxelFilterSize(0.025);

        // cloud_to_mesh_.setInput(cloud_dst);
        // if (cloud_to_mesh_.computeMesh())
        // {
        //     if (marker_pub_.getNumSubscribers() > 0){
        //         visualization_msgs::Marker mesh_marker;

        //         meshToMarkerMsg(cloud_to_mesh_.getMesh() ,mesh_marker);
        //         marker_pub_.publish(mesh_marker);
        //     }

        //     if (shape_pub_.getNumSubscribers() > 0){
        //         shape_msgs::Mesh shape_mesh;

        //         meshToShapeMsg(cloud_to_mesh_.getMesh() ,shape_mesh);
        //         shape_pub_.publish(shape_mesh);
        //     }
        // }
        // else{
        //     ROS_WARN("Could not generate mesh for point cloud!");
        // }

    }
        
  };
  GZ_REGISTER_WORLD_PLUGIN(PointcloudGazebo);
}  // namespace gazebo
