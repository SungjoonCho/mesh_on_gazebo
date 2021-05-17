
// #include <ignition/msgs.hh>
// #include <ignition/transport/Node.hh>
#include <ignition/math/Pose3.hh>
#include <functional>

#include "gazebo/common/Plugin.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/transport/Node.hh"
#include "gazebo/gazebo.hh"

// #include <pointcloud.pb.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>

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
  class realsense_mesh : public WorldPlugin  
  {

    private: transport::NodePtr node;
    private: transport::PublisherPtr factoryPub;
    // private: msgs::Factory msg;
    private: ros::Subscriber sub;
    // private: pcl::PointCloud<pcl::PointXYZ> cloud_dst;
    private: rs2::depth_frame dpt_frame;
    private: event::ConnectionPtr updateConnection;
    private: physics::WorldPtr my_world;
      
    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override
    {
      my_world = _world;
      
      // Create a new transport node
      this->node = transport::NodePtr(new transport::Node());

      // Initialize the node with the world name
      this->node->Init(my_world->Name());

      // Create a publisher on the ~/factory topic
      this->factoryPub = node->Advertise<msgs::Factory>("~/factory");

      // Create the message
      // msgs::Factory msg;

      // Model file to load
      // msg.set_sdf_filename("model://sphere");




      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client",
            ros::init_options::NoSigintHandler);
      }
      printf("Ros init complete\n");

      
      // ros::Rate r(1000);

      

      std::function<void(const rs2::depth_frame&)> cb =
          [=](const rs2::depth_frame& msg){
        realsense_mesh::OnContainPluginMsg(msg);
      };


      printf("In load\n");
      
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&realsense_mesh::OnUpdate, this));
      
      // ros::spin(); 
      
      // r.sleep();
      // ros::shutdown();        
    
    }

    public: void OnUpdate()
    {
      ros::NodeHandle node2;
      std::string topic("/camera/aligned_depth_to_color/image_raw");
      sub = node2.subscribe(topic, 1, &realsense_mesh::cb, this); 
      ros::spinOnce();

      // ros::Rate r(10);
      // r.sleep();
      // printf("in update\n");
    }

    public: void cb(const rs2::depth_frame &_msg)
    {
      // subscribe pointcloud msg
      printf("in callback()\n");

      // convert pointcloud2 to pointxyz to extract data
      // pcl::fromROSMsg(_msg, cloud_dst);

      // printf("%f\n",cloud_dst[0].x);

      // print type
      // printf("%lu\n",cloud_dst.points.size());

 
      // printf("published start\n");


      







      // for(int nIndex=0; nIndex < cloud_dst.points.size(); nIndex+=100000){
      //   msgs::Factory msg;
      //   msg.set_sdf_filename("model://sphere");

      //   x = int(cloud_dst.points[nIndex].x*10);
      //   y = int(cloud_dst.points[nIndex].y*10);
      //   z = int(cloud_dst.points[nIndex].z*10);

        
      //   // Pose to initialize the model to
      //   msgs::Set(msg.mutable_pose(), 
      //     ignition::math::Pose3d(
      //       ignition::math::Vector3d(x,y,z),
      //       ignition::math::Quaterniond(0, 0, 0)));

      //   // // Send the message
      //   this->factoryPub->Publish(msg);
      //   printf("%d %d %d %d published\n", nIndex, x,y,z);      
      // }

      // printf("published complete\n");
      // ros::shutdown();



        // for(int i =0; i<=1000; i+=10){
        //     msgs::Set(msg.mutable_pose(), 
        //     ignition::math::Pose3d(
        //       ignition::math::Vector3d(i,i,i),
        //       ignition::math::Quaterniond(0, 0, 0)));

        // // Send the message
        // this->factoryPub->Publish(msg);
        // }
        
      

    }
  };
  GZ_REGISTER_WORLD_PLUGIN(realsense_mesh);
}  // namespace gazebo
