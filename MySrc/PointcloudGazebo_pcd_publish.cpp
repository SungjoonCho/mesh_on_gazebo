
//#include <ignition/math/Pose3.hh>
//#include <functional>
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

    private: transport::NodePtr node;
    private: transport::PublisherPtr factoryPub;
      
    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override
    {

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_dst (new pcl::PointCloud<pcl::PointXYZ>);
        
        // Create a new transport node
        this->node = transport::NodePtr(new transport::Node());

        // Initialize the node with the world name
        this->node->Init(_world->Name());

        // Create a publisher on the ~/factory topic
        this->factoryPub = node->Advertise<msgs::Factory>("~/factory");

        if (!ros::isInitialized())
        {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "PointcloudGazebo_pcd_publish",
                ros::init_options::NoSigintHandler);
        }

        std::cout << "Ros init complete\n";
        
        //* load the file
        if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/jskimlab/gazebo_plugin_pc/PcdFile/test_cat.pcd", *cloud_dst) == -1) 
        {
          PCL_ERROR ("Couldn't read pcd file \n");
          return ;
        }


        std::cout << "Loaded "
                  << cloud_dst->width * cloud_dst->height
                  << " data points from test_pcd.pcd with the following fields: "
                  << std::endl;


        int cnt = 0; // counting point 

        for (const auto& point: *cloud_dst){
            // if(cnt%10 != 0){
            //   cnt++;              
            //   continue;
            // }
            msgs::Factory msg;
            msg.set_sdf_filename("model://sphere");
            
            // Pose to initialize the model to
            msgs::Set(msg.mutable_pose(), 
            ignition::math::Pose3d(
                ignition::math::Vector3d(point.x/5,point.y/5,point.z/5),
                ignition::math::Quaterniond(0, 0, 0)));

            // // Send the message
            this->factoryPub->Publish(msg);
            cnt++;            
        }

        std::cout << "Publish complete\n";
      
    }
        
  };
  GZ_REGISTER_WORLD_PLUGIN(PointcloudGazebo);
}  // namespace gazebo