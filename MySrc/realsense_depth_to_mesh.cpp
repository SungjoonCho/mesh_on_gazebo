
// #include <ignition/msgs.hh>
// #include <ignition/transport/Node.hh>
#include <ignition/math/Pose3.hh>
#include <functional>

#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Mesh.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/transport/Node.hh"
#include "gazebo/gazebo.hh"

#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <librealsense2/rs.hpp>
#include <sstream>
#include <image_transport/image_transport.h>
#include <unistd.h>

#include <gazebo/msgs/msgs.hh>


using namespace cv;

// template<typename ... Args>
// std::string format_string(const std::string& format, Args ... args)
// {
//   size_t size = snprintf(nullptr, 0, format.c_str(), args ...) + 1;
//   std::unique_ptr<char[]> buffer(new char[size]);
//   snprintf(buffer.get(), size, format.c_str(), args ...);
//   return std::string(buffer.get(), buffer.get() + size - 1);
// }

namespace gazebo
{
  class RealsenseDepthToMesh : public WorldPlugin  
  {

    private: transport::NodePtr node;
            //  transport::PublisherPtr factoryPub;     
                          
    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
        
        // // Create a new transport node
        this->node = transport::NodePtr(new transport::Node());

        // // Initialize the node with the world name
        this->node->Init(_world->Name());

        

        std::string text(
          "<?xml version='1.0' encoding='utf-8' ?>\
          <COLLADA xmlns='http://www.collada.org/2005/11/COLLADASchema' version='1.4.1'>\
              <asset>\
                  <contributor>\
                      <authoring_tool>modo 801 [Build 77509], Microsoft Windows 7 Service Pack 1 (6.1.7601 Service Pack 1)</authoring_tool>\
                      <comments>Plug-in: [Build 77509]; Use Absolute Path: No; Merge Reference Items: No; Save Hidden Items: No; Save Cameras: No; Save Lights: No; Save Locators: Yes; Save Triangles as Triangles: Yes; Order Vertex Maps Alphabetically: Yes; Bake Matrices: No; Save Vertex Normals: Yes; Save UV Texture Coordinates: Yes; Save Vertex Colors: No; Save Vertex Weights: No; Save Animation: Yes; Sample Animation: No; Sample Animation Start: 0; Sample Animation End: 120; Save modo Profile: No; Save Maya Profile: No; Save 3ds Max Profile: No; Formatted Arrays: No;</comments>\
                      <source_data>file:///C:/Users/Branden/Creative%20Cloud%20Files/03-2015%20Cesium%20Test%20Models/CesiumBoxTest.lxo</source_data>\
                  </contributor>\
                  <created>2015-03-04T16:39:37Z</created>\
                  <modified>2015-03-04T16:39:37Z</modified>\
                  <up_axis>Z_UP</up_axis>\
              </asset>\
              <library_materials>\
                  <material id='Material-Red' name='Red'>\
                      <instance_effect url='#Effect-Red' />\
                  </material>\
              </library_materials>\
              <library_effects>\
                  <effect id='Effect-Red' name='Red'>\
                      <profile_COMMON>\
                          <technique sid='common'>\
                              <phong>\
                                  <diffuse>\
                                      <color sid='diffuse_effect_rgb'>0.8 0 0 1</color>\
                                  </diffuse>\
                                  <specular>\
                                      <color sid='specular_effect_rgb'>0.2 0.2 0.2 1</color>\
                                  </specular>\
                                  <shininess>\
                                      <float sid='specular_effect_rgb'>256</float>\
                                  </shininess>\
                              </phong>\
                          </technique>\
                      </profile_COMMON>\
                  </effect>\
              </library_effects>\
              <library_geometries>\
                  <geometry id='Geometry-mesh002' name='Mesh'>\
                      <mesh>\
                          <source id='Geometry-mesh002-positions' name='positions'>\
                              <float_array id='Geometry-mesh002-positions-array' count='24'>\
                              -0.5 -0.5 -0.5 -0.5 0.5 -0.5 0.5 0.5 -0.5 0.5 -0.5 -0.5 -0.5 -0.5 0.5 -0.5 0.5 0.5 0.5 0.5 0.5 0.5 -0.5 0.5\
                              </float_array>\
                              <technique_common>\
                                  <accessor count='8' source='#Geometry-mesh002-positions-array' stride='3'>\
                                      <param name='X' type='float' />\
                                      <param name='Y' type='float' />\
                                      <param name='Z' type='float' />\
                                  </accessor>\
                              </technique_common>\
                          </source>\
                          <source id='Geometry-mesh002-normals' name='normals'>\
                              <float_array id='Geometry-mesh002-normals-array' count='18'>0 0 1 0 -1 0 1 0 0 0 1 0 -1 0 0 0 0 -1</float_array>\
                              <technique_common>\
                                  <accessor count='6' source='#Geometry-mesh002-normals-array' stride='3'>\
                                      <param name='X' type='float' />\
                                      <param name='Y' type='float' />\
                                      <param name='Z' type='float' />\
                                  </accessor>\
                              </technique_common>\
                          </source>\
                          <source id='Geometry-mesh002-Texture' name='Texture'>\
                              <float_array id='Geometry-mesh002-Texture-array' count='28'>0.25 1 0.25 0.666667 0.5 1 0.5 0.666667 0 0.666667 0.25 0.333333 0 0.333333 0.5 0.333333 0.75 0.666667 0.75 0.333333 1 0.666667 1 0.333333 0.25 0 0.5 0</float_array>\
                              <technique_common>\
                                  <accessor count='14' source='#Geometry-mesh002-Texture-array' stride='2'>\
                                      <param name='S' type='float' />\
                                      <param name='T' type='float' />\
                                  </accessor>\
                              </technique_common>\
                          </source>\
                          <vertices id='Geometry-mesh002-vertices'>\
                              <input semantic='POSITION' source='#Geometry-mesh002-positions' />\
                          </vertices>\
                          <triangles count='12' material='Material-Red'>\
                              <input semantic='VERTEX' source='#Geometry-mesh002-vertices' offset='0' />\
                              <input semantic='NORMAL' source='#Geometry-mesh002-normals' offset='1' />\
                              <input semantic='TEXCOORD' source='#Geometry-mesh002-Texture' offset='2' set='0' />\
                              <p>4 0 0 7 0 1 5 0 2 6 0 3 5 0 2 7 0 1 7 1 1 4 1 4 3 1 5 0 1 6 3 1 5 4 1 4 6 2 3 7 2 1 2 2 7 3 2 5 2 2 7 7 2 1 5 3 8 6 3 3 1 3 9 2 3 7 1 3 9 6 3 3 4 4 10 5 4 8 0 4 11 1 4 9 0 4 11 5 4 8 0 5 12 1 5 13 3 5 5 2 5 7 3 5 5 1 5 13</p>\
                          </triangles>\
                      </mesh>\
                  </geometry>\
              </library_geometries>\
              <library_visual_scenes>\
                  <visual_scene id='DefaultScene'>\
                      <node id='Geometry-mesh002Node' name='Mesh' type='NODE'>\
                          <instance_geometry url='#Geometry-mesh002'>\
                              <bind_material>\
                                  <technique_common>\
                                      <instance_material symbol='Material-Red' target='#Material-Red' />\
                                  </technique_common>\
                              </bind_material>\
                          </instance_geometry>\
                      </node>\
                  </visual_scene>\
              </library_visual_scenes>\
              <scene>\
                  <instance_visual_scene url='#DefaultScene' />\
              </scene>\
          </COLLADA>"
        );

        //Set geometry
        msgs::Geometry geom;
        geom.set_type(msgs::Geometry_Type_MESH);
        geom.mutable_mesh()->set_submesh(text);
        // std::cout << "meshgeom.set_submesh(text)" << std::endl;

        // geom.set_allocated_mesh(&meshgeom);
        // std::cout << "geom.set_allocated_mesh(&meshgeom)" << std::endl;




        // Set model
        msgs::Model model;
        model.set_name("my_mesh");
        std::cout << "model.set_name(my_mesh)" << std::endl;

        // Set pose
        msgs::Set(model.mutable_pose(), ignition::math::Pose3d(0, 0, 0, 0, 0, 0));
        // const double mass = 1.0;
        // const double radius = 0.5;
        // msgs::AddSphereLink(model, mass, radius);



        // msgs::Geometry geom;
        // geom.set_type(msgs::Geometry_Type_MESH);
        // geom.mutable_mesh()->set_submesh(text);
        // std::cout << "geom.mutable_sphere()->set_submesh(text)" << std::endl;

        msgs::AddLinkGeom(model, geom);
        // std::cout << "msgs::AddLinkGeom(model, geom)" << std::endl;

        msgs::Factory msg;
        msg.set_sdf(newModelStr.str());
        


        std::ostringstream newModelStr;
        newModelStr << "<sdf version='1.6'>"
          << msgs::ModelToSDF(model)->ToString("")
          << "</sdf>";

        std::cout << newModelStr.str() << std::endl;

        _world->InsertModelString(newModelStr.str());

        // std::cout << "model insert complete" << std::endl;











        // if (!ros::isInitialized())
        // {
        //     int argc = 0;
        //     char **argv = NULL;
        //     ros::init(argc, argv, "realsense_depth_to_mesh",
        //         ros::init_options::NoSigintHandler);
        // }
        // ROS_INFO("Ros init complete");
      
        // rs2::pipeline pipe;
        // rs2::frameset frames;
        // // rs2::config cfg;
        // // cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
        // // pipe.start(cfg);
        // pipe.start();
        // ROS_INFO("pipe started");

        // const auto window_name = "Display_Image";
        // namedWindow(window_name);

        // for(int i=0; i < 30; i ++)
        // {
        //     frames = pipe.wait_for_frames();
        // }

        // while(ros::ok())
        // {
        //     ROS_INFO("Realsense Frame");
        //     frames = pipe.wait_for_frames();
        //     rs2::depth_frame depth = frames.get_depth_frame();

        //     const int width = depth.as<rs2::video_frame>().get_width();
        //     const int height = depth.as<rs2::video_frame>().get_height();
            
        //     Mat depth_img(Size(width, height), CV_16UC1, (void*)depth.get_data(), Mat::AUTO_STEP);

        //     depth_img.convertTo(depth_img, CV_8UC1, 15 / 256.0);

        //     imshow(window_name, depth_img);






                
            // if(waitKey(10)==27) {
            //     destroyWindow("Display_Image");
            //     break;
            // }
            // ros::spinOnce();

        // }
        // sleep(120);
        // return;
    }
  };
  GZ_REGISTER_WORLD_PLUGIN(RealsenseDepthToMesh);
}  // namespace gazebo
