#include <string>
#include <vector>
#include <map>

#include <ignition/math/Matrix4.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Vector2.hh>

#include "gazebo/common/MeshLoader.hh"
#include "gazebo/util/system.hh"

// class TiXmlElement;

// namespace gazebo
// {
//   namespace common
//   {
//     class Material;
//     class ColladaLoaderPrivate;

//     class MyColladaLoader
//     {
//       public: MyColladaLoader();
//       public: void Load();
//     };
//   }
// }

class MyColladaLoader
{
    public: MyColladaLoader();
            void Load();
};