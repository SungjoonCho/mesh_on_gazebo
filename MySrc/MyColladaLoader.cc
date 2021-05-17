#include <curl/curl.h>
#include <tinyxml.h>
#include <math.h>
#include <sstream>
#include <set>
#include <memory>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/unordered_map.hpp>
#include <iostream>

#include <ignition/math/Color.hh>
#include <ignition/math/Helpers.hh>
#include <ignition/math/Matrix4.hh>
#include <ignition/math/Vector2.hh>
#include <ignition/math/Vector3.hh>

#include "gazebo/common/Console.hh"
#include "gazebo/common/Material.hh"
#include "gazebo/common/Mesh.hh"
#include "gazebo/common/Skeleton.hh"
#include "gazebo/common/SkeletonAnimation.hh"
#include "gazebo/common/SystemPaths.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/ColladaLoaderPrivate.hh"
#include "gazebo/common/ColladaLoader.hh"

// using namespace gazebo;
// using namespace common;

MyColladaLoader::MyColladaLoader()
: dataPtr(new ColladaLoaderPrivate)
{
  this->dataPtr->meter = 1.0;
  std::cout << this->dataPtr->meter;
  ss
}

void MyColladaLoader::Load()
{
  this->dataPtr->positionIds.clear();
  this->dataPtr->normalIds.clear();
  this->dataPtr->texcoordIds.clear();
  this->dataPtr->materialIds.clear();
  this->dataPtr->positionDuplicateMap.clear();
  this->dataPtr->normalDuplicateMap.clear();
  this->dataPtr->texcoordDuplicateMap.clear();

  // reset scale
  this->dataPtr->meter = 1.0;
  std::cout << this->dataPtr->meter;

  // TiXmlDocument xmlDoc;

//   boost::filesystem::path p(_filename);
//   this->dataPtr->path = p.parent_path().generic_string();

//   this->dataPtr->filename = _filename;
//   if (!xmlDoc.LoadFile(_filename))
//     gzerr << "Unable to load collada file[" << _filename << "]\n";

//   this->dataPtr->colladaXml = xmlDoc.FirstChildElement("COLLADA");
//   if (!this->dataPtr->colladaXml)
//     gzerr << "Missing COLLADA tag\n";

//   if (std::string(this->dataPtr->colladaXml->Attribute("version")) != "1.4.0" &&
//       std::string(this->dataPtr->colladaXml->Attribute("version")) != "1.4.1")
//     gzerr << "Invalid collada file. Must be version 1.4.0 or 1.4.1\n";

//   TiXmlElement *assetXml =
//       this->dataPtr->colladaXml->FirstChildElement("asset");
//   if (assetXml)
//   {
//     TiXmlElement *unitXml = assetXml->FirstChildElement("unit");
//     if (unitXml && unitXml->Attribute("meter"))
//       this->dataPtr->meter = ignition::math::parseFloat(
//           unitXml->Attribute("meter"));
//   }

  // Mesh *mesh = new Mesh();
//   mesh->SetPath(this->dataPtr->path);

//   this->LoadScene(mesh);

//   if (mesh->HasSkeleton())
//     ApplyInvBindTransform(mesh->GetSkeleton());

//   // This will make the model the correct size.
//   mesh->Scale(this->dataPtr->meter);
//   if (mesh->HasSkeleton())
//     mesh->GetSkeleton()->Scale(this->dataPtr->meter);

  // return mesh;
}