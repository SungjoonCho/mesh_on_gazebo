


#include <iostream>
#include "gazebo/common/ColladaLoader.hh"

int main(){

// const std::string _id = ;
std::vector<ignition::math::Vector3d> _values
std::map<unsigned int, unsigned int> _duplicates);

_duplicates = this->dataPtr->positionDuplicateMap[_id];
boost::unordered_map<ignition::math::Vector3d,
    unsigned int, Vector3Hash> unique;

// float array
for (int i=0; i<24; i += 1)
  {
    ignition::math::Vector3d vec(-0.5,-0.5,-0.5);

    vec = _transform * vec;
    _values.push_back(vec);

    // create a map of duplicate indices
    if (unique.find(vec) != unique.end())
      _duplicates[_values.size()-1] = unique[vec];
    else
      unique[vec] = _values.size()-1;
  }

  this->dataPtr->positionDuplicateMap[_id] = _duplicates;
  this->dataPtr->positionIds[_id] = _values;
}


return 0;
}