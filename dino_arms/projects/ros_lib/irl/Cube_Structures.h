#ifndef _ROS_irl_Cube_Structures_h
#define _ROS_irl_Cube_Structures_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "irl/Real_Structure.h"
#include "irl/Grid_Structure.h"

namespace irl
{

  class Cube_Structures : public ros::Msg
  {
    public:
      typedef irl::Real_Structure _real_building_type;
      _real_building_type real_building;
      typedef irl::Grid_Structure _grid_building_type;
      _grid_building_type grid_building;

    Cube_Structures():
      real_building(),
      grid_building()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->real_building.serialize(outbuffer + offset);
      offset += this->grid_building.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->real_building.deserialize(inbuffer + offset);
      offset += this->grid_building.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "irl/Cube_Structures"; };
    const char * getMD5(){ return "821d55d03fa8397d870092e00b1b1236"; };

  };

}
#endif