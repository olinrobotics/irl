#ifndef _ROS_irl_Real_Structure_h
#define _ROS_irl_Real_Structure_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "irl/Real_Cube.h"

namespace irl
{

  class Real_Structure : public ros::Msg
  {
    public:
      uint32_t building_length;
      typedef irl::Real_Cube _building_type;
      _building_type st_building;
      _building_type * building;

    Real_Structure():
      building_length(0), building(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->building_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->building_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->building_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->building_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->building_length);
      for( uint32_t i = 0; i < building_length; i++){
      offset += this->building[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t building_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      building_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      building_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      building_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->building_length);
      if(building_lengthT > building_length)
        this->building = (irl::Real_Cube*)realloc(this->building, building_lengthT * sizeof(irl::Real_Cube));
      building_length = building_lengthT;
      for( uint32_t i = 0; i < building_length; i++){
      offset += this->st_building.deserialize(inbuffer + offset);
        memcpy( &(this->building[i]), &(this->st_building), sizeof(irl::Real_Cube));
      }
     return offset;
    }

    const char * getType(){ return "irl/Real_Structure"; };
    const char * getMD5(){ return "33dc3b1ca22139cde0d862e1afb4a6bc"; };

  };

}
#endif