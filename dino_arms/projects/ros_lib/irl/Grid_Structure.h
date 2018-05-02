#ifndef _ROS_irl_Grid_Structure_h
#define _ROS_irl_Grid_Structure_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "irl/Grid_Cube.h"

namespace irl
{

  class Grid_Structure : public ros::Msg
  {
    public:
      uint32_t building_length;
      typedef irl::Grid_Cube _building_type;
      _building_type st_building;
      _building_type * building;

    Grid_Structure():
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
        this->building = (irl::Grid_Cube*)realloc(this->building, building_lengthT * sizeof(irl::Grid_Cube));
      building_length = building_lengthT;
      for( uint32_t i = 0; i < building_length; i++){
      offset += this->st_building.deserialize(inbuffer + offset);
        memcpy( &(this->building[i]), &(this->st_building), sizeof(irl::Grid_Cube));
      }
     return offset;
    }

    const char * getType(){ return "irl/Grid_Structure"; };
    const char * getMD5(){ return "f88a3db031c30b7feb92a5dce56106c4"; };

  };

}
#endif