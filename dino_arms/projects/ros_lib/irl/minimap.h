#ifndef _ROS_irl_minimap_h
#define _ROS_irl_minimap_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "irl/blocks.h"

namespace irl
{

  class minimap : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t structure_length;
      typedef irl::blocks _structure_type;
      _structure_type st_structure;
      _structure_type * structure;

    minimap():
      header(),
      structure_length(0), structure(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->structure_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->structure_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->structure_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->structure_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->structure_length);
      for( uint32_t i = 0; i < structure_length; i++){
      offset += this->structure[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t structure_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      structure_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      structure_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      structure_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->structure_length);
      if(structure_lengthT > structure_length)
        this->structure = (irl::blocks*)realloc(this->structure, structure_lengthT * sizeof(irl::blocks));
      structure_length = structure_lengthT;
      for( uint32_t i = 0; i < structure_length; i++){
      offset += this->st_structure.deserialize(inbuffer + offset);
        memcpy( &(this->structure[i]), &(this->st_structure), sizeof(irl::blocks));
      }
     return offset;
    }

    const char * getType(){ return "irl/minimap"; };
    const char * getMD5(){ return "f6fa5805c5db2865a1621f2622fd3d69"; };

  };

}
#endif