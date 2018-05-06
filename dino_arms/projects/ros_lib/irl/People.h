#ifndef _ROS_irl_People_h
#define _ROS_irl_People_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace irl
{

  class People : public ros::Msg
  {
    public:
      typedef std_msgs::Header _Header_type;
      _Header_type Header;
      typedef int32_t _ID_type;
      _ID_type ID;
      typedef int32_t _xpos_type;
      _xpos_type xpos;
      typedef int32_t _ypos_type;
      _ypos_type ypos;
      typedef int32_t _zpos_type;
      _zpos_type zpos;

    People():
      Header(),
      ID(0),
      xpos(0),
      ypos(0),
      zpos(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->Header.serialize(outbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_ID;
      u_ID.real = this->ID;
      *(outbuffer + offset + 0) = (u_ID.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ID.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ID.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ID.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ID);
      union {
        int32_t real;
        uint32_t base;
      } u_xpos;
      u_xpos.real = this->xpos;
      *(outbuffer + offset + 0) = (u_xpos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_xpos.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_xpos.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_xpos.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->xpos);
      union {
        int32_t real;
        uint32_t base;
      } u_ypos;
      u_ypos.real = this->ypos;
      *(outbuffer + offset + 0) = (u_ypos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ypos.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ypos.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ypos.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ypos);
      union {
        int32_t real;
        uint32_t base;
      } u_zpos;
      u_zpos.real = this->zpos;
      *(outbuffer + offset + 0) = (u_zpos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_zpos.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_zpos.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_zpos.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->zpos);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->Header.deserialize(inbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_ID;
      u_ID.base = 0;
      u_ID.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ID.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ID.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ID.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ID = u_ID.real;
      offset += sizeof(this->ID);
      union {
        int32_t real;
        uint32_t base;
      } u_xpos;
      u_xpos.base = 0;
      u_xpos.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_xpos.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_xpos.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_xpos.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->xpos = u_xpos.real;
      offset += sizeof(this->xpos);
      union {
        int32_t real;
        uint32_t base;
      } u_ypos;
      u_ypos.base = 0;
      u_ypos.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ypos.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ypos.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ypos.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ypos = u_ypos.real;
      offset += sizeof(this->ypos);
      union {
        int32_t real;
        uint32_t base;
      } u_zpos;
      u_zpos.base = 0;
      u_zpos.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_zpos.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_zpos.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_zpos.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->zpos = u_zpos.real;
      offset += sizeof(this->zpos);
     return offset;
    }

    const char * getType(){ return "irl/People"; };
    const char * getMD5(){ return "2dfe973f5185b9ead36a7b7d3f420ca5"; };

  };

}
#endif