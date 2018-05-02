#ifndef _ROS_irl_pointerpos_h
#define _ROS_irl_pointerpos_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace irl
{

  class pointerpos : public ros::Msg
  {
    public:
      typedef std_msgs::Header _Header_type;
      _Header_type Header;
      typedef int32_t _positionx_type;
      _positionx_type positionx;
      typedef int32_t _positiony_type;
      _positiony_type positiony;
      typedef int32_t _positionz_type;
      _positionz_type positionz;

    pointerpos():
      Header(),
      positionx(0),
      positiony(0),
      positionz(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->Header.serialize(outbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_positionx;
      u_positionx.real = this->positionx;
      *(outbuffer + offset + 0) = (u_positionx.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_positionx.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_positionx.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_positionx.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->positionx);
      union {
        int32_t real;
        uint32_t base;
      } u_positiony;
      u_positiony.real = this->positiony;
      *(outbuffer + offset + 0) = (u_positiony.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_positiony.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_positiony.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_positiony.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->positiony);
      union {
        int32_t real;
        uint32_t base;
      } u_positionz;
      u_positionz.real = this->positionz;
      *(outbuffer + offset + 0) = (u_positionz.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_positionz.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_positionz.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_positionz.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->positionz);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->Header.deserialize(inbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_positionx;
      u_positionx.base = 0;
      u_positionx.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_positionx.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_positionx.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_positionx.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->positionx = u_positionx.real;
      offset += sizeof(this->positionx);
      union {
        int32_t real;
        uint32_t base;
      } u_positiony;
      u_positiony.base = 0;
      u_positiony.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_positiony.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_positiony.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_positiony.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->positiony = u_positiony.real;
      offset += sizeof(this->positiony);
      union {
        int32_t real;
        uint32_t base;
      } u_positionz;
      u_positionz.base = 0;
      u_positionz.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_positionz.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_positionz.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_positionz.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->positionz = u_positionz.real;
      offset += sizeof(this->positionz);
     return offset;
    }

    const char * getType(){ return "irl/pointerpos"; };
    const char * getMD5(){ return "d851c36561a81b9f83e99031d0c61d04"; };

  };

}
#endif