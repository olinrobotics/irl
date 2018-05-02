#ifndef _ROS_irl_HHH_h
#define _ROS_irl_HHH_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace irl
{

  class HHH : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef int32_t _headx_type;
      _headx_type headx;
      typedef int32_t _heady_type;
      _heady_type heady;
      typedef int32_t _headz_type;
      _headz_type headz;
      typedef int32_t _torsox_type;
      _torsox_type torsox;
      typedef int32_t _torsoy_type;
      _torsoy_type torsoy;
      typedef int32_t _torsoz_type;
      _torsoz_type torsoz;
      typedef int32_t _rhandx_type;
      _rhandx_type rhandx;
      typedef int32_t _rhandy_type;
      _rhandy_type rhandy;
      typedef int32_t _rhandz_type;
      _rhandz_type rhandz;
      typedef int32_t _lhandx_type;
      _lhandx_type lhandx;
      typedef int32_t _lhandy_type;
      _lhandy_type lhandy;
      typedef int32_t _lhandz_type;
      _lhandz_type lhandz;

    HHH():
      header(),
      headx(0),
      heady(0),
      headz(0),
      torsox(0),
      torsoy(0),
      torsoz(0),
      rhandx(0),
      rhandy(0),
      rhandz(0),
      lhandx(0),
      lhandy(0),
      lhandz(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_headx;
      u_headx.real = this->headx;
      *(outbuffer + offset + 0) = (u_headx.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_headx.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_headx.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_headx.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->headx);
      union {
        int32_t real;
        uint32_t base;
      } u_heady;
      u_heady.real = this->heady;
      *(outbuffer + offset + 0) = (u_heady.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_heady.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_heady.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_heady.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->heady);
      union {
        int32_t real;
        uint32_t base;
      } u_headz;
      u_headz.real = this->headz;
      *(outbuffer + offset + 0) = (u_headz.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_headz.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_headz.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_headz.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->headz);
      union {
        int32_t real;
        uint32_t base;
      } u_torsox;
      u_torsox.real = this->torsox;
      *(outbuffer + offset + 0) = (u_torsox.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_torsox.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_torsox.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_torsox.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->torsox);
      union {
        int32_t real;
        uint32_t base;
      } u_torsoy;
      u_torsoy.real = this->torsoy;
      *(outbuffer + offset + 0) = (u_torsoy.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_torsoy.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_torsoy.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_torsoy.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->torsoy);
      union {
        int32_t real;
        uint32_t base;
      } u_torsoz;
      u_torsoz.real = this->torsoz;
      *(outbuffer + offset + 0) = (u_torsoz.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_torsoz.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_torsoz.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_torsoz.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->torsoz);
      union {
        int32_t real;
        uint32_t base;
      } u_rhandx;
      u_rhandx.real = this->rhandx;
      *(outbuffer + offset + 0) = (u_rhandx.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rhandx.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rhandx.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rhandx.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rhandx);
      union {
        int32_t real;
        uint32_t base;
      } u_rhandy;
      u_rhandy.real = this->rhandy;
      *(outbuffer + offset + 0) = (u_rhandy.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rhandy.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rhandy.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rhandy.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rhandy);
      union {
        int32_t real;
        uint32_t base;
      } u_rhandz;
      u_rhandz.real = this->rhandz;
      *(outbuffer + offset + 0) = (u_rhandz.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rhandz.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rhandz.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rhandz.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rhandz);
      union {
        int32_t real;
        uint32_t base;
      } u_lhandx;
      u_lhandx.real = this->lhandx;
      *(outbuffer + offset + 0) = (u_lhandx.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_lhandx.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_lhandx.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_lhandx.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->lhandx);
      union {
        int32_t real;
        uint32_t base;
      } u_lhandy;
      u_lhandy.real = this->lhandy;
      *(outbuffer + offset + 0) = (u_lhandy.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_lhandy.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_lhandy.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_lhandy.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->lhandy);
      union {
        int32_t real;
        uint32_t base;
      } u_lhandz;
      u_lhandz.real = this->lhandz;
      *(outbuffer + offset + 0) = (u_lhandz.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_lhandz.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_lhandz.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_lhandz.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->lhandz);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_headx;
      u_headx.base = 0;
      u_headx.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_headx.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_headx.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_headx.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->headx = u_headx.real;
      offset += sizeof(this->headx);
      union {
        int32_t real;
        uint32_t base;
      } u_heady;
      u_heady.base = 0;
      u_heady.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_heady.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_heady.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_heady.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->heady = u_heady.real;
      offset += sizeof(this->heady);
      union {
        int32_t real;
        uint32_t base;
      } u_headz;
      u_headz.base = 0;
      u_headz.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_headz.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_headz.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_headz.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->headz = u_headz.real;
      offset += sizeof(this->headz);
      union {
        int32_t real;
        uint32_t base;
      } u_torsox;
      u_torsox.base = 0;
      u_torsox.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_torsox.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_torsox.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_torsox.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->torsox = u_torsox.real;
      offset += sizeof(this->torsox);
      union {
        int32_t real;
        uint32_t base;
      } u_torsoy;
      u_torsoy.base = 0;
      u_torsoy.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_torsoy.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_torsoy.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_torsoy.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->torsoy = u_torsoy.real;
      offset += sizeof(this->torsoy);
      union {
        int32_t real;
        uint32_t base;
      } u_torsoz;
      u_torsoz.base = 0;
      u_torsoz.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_torsoz.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_torsoz.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_torsoz.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->torsoz = u_torsoz.real;
      offset += sizeof(this->torsoz);
      union {
        int32_t real;
        uint32_t base;
      } u_rhandx;
      u_rhandx.base = 0;
      u_rhandx.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rhandx.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rhandx.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rhandx.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rhandx = u_rhandx.real;
      offset += sizeof(this->rhandx);
      union {
        int32_t real;
        uint32_t base;
      } u_rhandy;
      u_rhandy.base = 0;
      u_rhandy.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rhandy.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rhandy.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rhandy.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rhandy = u_rhandy.real;
      offset += sizeof(this->rhandy);
      union {
        int32_t real;
        uint32_t base;
      } u_rhandz;
      u_rhandz.base = 0;
      u_rhandz.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rhandz.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rhandz.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rhandz.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rhandz = u_rhandz.real;
      offset += sizeof(this->rhandz);
      union {
        int32_t real;
        uint32_t base;
      } u_lhandx;
      u_lhandx.base = 0;
      u_lhandx.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_lhandx.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_lhandx.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_lhandx.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->lhandx = u_lhandx.real;
      offset += sizeof(this->lhandx);
      union {
        int32_t real;
        uint32_t base;
      } u_lhandy;
      u_lhandy.base = 0;
      u_lhandy.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_lhandy.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_lhandy.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_lhandy.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->lhandy = u_lhandy.real;
      offset += sizeof(this->lhandy);
      union {
        int32_t real;
        uint32_t base;
      } u_lhandz;
      u_lhandz.base = 0;
      u_lhandz.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_lhandz.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_lhandz.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_lhandz.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->lhandz = u_lhandz.real;
      offset += sizeof(this->lhandz);
     return offset;
    }

    const char * getType(){ return "irl/HHH"; };
    const char * getMD5(){ return "be0d3a6634893483c264cea91b3ec41b"; };

  };

}
#endif