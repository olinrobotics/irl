#ifndef _ROS_irl_edwin_gestures_h
#define _ROS_irl_edwin_gestures_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace irl
{

  class edwin_gestures : public ros::Msg
  {
    public:
      typedef int32_t _cmd_type;
      _cmd_type cmd;
      uint32_t param_length;
      typedef float _param_type;
      _param_type st_param;
      _param_type * param;
      typedef bool _wave_type;
      _wave_type wave;
      typedef bool _hello_type;
      _hello_type hello;
      typedef bool _goodbye_type;
      _goodbye_type goodbye;

    edwin_gestures():
      cmd(0),
      param_length(0), param(NULL),
      wave(0),
      hello(0),
      goodbye(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_cmd;
      u_cmd.real = this->cmd;
      *(outbuffer + offset + 0) = (u_cmd.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cmd.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cmd.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cmd.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cmd);
      *(outbuffer + offset + 0) = (this->param_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->param_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->param_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->param_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->param_length);
      for( uint32_t i = 0; i < param_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->param[i]);
      }
      union {
        bool real;
        uint8_t base;
      } u_wave;
      u_wave.real = this->wave;
      *(outbuffer + offset + 0) = (u_wave.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->wave);
      union {
        bool real;
        uint8_t base;
      } u_hello;
      u_hello.real = this->hello;
      *(outbuffer + offset + 0) = (u_hello.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->hello);
      union {
        bool real;
        uint8_t base;
      } u_goodbye;
      u_goodbye.real = this->goodbye;
      *(outbuffer + offset + 0) = (u_goodbye.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->goodbye);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_cmd;
      u_cmd.base = 0;
      u_cmd.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cmd.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cmd.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cmd.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cmd = u_cmd.real;
      offset += sizeof(this->cmd);
      uint32_t param_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      param_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      param_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      param_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->param_length);
      if(param_lengthT > param_length)
        this->param = (float*)realloc(this->param, param_lengthT * sizeof(float));
      param_length = param_lengthT;
      for( uint32_t i = 0; i < param_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_param));
        memcpy( &(this->param[i]), &(this->st_param), sizeof(float));
      }
      union {
        bool real;
        uint8_t base;
      } u_wave;
      u_wave.base = 0;
      u_wave.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->wave = u_wave.real;
      offset += sizeof(this->wave);
      union {
        bool real;
        uint8_t base;
      } u_hello;
      u_hello.base = 0;
      u_hello.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->hello = u_hello.real;
      offset += sizeof(this->hello);
      union {
        bool real;
        uint8_t base;
      } u_goodbye;
      u_goodbye.base = 0;
      u_goodbye.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->goodbye = u_goodbye.real;
      offset += sizeof(this->goodbye);
     return offset;
    }

    const char * getType(){ return "irl/edwin_gestures"; };
    const char * getMD5(){ return "5d9a0586df70bbae807bb3ddde4eecd5"; };

  };

}
#endif