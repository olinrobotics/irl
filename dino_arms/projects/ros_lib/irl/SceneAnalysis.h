#ifndef _ROS_irl_SceneAnalysis_h
#define _ROS_irl_SceneAnalysis_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "irl/People.h"

namespace irl
{

  class SceneAnalysis : public ros::Msg
  {
    public:
      typedef std_msgs::Header _Header_type;
      _Header_type Header;
      irl::People crowd[20];

    SceneAnalysis():
      Header(),
      crowd()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->Header.serialize(outbuffer + offset);
      for( uint32_t i = 0; i < 20; i++){
      offset += this->crowd[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->Header.deserialize(inbuffer + offset);
      for( uint32_t i = 0; i < 20; i++){
      offset += this->crowd[i].deserialize(inbuffer + offset);
      }
     return offset;
    }

    const char * getType(){ return "irl/SceneAnalysis"; };
    const char * getMD5(){ return "abbff978ecc683e94cf4af6b6d52e056"; };

  };

}
#endif