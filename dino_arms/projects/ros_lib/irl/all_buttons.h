#ifndef _ROS_irl_all_buttons_h
#define _ROS_irl_all_buttons_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "irl/red_button.h"
#include "irl/green_button.h"
#include "irl/blue_button.h"
#include "irl/yellow_button.h"

namespace irl
{

  class all_buttons : public ros::Msg
  {
    public:
      typedef irl::red_button _red_button_type;
      _red_button_type red_button;
      typedef irl::green_button _green_button_type;
      _green_button_type green_button;
      typedef irl::blue_button _blue_button_type;
      _blue_button_type blue_button;
      typedef irl::yellow_button _yellow_button_type;
      _yellow_button_type yellow_button;

    all_buttons():
      red_button(),
      green_button(),
      blue_button(),
      yellow_button()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->red_button.serialize(outbuffer + offset);
      offset += this->green_button.serialize(outbuffer + offset);
      offset += this->blue_button.serialize(outbuffer + offset);
      offset += this->yellow_button.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->red_button.deserialize(inbuffer + offset);
      offset += this->green_button.deserialize(inbuffer + offset);
      offset += this->blue_button.deserialize(inbuffer + offset);
      offset += this->yellow_button.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "irl/all_buttons"; };
    const char * getMD5(){ return "3256369db9b8b82015849425c82c02f9"; };

  };

}
#endif