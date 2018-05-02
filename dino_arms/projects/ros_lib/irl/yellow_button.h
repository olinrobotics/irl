#ifndef _ROS_irl_yellow_button_h
#define _ROS_irl_yellow_button_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace irl
{

  class yellow_button : public ros::Msg
  {
    public:
      typedef float _radius_type;
      _radius_type radius;
      typedef float _center_x_type;
      _center_x_type center_x;
      typedef float _center_y_type;
      _center_y_type center_y;

    yellow_button():
      radius(0),
      center_x(0),
      center_y(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->radius);
      offset += serializeAvrFloat64(outbuffer + offset, this->center_x);
      offset += serializeAvrFloat64(outbuffer + offset, this->center_y);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->radius));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->center_x));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->center_y));
     return offset;
    }

    const char * getType(){ return "irl/yellow_button"; };
    const char * getMD5(){ return "c0e860b50c1b5a35667b2212b254be76"; };

  };

}
#endif