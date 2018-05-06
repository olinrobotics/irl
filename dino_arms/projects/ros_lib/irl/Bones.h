#ifndef _ROS_irl_Bones_h
#define _ROS_irl_Bones_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Point.h"

namespace irl
{

  class Bones : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef geometry_msgs::Point _h_type;
      _h_type h;
      typedef geometry_msgs::Point _n_type;
      _n_type n;
      typedef geometry_msgs::Point _t_type;
      _t_type t;
      typedef geometry_msgs::Point _rs_type;
      _rs_type rs;
      typedef geometry_msgs::Point _re_type;
      _re_type re;
      typedef geometry_msgs::Point _rh_type;
      _rh_type rh;
      typedef geometry_msgs::Point _ls_type;
      _ls_type ls;
      typedef geometry_msgs::Point _le_type;
      _le_type le;
      typedef geometry_msgs::Point _lh_type;
      _lh_type lh;
      typedef geometry_msgs::Point _rp_type;
      _rp_type rp;
      typedef geometry_msgs::Point _rk_type;
      _rk_type rk;
      typedef geometry_msgs::Point _rf_type;
      _rf_type rf;
      typedef geometry_msgs::Point _lp_type;
      _lp_type lp;
      typedef geometry_msgs::Point _lk_type;
      _lk_type lk;
      typedef geometry_msgs::Point _lf_type;
      _lf_type lf;

    Bones():
      header(),
      h(),
      n(),
      t(),
      rs(),
      re(),
      rh(),
      ls(),
      le(),
      lh(),
      rp(),
      rk(),
      rf(),
      lp(),
      lk(),
      lf()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->h.serialize(outbuffer + offset);
      offset += this->n.serialize(outbuffer + offset);
      offset += this->t.serialize(outbuffer + offset);
      offset += this->rs.serialize(outbuffer + offset);
      offset += this->re.serialize(outbuffer + offset);
      offset += this->rh.serialize(outbuffer + offset);
      offset += this->ls.serialize(outbuffer + offset);
      offset += this->le.serialize(outbuffer + offset);
      offset += this->lh.serialize(outbuffer + offset);
      offset += this->rp.serialize(outbuffer + offset);
      offset += this->rk.serialize(outbuffer + offset);
      offset += this->rf.serialize(outbuffer + offset);
      offset += this->lp.serialize(outbuffer + offset);
      offset += this->lk.serialize(outbuffer + offset);
      offset += this->lf.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->h.deserialize(inbuffer + offset);
      offset += this->n.deserialize(inbuffer + offset);
      offset += this->t.deserialize(inbuffer + offset);
      offset += this->rs.deserialize(inbuffer + offset);
      offset += this->re.deserialize(inbuffer + offset);
      offset += this->rh.deserialize(inbuffer + offset);
      offset += this->ls.deserialize(inbuffer + offset);
      offset += this->le.deserialize(inbuffer + offset);
      offset += this->lh.deserialize(inbuffer + offset);
      offset += this->rp.deserialize(inbuffer + offset);
      offset += this->rk.deserialize(inbuffer + offset);
      offset += this->rf.deserialize(inbuffer + offset);
      offset += this->lp.deserialize(inbuffer + offset);
      offset += this->lk.deserialize(inbuffer + offset);
      offset += this->lf.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "irl/Bones"; };
    const char * getMD5(){ return "21ed4d5e4c234c54be317020c0255870"; };

  };

}
#endif