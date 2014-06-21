#ifndef _ROS_SERVICE_AddManualKeyframe_h
#define _ROS_SERVICE_AddManualKeyframe_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ccny_rgbd
{

static const char ADDMANUALKEYFRAME[] = "ccny_rgbd/AddManualKeyframe";

  class AddManualKeyframeRequest : public ros::Msg
  {
    public:

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return ADDMANUALKEYFRAME; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class AddManualKeyframeResponse : public ros::Msg
  {
    public:

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return ADDMANUALKEYFRAME; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class AddManualKeyframe {
    public:
    typedef AddManualKeyframeRequest Request;
    typedef AddManualKeyframeResponse Response;
  };

}
#endif
