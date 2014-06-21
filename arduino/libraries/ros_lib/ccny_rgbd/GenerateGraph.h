#ifndef _ROS_SERVICE_GenerateGraph_h
#define _ROS_SERVICE_GenerateGraph_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ccny_rgbd
{

static const char GENERATEGRAPH[] = "ccny_rgbd/GenerateGraph";

  class GenerateGraphRequest : public ros::Msg
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

    const char * getType(){ return GENERATEGRAPH; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GenerateGraphResponse : public ros::Msg
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

    const char * getType(){ return GENERATEGRAPH; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GenerateGraph {
    public:
    typedef GenerateGraphRequest Request;
    typedef GenerateGraphResponse Response;
  };

}
#endif
