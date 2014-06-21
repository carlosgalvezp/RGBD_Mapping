#ifndef _ROS_SERVICE_SolveGraph_h
#define _ROS_SERVICE_SolveGraph_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ccny_rgbd
{

static const char SOLVEGRAPH[] = "ccny_rgbd/SolveGraph";

  class SolveGraphRequest : public ros::Msg
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

    const char * getType(){ return SOLVEGRAPH; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class SolveGraphResponse : public ros::Msg
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

    const char * getType(){ return SOLVEGRAPH; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class SolveGraph {
    public:
    typedef SolveGraphRequest Request;
    typedef SolveGraphResponse Response;
  };

}
#endif
