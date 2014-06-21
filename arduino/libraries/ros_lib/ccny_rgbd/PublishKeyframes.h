#ifndef _ROS_SERVICE_PublishKeyframes_h
#define _ROS_SERVICE_PublishKeyframes_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ccny_rgbd
{

static const char PUBLISHKEYFRAMES[] = "ccny_rgbd/PublishKeyframes";

  class PublishKeyframesRequest : public ros::Msg
  {
    public:
      char * re;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_re = strlen( (const char*) this->re);
      memcpy(outbuffer + offset, &length_re, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->re, length_re);
      offset += length_re;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_re;
      memcpy(&length_re, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_re; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_re-1]=0;
      this->re = (char *)(inbuffer + offset-1);
      offset += length_re;
     return offset;
    }

    const char * getType(){ return PUBLISHKEYFRAMES; };
    const char * getMD5(){ return "a91fd359d10bd35fdc46f24ffc722fa2"; };

  };

  class PublishKeyframesResponse : public ros::Msg
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

    const char * getType(){ return PUBLISHKEYFRAMES; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class PublishKeyframes {
    public:
    typedef PublishKeyframesRequest Request;
    typedef PublishKeyframesResponse Response;
  };

}
#endif
