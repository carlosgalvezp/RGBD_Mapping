#ifndef _ROS_SERVICE_Save_h
#define _ROS_SERVICE_Save_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ccny_rgbd
{

static const char SAVE[] = "ccny_rgbd/Save";

  class SaveRequest : public ros::Msg
  {
    public:
      char * filename;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_filename = strlen( (const char*) this->filename);
      memcpy(outbuffer + offset, &length_filename, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->filename, length_filename);
      offset += length_filename;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_filename;
      memcpy(&length_filename, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_filename; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_filename-1]=0;
      this->filename = (char *)(inbuffer + offset-1);
      offset += length_filename;
     return offset;
    }

    const char * getType(){ return SAVE; };
    const char * getMD5(){ return "030824f52a0628ead956fb9d67e66ae9"; };

  };

  class SaveResponse : public ros::Msg
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

    const char * getType(){ return SAVE; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class Save {
    public:
    typedef SaveRequest Request;
    typedef SaveResponse Response;
  };

}
#endif
