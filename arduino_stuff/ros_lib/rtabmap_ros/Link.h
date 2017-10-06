#ifndef _ROS_rtabmap_ros_Link_h
#define _ROS_rtabmap_ros_Link_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Transform.h"

namespace rtabmap_ros
{

  class Link : public ros::Msg
  {
    public:
      typedef int32_t _fromId_type;
      _fromId_type fromId;
      typedef int32_t _toId_type;
      _toId_type toId;
      typedef int32_t _type_type;
      _type_type type;
      typedef geometry_msgs::Transform _transform_type;
      _transform_type transform;
      typedef float _rotVariance_type;
      _rotVariance_type rotVariance;
      typedef float _transVariance_type;
      _transVariance_type transVariance;

    Link():
      fromId(0),
      toId(0),
      type(0),
      transform(),
      rotVariance(0),
      transVariance(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_fromId;
      u_fromId.real = this->fromId;
      *(outbuffer + offset + 0) = (u_fromId.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_fromId.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_fromId.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_fromId.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->fromId);
      union {
        int32_t real;
        uint32_t base;
      } u_toId;
      u_toId.real = this->toId;
      *(outbuffer + offset + 0) = (u_toId.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_toId.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_toId.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_toId.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->toId);
      union {
        int32_t real;
        uint32_t base;
      } u_type;
      u_type.real = this->type;
      *(outbuffer + offset + 0) = (u_type.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_type.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_type.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_type.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->type);
      offset += this->transform.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_rotVariance;
      u_rotVariance.real = this->rotVariance;
      *(outbuffer + offset + 0) = (u_rotVariance.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rotVariance.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rotVariance.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rotVariance.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rotVariance);
      union {
        float real;
        uint32_t base;
      } u_transVariance;
      u_transVariance.real = this->transVariance;
      *(outbuffer + offset + 0) = (u_transVariance.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_transVariance.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_transVariance.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_transVariance.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->transVariance);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_fromId;
      u_fromId.base = 0;
      u_fromId.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_fromId.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_fromId.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_fromId.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->fromId = u_fromId.real;
      offset += sizeof(this->fromId);
      union {
        int32_t real;
        uint32_t base;
      } u_toId;
      u_toId.base = 0;
      u_toId.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_toId.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_toId.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_toId.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->toId = u_toId.real;
      offset += sizeof(this->toId);
      union {
        int32_t real;
        uint32_t base;
      } u_type;
      u_type.base = 0;
      u_type.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_type.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_type.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_type.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->type = u_type.real;
      offset += sizeof(this->type);
      offset += this->transform.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_rotVariance;
      u_rotVariance.base = 0;
      u_rotVariance.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rotVariance.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rotVariance.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rotVariance.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rotVariance = u_rotVariance.real;
      offset += sizeof(this->rotVariance);
      union {
        float real;
        uint32_t base;
      } u_transVariance;
      u_transVariance.base = 0;
      u_transVariance.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_transVariance.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_transVariance.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_transVariance.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->transVariance = u_transVariance.real;
      offset += sizeof(this->transVariance);
     return offset;
    }

    const char * getType(){ return "rtabmap_ros/Link"; };
    const char * getMD5(){ return "aab00d57f13a2febcbee54fbb44691d2"; };

  };

}
#endif