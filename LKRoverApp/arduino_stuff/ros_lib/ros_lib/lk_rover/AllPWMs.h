#ifndef _ROS_lk_rover_AllPWMs_h
#define _ROS_lk_rover_AllPWMs_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace lk_rover
{

  class AllPWMs : public ros::Msg
  {
    public:
      typedef float _bucket_left_type;
      _bucket_left_type bucket_left;
      typedef float _bucket_right_type;
      _bucket_right_type bucket_right;
      typedef float _bucket_spin_type;
      _bucket_spin_type bucket_spin;
      typedef float _bucket_flap_type;
      _bucket_flap_type bucket_flap;
      typedef float _ladder_left_type;
      _ladder_left_type ladder_left;
      typedef float _ladder_right_type;
      _ladder_right_type ladder_right;
      typedef float _front_left_type;
      _front_left_type front_left;
      typedef float _front_right_type;
      _front_right_type front_right;
      typedef float _back_left_type;
      _back_left_type back_left;
      typedef float _back_right_type;
      _back_right_type back_right;

    AllPWMs():
      bucket_left(0),
      bucket_right(0),
      bucket_spin(0),
      bucket_flap(0),
      ladder_left(0),
      ladder_right(0),
      front_left(0),
      front_right(0),
      back_left(0),
      back_right(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_bucket_left;
      u_bucket_left.real = this->bucket_left;
      *(outbuffer + offset + 0) = (u_bucket_left.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_bucket_left.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_bucket_left.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_bucket_left.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->bucket_left);
      union {
        float real;
        uint32_t base;
      } u_bucket_right;
      u_bucket_right.real = this->bucket_right;
      *(outbuffer + offset + 0) = (u_bucket_right.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_bucket_right.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_bucket_right.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_bucket_right.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->bucket_right);
      union {
        float real;
        uint32_t base;
      } u_bucket_spin;
      u_bucket_spin.real = this->bucket_spin;
      *(outbuffer + offset + 0) = (u_bucket_spin.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_bucket_spin.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_bucket_spin.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_bucket_spin.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->bucket_spin);
      union {
        float real;
        uint32_t base;
      } u_bucket_flap;
      u_bucket_flap.real = this->bucket_flap;
      *(outbuffer + offset + 0) = (u_bucket_flap.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_bucket_flap.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_bucket_flap.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_bucket_flap.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->bucket_flap);
      union {
        float real;
        uint32_t base;
      } u_ladder_left;
      u_ladder_left.real = this->ladder_left;
      *(outbuffer + offset + 0) = (u_ladder_left.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ladder_left.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ladder_left.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ladder_left.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ladder_left);
      union {
        float real;
        uint32_t base;
      } u_ladder_right;
      u_ladder_right.real = this->ladder_right;
      *(outbuffer + offset + 0) = (u_ladder_right.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ladder_right.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ladder_right.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ladder_right.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ladder_right);
      union {
        float real;
        uint32_t base;
      } u_front_left;
      u_front_left.real = this->front_left;
      *(outbuffer + offset + 0) = (u_front_left.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_front_left.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_front_left.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_front_left.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->front_left);
      union {
        float real;
        uint32_t base;
      } u_front_right;
      u_front_right.real = this->front_right;
      *(outbuffer + offset + 0) = (u_front_right.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_front_right.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_front_right.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_front_right.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->front_right);
      union {
        float real;
        uint32_t base;
      } u_back_left;
      u_back_left.real = this->back_left;
      *(outbuffer + offset + 0) = (u_back_left.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_back_left.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_back_left.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_back_left.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->back_left);
      union {
        float real;
        uint32_t base;
      } u_back_right;
      u_back_right.real = this->back_right;
      *(outbuffer + offset + 0) = (u_back_right.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_back_right.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_back_right.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_back_right.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->back_right);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_bucket_left;
      u_bucket_left.base = 0;
      u_bucket_left.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_bucket_left.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_bucket_left.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_bucket_left.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->bucket_left = u_bucket_left.real;
      offset += sizeof(this->bucket_left);
      union {
        float real;
        uint32_t base;
      } u_bucket_right;
      u_bucket_right.base = 0;
      u_bucket_right.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_bucket_right.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_bucket_right.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_bucket_right.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->bucket_right = u_bucket_right.real;
      offset += sizeof(this->bucket_right);
      union {
        float real;
        uint32_t base;
      } u_bucket_spin;
      u_bucket_spin.base = 0;
      u_bucket_spin.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_bucket_spin.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_bucket_spin.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_bucket_spin.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->bucket_spin = u_bucket_spin.real;
      offset += sizeof(this->bucket_spin);
      union {
        float real;
        uint32_t base;
      } u_bucket_flap;
      u_bucket_flap.base = 0;
      u_bucket_flap.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_bucket_flap.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_bucket_flap.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_bucket_flap.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->bucket_flap = u_bucket_flap.real;
      offset += sizeof(this->bucket_flap);
      union {
        float real;
        uint32_t base;
      } u_ladder_left;
      u_ladder_left.base = 0;
      u_ladder_left.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ladder_left.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ladder_left.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ladder_left.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ladder_left = u_ladder_left.real;
      offset += sizeof(this->ladder_left);
      union {
        float real;
        uint32_t base;
      } u_ladder_right;
      u_ladder_right.base = 0;
      u_ladder_right.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ladder_right.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ladder_right.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ladder_right.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ladder_right = u_ladder_right.real;
      offset += sizeof(this->ladder_right);
      union {
        float real;
        uint32_t base;
      } u_front_left;
      u_front_left.base = 0;
      u_front_left.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_front_left.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_front_left.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_front_left.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->front_left = u_front_left.real;
      offset += sizeof(this->front_left);
      union {
        float real;
        uint32_t base;
      } u_front_right;
      u_front_right.base = 0;
      u_front_right.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_front_right.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_front_right.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_front_right.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->front_right = u_front_right.real;
      offset += sizeof(this->front_right);
      union {
        float real;
        uint32_t base;
      } u_back_left;
      u_back_left.base = 0;
      u_back_left.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_back_left.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_back_left.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_back_left.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->back_left = u_back_left.real;
      offset += sizeof(this->back_left);
      union {
        float real;
        uint32_t base;
      } u_back_right;
      u_back_right.base = 0;
      u_back_right.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_back_right.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_back_right.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_back_right.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->back_right = u_back_right.real;
      offset += sizeof(this->back_right);
     return offset;
    }

    const char * getType(){ return "lk_rover/AllPWMs"; };
    const char * getMD5(){ return "eb0735f9bb116dadf6ca9b80fd5313e6"; };

  };

}
#endif