#ifndef _ROS_lk_rover_AllEncoders_h
#define _ROS_lk_rover_AllEncoders_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace lk_rover
{
class AllEncoders : public ros::Msg
{
public:
  typedef float _bucket_left_enc_type;
  _bucket_left_enc_type bucket_left_enc;
  typedef float _bucket_right_enc_type;
  _bucket_right_enc_type bucket_right_enc;
  typedef float _ladder_left_enc_type;
  _ladder_left_enc_type ladder_left_enc;
  typedef float _ladder_right_enc_type;
  _ladder_right_enc_type ladder_right_enc;
  typedef uint64_t _front_left_enc_type;
  _front_left_enc_type front_left_enc;
  typedef uint64_t _front_right_enc_type;
  _front_right_enc_type front_right_enc;
  typedef uint64_t _back_left_enc_type;
  _back_left_enc_type back_left_enc;
  typedef uint64_t _back_right_enc_type;
  _back_right_enc_type back_right_enc;

  AllEncoders()
    : bucket_left_enc(0)
    , bucket_right_enc(0)
    , ladder_left_enc(0)
    , ladder_right_enc(0)
    , front_left_enc(0)
    , front_right_enc(0)
    , back_left_enc(0)
    , back_right_enc(0)
  {
  }

  virtual int serialize(unsigned char* outbuffer) const
  {
    int offset = 0;
    union
    {
      float real;
      uint32_t base;
    } u_bucket_left_enc;
    u_bucket_left_enc.real = this->bucket_left_enc;
    *(outbuffer + offset + 0) = (u_bucket_left_enc.base >> (8 * 0)) & 0xFF;
    *(outbuffer + offset + 1) = (u_bucket_left_enc.base >> (8 * 1)) & 0xFF;
    *(outbuffer + offset + 2) = (u_bucket_left_enc.base >> (8 * 2)) & 0xFF;
    *(outbuffer + offset + 3) = (u_bucket_left_enc.base >> (8 * 3)) & 0xFF;
    offset += sizeof(this->bucket_left_enc);
    union
    {
      float real;
      uint32_t base;
    } u_bucket_right_enc;
    u_bucket_right_enc.real = this->bucket_right_enc;
    *(outbuffer + offset + 0) = (u_bucket_right_enc.base >> (8 * 0)) & 0xFF;
    *(outbuffer + offset + 1) = (u_bucket_right_enc.base >> (8 * 1)) & 0xFF;
    *(outbuffer + offset + 2) = (u_bucket_right_enc.base >> (8 * 2)) & 0xFF;
    *(outbuffer + offset + 3) = (u_bucket_right_enc.base >> (8 * 3)) & 0xFF;
    offset += sizeof(this->bucket_right_enc);
    union
    {
      float real;
      uint32_t base;
    } u_ladder_left_enc;
    u_ladder_left_enc.real = this->ladder_left_enc;
    *(outbuffer + offset + 0) = (u_ladder_left_enc.base >> (8 * 0)) & 0xFF;
    *(outbuffer + offset + 1) = (u_ladder_left_enc.base >> (8 * 1)) & 0xFF;
    *(outbuffer + offset + 2) = (u_ladder_left_enc.base >> (8 * 2)) & 0xFF;
    *(outbuffer + offset + 3) = (u_ladder_left_enc.base >> (8 * 3)) & 0xFF;
    offset += sizeof(this->ladder_left_enc);
    union
    {
      float real;
      uint32_t base;
    } u_ladder_right_enc;
    u_ladder_right_enc.real = this->ladder_right_enc;
    *(outbuffer + offset + 0) = (u_ladder_right_enc.base >> (8 * 0)) & 0xFF;
    *(outbuffer + offset + 1) = (u_ladder_right_enc.base >> (8 * 1)) & 0xFF;
    *(outbuffer + offset + 2) = (u_ladder_right_enc.base >> (8 * 2)) & 0xFF;
    *(outbuffer + offset + 3) = (u_ladder_right_enc.base >> (8 * 3)) & 0xFF;
    offset += sizeof(this->ladder_right_enc);
    *(outbuffer + offset + 0) = (this->front_left_enc >> (8 * 0)) & 0xFF;
    *(outbuffer + offset + 1) = (this->front_left_enc >> (8 * 1)) & 0xFF;
    *(outbuffer + offset + 2) = (this->front_left_enc >> (8 * 2)) & 0xFF;
    *(outbuffer + offset + 3) = (this->front_left_enc >> (8 * 3)) & 0xFF;
    *(outbuffer + offset + 4) = (this->front_left_enc >> (8 * 4)) & 0xFF;
    *(outbuffer + offset + 5) = (this->front_left_enc >> (8 * 5)) & 0xFF;
    *(outbuffer + offset + 6) = (this->front_left_enc >> (8 * 6)) & 0xFF;
    *(outbuffer + offset + 7) = (this->front_left_enc >> (8 * 7)) & 0xFF;
    offset += sizeof(this->front_left_enc);
    *(outbuffer + offset + 0) = (this->front_right_enc >> (8 * 0)) & 0xFF;
    *(outbuffer + offset + 1) = (this->front_right_enc >> (8 * 1)) & 0xFF;
    *(outbuffer + offset + 2) = (this->front_right_enc >> (8 * 2)) & 0xFF;
    *(outbuffer + offset + 3) = (this->front_right_enc >> (8 * 3)) & 0xFF;
    *(outbuffer + offset + 4) = (this->front_right_enc >> (8 * 4)) & 0xFF;
    *(outbuffer + offset + 5) = (this->front_right_enc >> (8 * 5)) & 0xFF;
    *(outbuffer + offset + 6) = (this->front_right_enc >> (8 * 6)) & 0xFF;
    *(outbuffer + offset + 7) = (this->front_right_enc >> (8 * 7)) & 0xFF;
    offset += sizeof(this->front_right_enc);
    *(outbuffer + offset + 0) = (this->back_left_enc >> (8 * 0)) & 0xFF;
    *(outbuffer + offset + 1) = (this->back_left_enc >> (8 * 1)) & 0xFF;
    *(outbuffer + offset + 2) = (this->back_left_enc >> (8 * 2)) & 0xFF;
    *(outbuffer + offset + 3) = (this->back_left_enc >> (8 * 3)) & 0xFF;
    *(outbuffer + offset + 4) = (this->back_left_enc >> (8 * 4)) & 0xFF;
    *(outbuffer + offset + 5) = (this->back_left_enc >> (8 * 5)) & 0xFF;
    *(outbuffer + offset + 6) = (this->back_left_enc >> (8 * 6)) & 0xFF;
    *(outbuffer + offset + 7) = (this->back_left_enc >> (8 * 7)) & 0xFF;
    offset += sizeof(this->back_left_enc);
    *(outbuffer + offset + 0) = (this->back_right_enc >> (8 * 0)) & 0xFF;
    *(outbuffer + offset + 1) = (this->back_right_enc >> (8 * 1)) & 0xFF;
    *(outbuffer + offset + 2) = (this->back_right_enc >> (8 * 2)) & 0xFF;
    *(outbuffer + offset + 3) = (this->back_right_enc >> (8 * 3)) & 0xFF;
    *(outbuffer + offset + 4) = (this->back_right_enc >> (8 * 4)) & 0xFF;
    *(outbuffer + offset + 5) = (this->back_right_enc >> (8 * 5)) & 0xFF;
    *(outbuffer + offset + 6) = (this->back_right_enc >> (8 * 6)) & 0xFF;
    *(outbuffer + offset + 7) = (this->back_right_enc >> (8 * 7)) & 0xFF;
    offset += sizeof(this->back_right_enc);
    return offset;
  }

  virtual int deserialize(unsigned char* inbuffer)
  {
    int offset = 0;
    union
    {
      float real;
      uint32_t base;
    } u_bucket_left_enc;
    u_bucket_left_enc.base = 0;
    u_bucket_left_enc.base |= ((uint32_t)(*(inbuffer + offset + 0))) << (8 * 0);
    u_bucket_left_enc.base |= ((uint32_t)(*(inbuffer + offset + 1))) << (8 * 1);
    u_bucket_left_enc.base |= ((uint32_t)(*(inbuffer + offset + 2))) << (8 * 2);
    u_bucket_left_enc.base |= ((uint32_t)(*(inbuffer + offset + 3))) << (8 * 3);
    this->bucket_left_enc = u_bucket_left_enc.real;
    offset += sizeof(this->bucket_left_enc);
    union
    {
      float real;
      uint32_t base;
    } u_bucket_right_enc;
    u_bucket_right_enc.base = 0;
    u_bucket_right_enc.base |= ((uint32_t)(*(inbuffer + offset + 0))) << (8 * 0);
    u_bucket_right_enc.base |= ((uint32_t)(*(inbuffer + offset + 1))) << (8 * 1);
    u_bucket_right_enc.base |= ((uint32_t)(*(inbuffer + offset + 2))) << (8 * 2);
    u_bucket_right_enc.base |= ((uint32_t)(*(inbuffer + offset + 3))) << (8 * 3);
    this->bucket_right_enc = u_bucket_right_enc.real;
    offset += sizeof(this->bucket_right_enc);
    union
    {
      float real;
      uint32_t base;
    } u_ladder_left_enc;
    u_ladder_left_enc.base = 0;
    u_ladder_left_enc.base |= ((uint32_t)(*(inbuffer + offset + 0))) << (8 * 0);
    u_ladder_left_enc.base |= ((uint32_t)(*(inbuffer + offset + 1))) << (8 * 1);
    u_ladder_left_enc.base |= ((uint32_t)(*(inbuffer + offset + 2))) << (8 * 2);
    u_ladder_left_enc.base |= ((uint32_t)(*(inbuffer + offset + 3))) << (8 * 3);
    this->ladder_left_enc = u_ladder_left_enc.real;
    offset += sizeof(this->ladder_left_enc);
    union
    {
      float real;
      uint32_t base;
    } u_ladder_right_enc;
    u_ladder_right_enc.base = 0;
    u_ladder_right_enc.base |= ((uint32_t)(*(inbuffer + offset + 0))) << (8 * 0);
    u_ladder_right_enc.base |= ((uint32_t)(*(inbuffer + offset + 1))) << (8 * 1);
    u_ladder_right_enc.base |= ((uint32_t)(*(inbuffer + offset + 2))) << (8 * 2);
    u_ladder_right_enc.base |= ((uint32_t)(*(inbuffer + offset + 3))) << (8 * 3);
    this->ladder_right_enc = u_ladder_right_enc.real;
    offset += sizeof(this->ladder_right_enc);
    this->front_left_enc = ((uint64_t)(*(inbuffer + offset)));
    this->front_left_enc |= ((uint64_t)(*(inbuffer + offset + 1))) << (8 * 1);
    this->front_left_enc |= ((uint64_t)(*(inbuffer + offset + 2))) << (8 * 2);
    this->front_left_enc |= ((uint64_t)(*(inbuffer + offset + 3))) << (8 * 3);
    this->front_left_enc |= ((uint64_t)(*(inbuffer + offset + 4))) << (8 * 4);
    this->front_left_enc |= ((uint64_t)(*(inbuffer + offset + 5))) << (8 * 5);
    this->front_left_enc |= ((uint64_t)(*(inbuffer + offset + 6))) << (8 * 6);
    this->front_left_enc |= ((uint64_t)(*(inbuffer + offset + 7))) << (8 * 7);
    offset += sizeof(this->front_left_enc);
    this->front_right_enc = ((uint64_t)(*(inbuffer + offset)));
    this->front_right_enc |= ((uint64_t)(*(inbuffer + offset + 1))) << (8 * 1);
    this->front_right_enc |= ((uint64_t)(*(inbuffer + offset + 2))) << (8 * 2);
    this->front_right_enc |= ((uint64_t)(*(inbuffer + offset + 3))) << (8 * 3);
    this->front_right_enc |= ((uint64_t)(*(inbuffer + offset + 4))) << (8 * 4);
    this->front_right_enc |= ((uint64_t)(*(inbuffer + offset + 5))) << (8 * 5);
    this->front_right_enc |= ((uint64_t)(*(inbuffer + offset + 6))) << (8 * 6);
    this->front_right_enc |= ((uint64_t)(*(inbuffer + offset + 7))) << (8 * 7);
    offset += sizeof(this->front_right_enc);
    this->back_left_enc = ((uint64_t)(*(inbuffer + offset)));
    this->back_left_enc |= ((uint64_t)(*(inbuffer + offset + 1))) << (8 * 1);
    this->back_left_enc |= ((uint64_t)(*(inbuffer + offset + 2))) << (8 * 2);
    this->back_left_enc |= ((uint64_t)(*(inbuffer + offset + 3))) << (8 * 3);
    this->back_left_enc |= ((uint64_t)(*(inbuffer + offset + 4))) << (8 * 4);
    this->back_left_enc |= ((uint64_t)(*(inbuffer + offset + 5))) << (8 * 5);
    this->back_left_enc |= ((uint64_t)(*(inbuffer + offset + 6))) << (8 * 6);
    this->back_left_enc |= ((uint64_t)(*(inbuffer + offset + 7))) << (8 * 7);
    offset += sizeof(this->back_left_enc);
    this->back_right_enc = ((uint64_t)(*(inbuffer + offset)));
    this->back_right_enc |= ((uint64_t)(*(inbuffer + offset + 1))) << (8 * 1);
    this->back_right_enc |= ((uint64_t)(*(inbuffer + offset + 2))) << (8 * 2);
    this->back_right_enc |= ((uint64_t)(*(inbuffer + offset + 3))) << (8 * 3);
    this->back_right_enc |= ((uint64_t)(*(inbuffer + offset + 4))) << (8 * 4);
    this->back_right_enc |= ((uint64_t)(*(inbuffer + offset + 5))) << (8 * 5);
    this->back_right_enc |= ((uint64_t)(*(inbuffer + offset + 6))) << (8 * 6);
    this->back_right_enc |= ((uint64_t)(*(inbuffer + offset + 7))) << (8 * 7);
    offset += sizeof(this->back_right_enc);
    return offset;
  }

  const char* getType()
  {
    return "lk_rover/AllEncoders";
  };
  const char* getMD5()
  {
    return "8fa32fb42daddba56b0f01a48f605cb7";
  };
};
}
#endif