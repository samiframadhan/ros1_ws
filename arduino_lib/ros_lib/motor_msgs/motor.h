#ifndef _ROS_motor_msgs_motor_h
#define _ROS_motor_msgs_motor_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "motor_msgs/PID.h"

namespace motor_msgs
{

  class motor : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef float _speed_type;
      _speed_type speed;
      typedef float _pwm_type;
      _pwm_type pwm;
      typedef motor_msgs::PID _pid_data_type;
      _pid_data_type pid_data;

    motor():
      header(),
      speed(0),
      pwm(0),
      pid_data()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_speed;
      u_speed.real = this->speed;
      *(outbuffer + offset + 0) = (u_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->speed);
      union {
        float real;
        uint32_t base;
      } u_pwm;
      u_pwm.real = this->pwm;
      *(outbuffer + offset + 0) = (u_pwm.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pwm.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pwm.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pwm.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pwm);
      offset += this->pid_data.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_speed;
      u_speed.base = 0;
      u_speed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->speed = u_speed.real;
      offset += sizeof(this->speed);
      union {
        float real;
        uint32_t base;
      } u_pwm;
      u_pwm.base = 0;
      u_pwm.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pwm.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pwm.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pwm.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pwm = u_pwm.real;
      offset += sizeof(this->pwm);
      offset += this->pid_data.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "motor_msgs/motor"; };
    virtual const char * getMD5() override { return "448a7a4c5a90f6dfc0d67c7324278021"; };

  };

}
#endif
