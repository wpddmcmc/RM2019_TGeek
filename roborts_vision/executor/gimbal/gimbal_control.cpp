#include <cmath>
#include <stdio.h>

#include "gimbal_control.h"


//air friction is considered
float GimbalContrl::BulletModel(float z, float v, float angle) { //x:m,v:m/s,angle:rad
  float t, y;
  t = (float)((exp(init_k_ * z) - 1) / (init_k_ * v * cos(angle)));
  y = (float)(v * sin(angle) * t - GRAVITY * t * t / 2);
  return y;
}

//z:distance , y: height
float GimbalContrl::GetPitch(float z, float y, float v) {
  float y_temp, y_actual, dy;
  float a;
  y_temp = y;
  // by iteration
  for (int i = 0; i < 20; i++) {
    a = (float) atan2(y_temp, z);
    y_actual = BulletModel(z, v, a);
    dy = y - y_actual;
    y_temp = y_temp + dy;
    if (fabsf(dy) < 0.001) {
      break;
    }
    /*printf("iteration num %d :\n 
            angle %f,
            temp target y:%f,
            err of y:%f \n", 
            i+1, 
            a*180/3.1415926535,
            y_temp,
            dy);*/
  }
  return a;

}

void GimbalContrl::SolveContrlAgnle(cv::Point3f &postion, float &yaw, float &pitch) 
{
  yaw = (float)(atan2(postion.x + offset_.x, postion.z + offset_.z)) - (float)(offset_yaw_ * M_PI / 180.0); 

  yaw = yaw * 180.0 / M_PI;

  pitch = (float)(offset_pitch_ * M_PI / 180.0)
        - GetPitch( (offset_.z + postion.z) / 1000.0, 
                    (offset_.y - postion.y) / 1000.0, init_v_); 


  pitch = pitch * 180.0 / M_PI;
}



