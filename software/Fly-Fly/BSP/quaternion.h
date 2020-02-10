#ifndef _QUATERNION_H_
#define _QUATERNION_H_

#include "board_define.h"

#ifdef __cplusplus
extern "C"{
#endif

/**********************************************************************************************************/

void sensfusion6UpdateQ(float gx, float gy, float gz, float ax, float ay, float az, float dt);
void sensfusion6GetEulerRPY(float* roll, float* pitch,float* yaw);
	



/**********************************************************************************************************/
  
#ifdef __cplusplus
} // extern "C"
#endif

#endif



