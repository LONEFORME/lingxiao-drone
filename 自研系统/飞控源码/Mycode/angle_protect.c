#include "angle_protect.h"

#define ROLL_MAX  30
#define PITCH_MAX 30

u8 Attitude_Check(void)
{
    s16 roll  = fc_att.st_data.rol_x100;
    s16 pitch = fc_att.st_data.pit_x100;

    if(roll < 0) roll = -roll;
    if(pitch < 0) pitch = -pitch;

    if(roll > ROLL_MAX*100 || pitch > PITCH_MAX*100)
    {
        return 1;
    }
    return 0;
}
