#include <lqrctrl.h>
#include <userdefine.h>


st_lqr stg_lqr;

void LQRcontrol(float pitch);

void LQRcontrol(float pitch)
{

    st_lqr *stp_lqr = &stg_lqr;
    
    stp_lqr->refTorq = -LQRGAIN * pitch;

}