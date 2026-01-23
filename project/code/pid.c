#include "mm32_device.h"                // Device header
#include "PID.h"

void PID_Update(PID_t *p)
{
	
	if (p == NULL)
    {
        return;
    }
	
	p->Error1 = p->Error0;
	p->Error0 = p->Target - p->Actual;
	
	if (p->Ki != 0)
	{
		p->ErrorInt += p->Error0;
		if (p->ErrorInt > p->OutMax/p->Ki) p->ErrorInt = p->OutMax/p->Ki;
        if (p->ErrorInt < p->OutMin/p->Ki) p->ErrorInt = p->OutMin/p->Ki;
	}
	else
	{
		p->ErrorInt = 0;
	}
	
	p->Out = p->Kp * p->Error0
		   + p->Ki * p->ErrorInt
		   + p->Kd * (p->Error0 - p->Error1);
	
	if (p->Out > p->OutMax) {p->Out = p->OutMax;}
	if (p->Out < p->OutMin) {p->Out = p->OutMin;}
}
