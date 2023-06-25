#include "my_pid.h"
PID::PID(float _p,float _i,float _d,float _max_sum,float _max){
    p=_p;
    i=_i;
    d=_d;
    last_error=0;
    max=_max;
    max_sum=_max_sum;
}
float PID::GetRes(float input,float target){
    error=target-input;
    error_sum+=error;
    if(error_sum>max_sum)
        error_sum=max_sum;
    else if(error_sum<-max_sum)
        error_sum=-max_sum;
    float res=p*error+i*max_sum+d*(error-last_error);
    if(res>max)
        res=max;
    else if(res<-max)
        res=-max;
    last_error=error;
    return res;
}