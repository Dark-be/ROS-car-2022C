
class PID{
public:
    float p;
    float i;
    float d;
    float target;
    float error;
    float last_error;
    float error_sum;
    float max_sum;
    float max;
    PID(float _p,float _i,float _d,float _max_sum,float _max);
    float GetRes(float input,float target);
};