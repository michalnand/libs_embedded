#ifndef _PID_H_
#define _PID_H_


class PID
{
    public:
        PID();
        virtual ~PID();

        void init(float Kp, float Ki, float Kd, float out_range_min = -1.0, float out_range_max = 1.0, float dt = 0.005);
        void reset(float output_initial = 0.0);

        float step(float x_setpoint, float x_measured);

    private:
        float k0, k1, k0d, k1d, k2d;
        float out_range_min, out_range_max;
        float alpha;

        float e0, e1, e2, d0, d1;
        float y, y_der;
};
 
#endif