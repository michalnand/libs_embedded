#ifndef _KALMAN_H_
#define _KALMAN_H_


class KalmanFilter
{
    public:
        KalmanFilter();
        virtual ~KalmanFilter();


        /*
            input : 
                z   : position measurement
                dz  : velocity measurement
                pz  : position measurement uncertaininty
                pdz : velocity measurement uncertaininty
                dt  : time between two measurements [ms]

            returns:
                z^  : filtered z
        */
        float step(float z, float dz, float pz, float pdz, float dt);

        float get_x();
        float get_v();

    private:
        float x_hat, p;

        float x_hat_prev, v_hat;
};

#endif