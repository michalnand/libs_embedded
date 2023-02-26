#ifndef _BIQUAD_H_
#define _BIQUAD_H_

template<class DType, int32_t denominator = 1024>
class Biquad
{
    public:
        Biquad() 
        {
            this->init(0, 0, 0, 0, 0);
        }

        Biquad(DType b0, DType b1, DType b2, DType a1, DType a2)
        {
            this->init(b0, b1, b2, a1, a2);
        }

    public:
        void init(DType b0, DType b1, DType b2, DType a1, DType a2)
        {
            this->b0 = b0;
            this->b1 = b1;
            this->b2 = b2;

            this->a1 = a1;
            this->a2 = a2;

            this->y1 = 0; 
            this->y2 = 0; 
            this->x1 = 0;
            this->x2 = 0;
        }

        DType step(DType x)
        {
            DType y = b0*x + b1*x1 + b2*x2 - a1*y1 - a2*y2;
            y = y/denominator;

            x2 = x1;
            x1 = x;
            y2 = y1;
            y1 = y;

            return y;
        }

    private:
        DType b0, b1, b2;
        DType a1, a2;

        DType y1, y2, x1, x2;
};

#endif
