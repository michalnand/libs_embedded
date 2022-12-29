#ifndef LQC_H
#define LQC_H

template <unsigned int required_max_dim = 8, unsigned int output_max_dim = 8, unsigned int state_max_dim = 8, class DType = float> 
class LQC
{
    public:
        LQC()   
        {
            unsigned int count = output_max_dim*(required_max_dim + state_max_dim);
 
            for (unsigned int i = 0; i < count; i++)
            {
                control_mat[i] = 0;
            }

            for (unsigned int i = 0; i < (required_max_dim + state_max_dim); i++)
            {
                input_mat[i] = 0;
            }

            for (unsigned int i = 0; i < output_max_dim; i++)
            {
                output_mat[i] = 0;
            }
        }

        void set_cm(unsigned int row, unsigned int col, DType value)
        {
            auto cols = required_max_dim + state_max_dim;
            control_mat[row*cols + col] = value;
        }

        void set_required(unsigned int idx, DType value)
        {
            input_mat[idx] = value;
        }

        void set_state(unsigned int idx, DType value)
        {
            input_mat[idx + required_max_dim] = value;
        } 
        
        DType get_output(unsigned int idx)
        {
            return output_mat[idx];
        }

        void step()
        {
            auto rows = output_max_dim;
            auto cols = required_max_dim + state_max_dim;

            for (unsigned int j = 0; j < rows; j++)
            {
                DType sum = 0;

                for (unsigned int i = 0; i < cols; i++)
                {
                    sum+= control_mat[j*cols + i]*input_mat[i];
                }

                output_mat[j] = sum;
            } 
        } 

    private:
        DType control_mat[output_max_dim*(required_max_dim + state_max_dim)];
        DType input_mat[required_max_dim + state_max_dim];

        DType output_mat[output_max_dim];
};

#endif