#ifndef scale_H
#define scale_H

#include "../header.h"

class scale
{
public:
    scale();
    void initialize(int);
    void update_parameters(float *val);
    void get_parameters(float *mean_val,float *var_val);
    void clear();


private:
    struct table{
        std::vector<float> value;
    };
    table *data;
    int length,count;
    double pre_mean[20],pre_var[20];



};

#endif // scale_H
