#include "scale.h"
#include "../header.h"

 scale::scale()
{

    count=0;
    for(int i=0;i<20;i++)
         pre_mean[i]=pre_var[i]=0;

}


 void scale::initialize(int item)
 {
  data=new table[item];
  length=item;

 }

 void scale::update_parameters(float *val)
 {

     for(int i=0;i<length;i++)
     {data[i].value.push_back(val[i]);
     }
     count++;

 }

 void scale::get_parameters(float *mean_val,float *var_val)
 {
    if(count>0)
    {
         for(int i=0;i<length;i++){
             mean_val[i]=pre_mean[i]=mean(data[i].value);
             var_val[i]=pre_var[i]=var(mean_val[i],data[i].value);

         }
         clear();
    }
    else
    {
        for(int i=0;i<length;i++){
            mean_val[i]=pre_mean[i];
            var_val[i]=pre_var[i];
        }
    }

 }

 void scale::clear(){
        for(int i=0;i<length;i++)
            data[i].value.clear();
        count=0;
 }

