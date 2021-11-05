#include <cstdlib>
#include <cstdio>
#include <random>

int main(int argc , char** argv)
{
    int N = (argc == 2) ? atoi(argv[1]) : 1024 , hits = 0;

   std::default_random_engine generator;
   std::uniform_real_distribution<float> distribution(0.0,1.0);

    for(int i = 0; i < N ; ++i)
    {
        
       // float x = distribution(generator);
       // float y = distribution(generator);
       float x = drand48();
       float y = drand48();
       
        float l = sqrt(x *x + y * y);

        if(l <= 1) hits++;
    }

    fprintf(stderr , "Area of unit disk: %f (%d)\n", float(hits)/ N * 4, hits);


    return 0;
}
