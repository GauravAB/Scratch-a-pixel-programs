#define _USE_MATH_DEFINES

 #include <cstdlib>
 #include <cstdio>
 #include <cmath>
 #include <algorithm>
 #include <fstream>
#include <cstring>


//sampling H-G scattering phase function
 double getCosTheta(const double& g)
 {
    //isotropic
    if(g == 0) return 2 * drand48() - 1;

    double mu = ( 1- g * g) / (1 - g +  2 * g * drand48());
    return (1 + g * g - mu * mu) / (2.0 * g); //return cos(theta)
 }

 //compute new photon direction due to scattering event
 void spin(double& mu_x , double& mu_y, double& mu_z , const double &g)
 {
    double costheta = getCosTheta(g);
    double phi = 2 * M_PI * drand48();
    double sintheta = sqrt(1.0 - costheta * costheta) ;
    double sinphi = sin(phi);
    double cosphi = cos(phi);

    if(mu_z == 1.0)
    {
        mu_x = sintheta * cosphi;
        mu_y = -sintheta * sinphi;
        mu_z = costheta;       
    }
    else if(mu_z == -1.0)
    {
        mu_x = sintheta * cosphi;
        mu_y = -sintheta * sinphi;
        mu_z = -costheta;
    }
    else{
        double denom = sqrt(1.0 - mu_z * mu_z);
        double muzcosphi = mu_z * cosphi;
        double ux = sintheta * (mu_x * muzcosphi - mu_y * sinphi) / denom + mu_x * costheta;
        double uy = sintheta * (mu_y * muzcosphi - mu_x * sinphi) / denom + mu_y * costheta;
        double uz = -denom * sintheta * cosphi + mu_z * costheta;
        mu_x  = ux, mu_y = uy , mu_z = uz;
    }
 }

 void MCSimulation (double *&records , const uint32_t& size)
 {
     uint32_t nphotons = 100000;
     double scale = 1.0 / nphotons;
     double sigma_a = 1 , sigma_s = 2, sigma_t = sigma_a + sigma_s;
     double d = 0.5, slabsize = 0.5, g = 0.78;
     static const short m = 10;
     double Rd = 0, Tt = 0;

     //launch nphoton packets into the slab make them travel randomly and store the final results
     for(int n = 0; n < nphotons; ++n)
     {
         //some weight to packet of photons travelling together
        double w = 1;
        
        //mux muy muz is the direction vector of the photon
        //x y z current position of photon
        double x = 0, y = 0, z = 0, mux = 0, muy = 0, muz = 1;
        
        //photon gets absorbed, next pass
        while(w != 0)
        {
            //inverse cdf sample from a uniform distribution input
            double s = -log(drand48()) / sigma_t;
            double distToBoundary = 0;
            //calculate distance inside slab
            if(muz > 0) distToBoundary = (d - z) / muz;
            else if( muz < 0) distToBoundary = -z / muz;

            //sample out of slab , (Transmitted photon packet added to the 2D grid value)
            if(s > distToBoundary)
            {
#ifdef ONED
                if(muz > 0) Tt += w; else Rd += w;
#else
                //map to 2D image size
                int xi = (int)((x + slabsize / 2) / slabsize * size);
                int yi = (int)((y + slabsize / 2) / slabsize * size);
                if(muz > 0 && xi >= 0 && x < size && yi >= 0 && yi < size)
                {
                    records[yi * size + xi] += w;
                }
#endif
                break;
            }

            //new position inside slab
            //update direction vector and add to current pos
            x += s * mux;
            y += s * muy;
            z += s * muz;

            //probability that photon gets absorbed
            double dw = sigma_a / sigma_t;
            //portion of photon packet left
            w -= dw; 
            //clamp
            w = std::max(0.0,w);

            //packet is reduced to nothing by absorbtion inside the atom
            if(w < 0.001) {  
                //we make sure our algorithm stays unbiased by using russian roulette to kill the photons
                //russian roulette test
                //stochastically decide to kill the packet
                if(drand48() > 1.0 / m) break;
                //increase their life
                //adjust weight
                else w *= m;
            }

            //diffuse scatter using H-G scattering phase function
            //in abstract way if the photon is inside slab and is not absorbed it has to scatter
            spin(mux , muy , muz , g);
        }
     }

#ifdef ONED
     printf("Rd %f Tt %f\n", Rd * scale, Tt * scale);
#endif
}

int main(int argc , char** argv)
{
    double *records = NULL;
    const uint32_t size = 512;
  
    //2D grid representing the bottom of the surface of the slab
    records = new double[size * size ];
    memset(records , 0x0, sizeof(double) * size * size );
    uint32_t npasses = 1;

    //image
    float *pixels = new float[size * size];
    
    while(npasses < 256)
    {
        //simulate path of n photon packets
        MCSimulation(records, size);
        for(int i = 0; i < size * size ; ++i) pixels[i] = records[i] / npasses;
        //display(pixels) 
        npasses++;
        printf("num passes: %d \n", npasses);
    }


    //save image to file
    std::ofstream ofs;
    ofs.open("out256_g0.ppm",std::ios::out | std::ios::binary);
    ofs << "P6\n" << size << " " << size << "\n255\n";

    for(uint32_t i = 0; i < size * size; ++i)
    {
        unsigned char val = (unsigned char)(255 * std::min(1.0f, pixels[i]));
        ofs << val << val << val;
    }

    ofs.close();

    delete [] records;
    delete [] pixels;

    return 0;
}










