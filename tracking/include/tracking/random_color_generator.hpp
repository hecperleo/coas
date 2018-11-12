#ifndef RNDCOLOR_H_
#define RNDCOLOR_H_

//#define DEBUG_MODE

#include <std_msgs/ColorRGBA.h>
#include <random>

#ifdef DEBUG_MODE
#include <iostream>
#endif

/**
 * class RandomColorGenerator
 *
 * A Random Color Generator based on the equations found in the next webpage
 *
 * https://www.rapidtables.com/convert/color/hsv-to-rgb.html
 *
 * @author Alejandro Braza Barba
 */

class RandomColorGenerator
{
public:
    /**
     * Create a new Random Color Generator
     */
    RandomColorGenerator()
    {
        typedef std::chrono::high_resolution_clock myclock;
        unsigned seed1 = myclock::now().time_since_epoch().count();
        generator1_.seed(seed1);
        unsigned seed2 = myclock::now().time_since_epoch().count();
        generator2_.seed(seed2);
        unsigned seed3 = myclock::now().time_since_epoch().count();  
        generator3_.seed(seed3);         
    }

    std_msgs::ColorRGBA generateColor()
    {
        std_msgs::ColorRGBA color_rgba;
        
        std::uniform_int_distribution<int> distribution1(0,360);
        int H = distribution1(generator1_); // 0-360

        std::uniform_real_distribution<float> distribution2(0.5,1);
        float S = distribution2(generator2_); // 0-1

        std::uniform_real_distribution<float> distribution3(0.5,1);
        float V = distribution3(generator3_); // 0-1

        #ifdef DEBUG_MODE
        std::cout << "H: " << H << std::endl;
        std::cout << "S: " << S << std::endl;
        std::cout << "V: " << V << std::endl;
        #endif

        float C = V * S; // V x S
        float X = C * (1 - abs( (H/60)%2 - 1 ) ); // C x (1 - abs( (H/60) mod 2 - 1) )
        float m = V - C;

        #ifdef DEBUG_MODE
        std::cout << "C " << C << std::endl;
        std::cout << "X " << X << std::endl;
        std::cout << "m " << m << std::endl;
        #endif

        float aux_R;
        float aux_G;
        float aux_B;

        if( (H >= 0) && (H < 60) )
        {
            aux_R = C;
            aux_G = X;
            aux_B = 0;
        }
        else if( (H >= 60) && (H < 120) )
        {
            aux_R = X;
            aux_G = C;
            aux_B = 0;
        }
        else if( (H >= 120) && (H < 180) )
        {
            aux_R = 0;
            aux_G = C;
            aux_B = X;
        }
        else if( (H >= 180) && (H < 240) )
        {
            aux_R = 0;
            aux_G = X;
            aux_B = C;
        }
        else if( (H >= 240) && (H < 300) )
        {
            aux_R = X;
            aux_G = 0;
            aux_B = C;
        }
        else if( (H >= 300) && (H < 360) )
        {
            aux_R = C;
            aux_G = 0;
            aux_B = X;
        }

        color_rgba.r = aux_R + m;
        color_rgba.g = aux_G + m;
        color_rgba.b = aux_B + m;
        color_rgba.a = 1;

        #ifdef DEBUG_MODE
        std::cout << "Color generado :" << std::endl;
        std::cout << "R " << color_rgba.r << std::endl;
        std::cout << "G " << color_rgba.g << std::endl;
        std::cout << "B " << color_rgba.b << std::endl;
        #endif

        return color_rgba;
    }
private:
    std::default_random_engine generator1_;
    std::default_random_engine generator2_;
    std::default_random_engine generator3_;
};

#endif