// g++ test.cpp -Wall -o test -lwiringPi

#include <wiringPi.h>
#include <stdio.h>
#include <cstdlib>
#include <iostream>

#define MaxPWM 256

using namespace std;

const int PWM_pin = 1;   /* GPIO 1 as per WiringPi, GPIO18 as per BCM */

int main ()
{
  int intensity ;

  if (wiringPiSetup() == -1)
  {
   cout<<"Hello"<<endl;
    exit (1) ;
	}


  pinMode (PWM_pin, PWM_OUTPUT) ; /* set PWM pin as output */

  pwmSetRange(MaxPWM);
  pwmSetClock(4);

  while (1)
  {
   for (intensity = 0; intensity < MaxPWM; intensity++){
       pwmWrite(PWM_pin, intensity);
       delay(10);
       cout<<intensity<<endl;
   }
   for (intensity = MaxPWM; intensity > 0; intensity --){
       pwmWrite(PWM_pin, intensity);
       delay(10);
       cout<<intensity<<endl;
   }


    //pwmWrite (PWM_pin, 128) ;    /* provide PWM value for duty cycle */
}
}
