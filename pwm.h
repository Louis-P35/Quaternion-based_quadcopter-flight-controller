/*
This code configure a 500Hz PWM signal on pins 11, 12, 44 & 45
with a resolution of 32000 distinct levels.
Using the timer 1 and timer 5
*/


void setupPWM();
void setupPwmTimer1();
void setupPwmTimer5();
void setPwmPin11(unsigned int Ton);
void setPwmPin12(unsigned int Ton);
void setPwmPin44(unsigned int Ton);
void setPwmPin45(unsigned int Ton);