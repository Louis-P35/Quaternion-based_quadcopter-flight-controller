

void setup_PWM_reader(uint8_t channel0, uint8_t channel1, uint8_t channel2, uint8_t channel3);

/* functions to read PWM signal, called at any state change (high to low or low to high) */
void handle_PWM_read(int pin, int channel);

/* ISR function */
void readPWM0();
void readPWM1();
void readPWM2();
void readPWM3();

/* Interrupt safe getter */
unsigned long getRadioChannel(int channel);