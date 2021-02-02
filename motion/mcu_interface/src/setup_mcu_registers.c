#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>


/* Counters. Write the thrust values to these registers                   */
#define THR0 = OCR3A              /* Counter THR0                         */
#define THR1 = OCR3B              /* Counter THR1                         */ 
#define THR2 = OCR3C              /* Counter THR2                         */ 
#define THR3 = OCR4A              /* Counter THR3                         */
#define THR4 = OCR4B              /* Counter THR4                         */
#define THR5 = OCR4C              /* Counter THR5                         */
#define THR6 = OCR5A              /* Counter THR6                         */
#define THR7 = OCR5B              /* Counter THR7                         */


/* Thrust-values                                                          */
#define THRUST_FULL_REVERSE  140  /* Value that gives max reverse thrust  */
#define THRUST_IDLE          191  /* Value that gives idle thrust         */
#define THRUST_FULL_FORWARD  242  /* Value that gives max forward thrust  */


/* Limits on 16-bits thrust-values                                        */
#define MIN_THRUST_16       1100  /* Min thrust the thrusters handles     */
#define MAX_THRUST_16       1900  /* Max thrust the thrusters handles     */



/**
 * @brief This function does a linear transformation of a 16 bit control signal
 * in the range [1100, 1900] to a 8 bit control signal in the range [127, 255] 
 * 
 * 
 * From linear calculations: 
 *      thruster_control_signal = 1000 => pwm_signal = 127
 *      thruster_control_signal = 2000 => pwm_signal = 255
 * 
 * we get the function f: R -> R given by
 * pwm_signal := f(thruster_control_signal) = 0.128 * thruster_control_signal - 1  
 * 
 * The calculated pwm_signal - value is casted to uint8_t as we desire values between
 * 0x7F and 0xFF
 *
 * @warning If the given value is outside the specified range, the thrust is set to idle
 * 
 * @warning This linear transformation assumes that the clock-scaler is 
 * set to 1/64
 * 
 * @warning May be a redundant code, due to using a 8 bit MCU. The thruster-commands
 * should therefore be specified to be within 0x00 and 0xFF
 */
uint8_t calculate_pwm_counter(const uint16_t& thruster_control_signal){
  /**
   * Checking if the signal is valid
   * 
   * If not valid (outside the range), the thruster is set to idle thrust
   */
  if(thruster_control_signal <= MIN_THRUST_16 || thruster_control_signal >= MAX_THRUST_16)
    return (uint8_t) (THRUST_IDLE);

  return (uint8_t) (0.128 * thruster_control_signal - 1);
}



/**
 * @brief Function to initialize the thrusters to IDLE
 * 
 * @warning Must be enabled after the pwm
 * 
 * @warning The function assumes that the LED is initialized
 * 
 * @warning Must be set in at least a couple of seconds
 */
void initialize_thrusters(){
  /* Turning LED D3 high and LED D2 low */
  PORTF = 0xAA;

  /* Init thrusters to idle */
  THR0 = THRUST_IDLE;
  THR1 = THRUST_IDLE;
  THR2 = THRUST_IDLE;
  THR3 = THRUST_IDLE;
  THR4 = THRUST_IDLE;
  THR5 = THRUST_IDLE;
  THR6 = THRUST_IDLE;
  THR7 = THRUST_IDLE;

  /* Giving the thrusters 3 seconds to init */
  _delay_ms(3000);

  /* Turning LED D2 high and LED D3 low */
  PORTF = 0x55;

  /* Waiting for another second */
  _delay_ms(1000);
}


/**
 * @brief This function initializes the pwm
 * 
 * @warning Must be enabled before the thrusters
 */
void initialize_pwm(){
  /* Clear ports */
  DDRE = 0;
  DDRH = 0;
  DDRL = 0;


  /* Setting ports as output */
  DDRE |= (1 << PE3) | (1 << PE4) | (1 << PE5); 
  DDRH |= (1 << PH1) | (1 << PH3) | (1 << PH4) | (1 << PH5);
  DDRL |= (1 << PL3) | (1 << PL4);
  
  
  /* Initializing ports as pull-down */
  PORTE &= !((1 << PE3) | (1 << PE4) | (1 << PE5));  
  PORTH &= !((1 << PH2) | (1 << PH1) | (1 << PH0));
  PORTL &= !((1 << PL3) | (1 << PL4));

  
  /* Setting A to Fast PWM, clearing OCnA and OCnB on compare match */
  /* | COMnA1 | COMnA0 | COMnB1 | COMnB0 | WGMn3 | WMGn2 | WMGn1 | WMGn0 | = | 1 | 0 | 1 | 0 | 0 | 1 | 1 | 0 | */
  TCCR3A = (1 << COM3A1) | (1 << COM3B1) | (1 << COM3C1) | (1 << WGM32) | (1 << WGM31);
  TCCR4A = (1 << COM4A1) | (1 << COM4B1) | (1 << COM4C1) | (1 << WGM42) | (1 << WGM41);
  TCCR5A = (1 << COM5A1) | (1 << COM5B1) | (1 << COM5C1) | (1 << WGM52) | (1 << WGM51);
  
  
  /* Setting clock-source to 1/64 * clk. See table 17-6 */
  TCCR3B = (1 << CS31) | (1 << CS30);
  TCCR4B = (1 << CS41) | (1 << CS40);
  TCCR5B = (1 << CS51) | (1 << CS50);
  
  
  /* Init timers */
  TCNT3 = 0;
  TCNT4 = 0;
  TCNT5 = 0;
}


/**
 * @brief This function initializes SPI
 */
void initialize_spi(){
  /* Clear port */
  DDRB = 0;

  /* Set MISO as output, all others input */
  DDRB |= (1 << MISO);
  
  /* Enable SPI as slave */
  SPCR = (1 << SPE);
  SPCR &= !(1 << MSTR);
}


/**
 * @brief Initialize LEDs D2 and D3
 */
void initialize_leds(){
  /* Clear port */
  DDRF = 0;

  /* Enable LEDs */
  DDRF |= (1 << PF0) | (1 << PF1);
}


/**
 * @brief Standard setup-wrapper for initializing code
 */
void setup() {  
  /* Init LED */
  initialize_leds();
  
  /* Init SPI */
  initialize_spi();

  /* Init PWM */
  initialize_pwm();

  /* Enabling global interrupt */
  SREG = (1 << 7);

  /* Init thrusters */
  initialize_thrusters();
}

// /**
//  * @brief Small test-script to test THR7 
//  */
// void test_THR7(){
//   /* Setting LED D3 high and LED D2 low */
//   PORTF = 0xAA;
//   OCR5B = 191;
//   _delay_ms(10000);
  

//   /* Spinning from idle to full reverse thrust */
//   for(int thrust_count = 191; thrust_count > 126; thrust_count--){
//     OCR5B = thrust_count;
//     _delay_ms(50);
//   }

//   /* Waiting 1 second */
//   _delay_ms(1000);

//   /* Spinning from full reverse thrust to idle */
//   for(int thrust_count = 127; thrust_count < 192; thrust_count++){
//     OCR5B = thrust_count;
//     _delay_ms(50);
//   }
  
//   /* Waiting for 2 seconds */
//   _delay_ms(2000);

//   /* Spinning from idle to full forward thrust */
//   for(int thrust_count = 191; thrust_count < 256; thrust_count++){
//     OCR5B = thrust_count;
//     _delay_ms(50);
//   }

//   /* Spinning from full forward thrust to idle */
//   for(int thrust_count = 255; thrust_count > 191; thrust_count--){
//     OCR5B = thrust_count;
//     _delay_ms(50);
//   }

//   /* Setting LED D2 high and LED D3 low */
//   PORTF = 0x55;
//   _delay_ms(100000);

//   return;
// }

void loop() {
  /* Do something */
}