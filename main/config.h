#ifndef CONFIG_H
#define CONFIG_H

#define DEBUGGING 1

/**** Analog Pins ****/
#define ANALOGpin_pot 14
#define ANALOGpin_current 15
#define ANALOGpin_voltage 16

/**** Digital Pins ****/
/*PLC Signals*/
#define IOpin_enable 2
#define IOpin_start 3
#define IOpin_preheat 4
#define IOpin_sealing 5
#define IOpin_reset 6
#define IOpin_alarm 7
/*Control Pins*/
#define CTRLpin_PWM 8
#define CTRLpin_OnOff 9

/*General defs*/
#define LOW 0
#define HIGH 1

/*Serial*/
#define UART_BAUDRATE 9600
#define DISPLAY_BAUDRATE 115200

/*ADC*/
#define ADC_RESOLUTION 12

/*PWM*/
#define PWM_FREQUENCY 2000 /* Frequency at which the PWM signal operates */
#define PWM_RESOLUTION 12  /* 12 bit resolution for PWM */
#define MAX_DUTY_CYCLE 4095 /* 2^PWM_RESOLUTION */
#define MIN_DUTY_CYCLE 0 

/*Sensors*/
#define CURRENT_K 29464 /* Conditioning circuit - Current to voltage conversion constant ( 29.464 * 1000 )*/
#define VOLTAGE_K 4114 /* Conditioning circuit - resistor divider constant  ( 41.14 * 100 ) */
#define TEMP_COEF 0.001F /* Example of temperature coefficient */
#define R_ZERO 1.5F /* Resistance of heatband at reference temperature */
#define T_ZERO 20 /* Reference temperature */

/*Control*/
#define MAX_TEMP 300
#define PID_KP 1
#define PID_KI 1
#define PID_KD 1
#define INTEGRAL_CLAMP 1000

/*Periods ( in microseconds) */
#define PERIOD_MAIN 100
#define PERIOD_230V 20000
/* !!!These must be 2^n multiple of PERIOD_MAIN!!!*/
#define PERIOD_POLLING 100
#define PERIOD_SAMPLING 100
#define PERIOD_DEBOUNCE 1600
#define PERIOD_PRINT 100
#define PERIOD_CONTROL 1600
#define PERIOD_SM_EXECUTE 200
#define PERIOD_DISPLAY 1000

#endif