/*
   File:   config.h
   Author: Diogo Vala

   Overview: Constants for system operation
*/

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
/*Control signals*/
#define CTRLpin_PWM 8
#define CTRLpin_OnOff 9

/*General defs*/
#define LOW 0
#define HIGH 1

/*Uart*/
#define UART_BAUDRATE 9600

/*ADC*/
#define ADC_RESOLUTION 12

/*PWM*/
#define PWM_FREQUENCY_HZ 2000 
#define PWM_RESOLUTION 12 
#define MAX_DUTY_CYCLE 4095 /* Change according to PWM_RESOLUTION*/
#define MIN_DUTY_CYCLE 0

/*Main timekeeping period*/
#define PERIOD_MAIN 100

/*Periods ( in microseconds) */
#define PERIOD_230V 20000
#define PERIOD_POLLING 1000
#define PERIOD_SAMPLING 100
#define PERIOD_CONTROL 20000
#define PERIOD_SM_EXECUTE 1000
#define PERIOD_DISPLAY 100000

/*Sensors*/
#define CURRENT_M 1.947/* Linearization slope */
#define CURRENT_B 3986/* Linearization constant */
#define VOLTAGE_M 3.33 /* Linearization slope */
#define VOLTAGE_B 6822 /* Linearization constant */
#define T_ZERO 20 /* Ambient temperature */
#define MAX_CALIBRATION_VALUE 250 /* System will not calibrate if sensor value is at no load is larger than this*/

/*Limits*/
#define MAX_TEMPERATURE 300 /* Temperature limit before system error*/
#define MAX_PREHEAT 300 /* Maximum preheat temperatura value*/
#define MAX_SEALING 300 /* Maximum sealing temperatura value*/
#define MAX_VOLTAGE_RMS 7000 /* Maximum RMS voltage *100 */
#define MAX_CURRENT_RMS 5000 /* Maximum RMS current *100 */
#define MIN_VOLTAGE_RMS 101 /* Maximum RMS voltage *100 */
#define MIN_CURRENT_RMS 100 /* Maximum RMS current *100 */
#define MAX_RESISTANCE 2 /* Maximum resistance value for heatband*/
#define MIN_SENSOR_VAL 50 /* Ignore sensor current/voltage < X */
#define POT_HYSTERESIS 5 /* Temperature hysteresis - Only update sealing if new pot temp is above this*/
