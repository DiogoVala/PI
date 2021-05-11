
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
#define CTRLpin_zerocross 10

/*General defs*/
#define LOW 0
#define HIGH 1

/*Uart*/
#define UART_BAUDRATE 9600

/*ADC*/
#define ADC_RESOLUTION 12

/*PWM*/
#define PWM_FREQUENCY 2000
#define PWM_RESOLUTION 12 
#define MAX_DUTY_CYCLE 4095 // Change according to PWM_RESOLUTION
#define MIN_DUTY_CYCLE 0

/*Sensors*/
#define CURRENT_K 29464 /* Conditioning circuit - Current to voltage conversion constant ( 29.464 * 1000 )*/
#define VOLTAGE_K 4861 /* Conditioning circuit - resistor divider constant  ( 41.14 * 100 ) */
#define TEMP_COEF 0.001F /* Example of temperature coefficient */
#define R_ZERO 1.01F /* Resistance of heatband at reference temperature */
#define T_ZERO 20 /* Reference temperature */

/*Control*/
#define MAX_TEMPERATURE 300
#define MAX_PREHEAT 300
#define MAX_SEALING 300
#define MAX_VOLTAGE 50
#define MAX_CURRENT 40
#define PID_KP 10
#define PID_KI 5
#define PID_KD 10
#define INTEGRAL_CLAMP 20

/*Main timekeeping period*/
#define PERIOD_MAIN 100

/*Periods ( in microseconds) */
#define PERIOD_230V 20000
#define PERIOD_POLLING 100
#define PERIOD_SAMPLING 2000
#define PERIOD_CONTROL 20000
#define PERIOD_SM_EXECUTE 1000
#define PERIOD_DISPLAY 100000