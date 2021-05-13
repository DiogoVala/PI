#define CTRLpin_PWM 8

/**** Analog Pins ****/
#define ANALOGpin_current 15
#define ANALOGpin_voltage 16

/**** Digital Pins ****/
#define CTRLpin_OnOff 9

/*ADC*/
#define ADC_RESOLUTION 12

/*Sensors*/
#define CURRENT_K 29464 /* Conditioning circuit - Current to voltage conversion constant ( 29.464 * 1000 )*/
#define VOLTAGE_K 4562 /* Conditioning circuit - resistor divider constant  ( 41.14 * 100 ) */
#define TEMP_COEF 0.001F /* Example of temperature coefficient */
#define R_ZERO 1.01F /* Resistance of heatband at reference temperature */
#define T_ZERO 20 /* Reference temperature */

/*Main timekeeping period*/
#define PERIOD_MAIN 100

/*Periods ( in microseconds) */
#define PERIOD_SAMPLING 2000
#define PERIOD_230V 20000

#define TEST_DURATION_MS 2000

#define PWM_FREQUENCY 2000
#define PWM_RESOLUTION 12 
#define MAX_DUTY_CYCLE 4095 // Change according to PWM_RESOLUTION
#define MIN_DUTY_CYCLE 0



IntervalTimer Timer_Main; /* Interrupt timer*/
IntervalTimer Timer_230V;

static const uint32_t COUNT_SAMPLING = (PERIOD_SAMPLING / PERIOD_MAIN);
volatile uint32_t count_sampling = 0;

volatile uint32_t temp_measured = 0; 

volatile float current_rms = 0;
volatile float voltage_rms = 0;
volatile float resistance_rms = 0;

volatile float max_current_rms = 0;
volatile float max_voltage_rms = 0;
volatile float max_resistance_rms = 0;

volatile int32_t min_voltage =0;
volatile int32_t min_current =0;
volatile int32_t max_voltage =0;
volatile int32_t max_current =0;

/* Sampling */
volatile uint8_t sample_count = 0;
volatile int64_t sum_current = 0; /* Storage of current samples*/
volatile int64_t sum_voltage = 0; /* Storage of voltage samples*/

bool terminate = true;

static void _timer_ISR() {
	count_sampling++;
}


static void _calcTemp_ISR() {

	static const float T_slope=1/(TEMP_COEF*R_ZERO);
  	static const float T_b=1/TEMP_COEF-T_ZERO;

	voltage_rms = sqrt(sum_voltage / sample_count);
	if(voltage_rms>max_voltage_rms)
	{
		max_voltage_rms=voltage_rms;
	}

	current_rms = sqrt(sum_current / sample_count);
	if(current_rms>max_current_rms)
	{
		max_current_rms=current_rms;
	}

	resistance_rms = voltage_rms / current_rms;

	if(resistance_rms>max_resistance_rms)
	{
		max_resistance_rms=resistance_rms;
	}

	temp_measured = (resistance_rms*T_slope-T_b);

	sample_count = 0;
	sum_voltage = 0;
	sum_current = 0;
}

static int32_t sampleCurrent() {

	int32_t sample = 0;
	sample = analogRead(ANALOGpin_current);
	sample = (sample * 330000)/4095; /* [0 , 330000] V */
	//sample = sample - 165000; /* [-165000 , 165000] V */
	//sample = sample * 1000 / CURRENT_K; /* [-165000 , 165000] / 29464 = [-5600 , 5600] Amps*100 */
	sample=(((sample*100) / 3400)-4823);
	return sample;
}

static int32_t sampleVoltage() {
	int32_t sample = 0;
	sample = analogRead(ANALOGpin_voltage);
	sample = (sample * 330000)/4095; /* [0 , 330000] V */
	//sample = (sample - 165000); /* [-165000 , 165000] V */
	//Serial.println(sample)
	//sample = sample * VOLTAGE_K / 100000; /* [-165000 , 165000] V  * 4114 / 100000 = [-6788 , 6788] V*100 */
	sample = (((sample*100) / 2209 )-7466)-34;
	//Serial.println(sample);
	return sample;
}

void setup(){
	/*Analog Pins*/
	analogReadRes(ADC_RESOLUTION);

	/*Initialize digital outputs*/
	pinMode(CTRLpin_OnOff, OUTPUT);
	digitalWrite(CTRLpin_OnOff, LOW);

	pinMode(CTRLpin_PWM, OUTPUT);
	digitalWrite(CTRLpin_PWM, LOW);

	analogWriteResolution(PWM_RESOLUTION); /* With 12 bits, the frequency is 36621.09 Hz (teensy 4.1 doc)*/
  	analogWriteFrequency(CTRLpin_PWM, PWM_FREQUENCY);
  	analogWrite(CTRLpin_PWM, 0); /* Power controller control signal*/

  /*Start Timer ISR*/
	Timer_Main.begin(_timer_ISR, PERIOD_MAIN);
	Timer_230V.begin(_calcTemp_ISR, PERIOD_230V);

	Serial.print("\x1b[2J"); /*Clear screen*/
}

elapsedMillis time;
int i=0;
void loop()
{
	if(Serial.read()=='a')
	{
		terminate=false;
		time=0;
		max_current=0;
		max_voltage=0;
		max_voltage_rms=0;
		max_current_rms=0;
		max_resistance_rms=0;
		i++;
	}
	while(time<=TEST_DURATION_MS && terminate == false){
		digitalWrite(CTRLpin_OnOff, HIGH);
		analogWrite(CTRLpin_PWM, 0.55*MAX_DUTY_CYCLE); /* Power controller control signal*/
		if (count_sampling >= COUNT_SAMPLING)
		{
			int32_t sample;

			sample=sampleCurrent();
			sum_current += sample*sample;

			if(sample>max_current){
				max_current=sample;
			}
			if(sample<min_current){
				min_current=sample;
			}
			sample=sampleVoltage();
			sum_voltage += sample*sample;

			if(sample>max_voltage){
				max_voltage=sample;
			}
			if(sample<min_voltage){
				min_voltage=sample;
			}

			sample_count++;
			count_sampling = 0;
			
		}
	}
	if(terminate==false){
		Serial.print("\rTeste Numero ");
		Serial.println(i);
		Serial.print("\rMax Voltage RMS: ");
		Serial.println(max_voltage_rms);
		Serial.print("\rMax Current RMS: ");
		Serial.println(max_current_rms);
		Serial.print("\rMax Resistance: ");
		Serial.println(max_resistance_rms);
		Serial.print("\rMax Voltage Sample: ");
		Serial.println(max_voltage);
		Serial.print("\rMax Current Sample: ");
		Serial.println(max_current);
		Serial.print("\rMin Voltage Sample: ");
		Serial.println(min_voltage);
		Serial.print("\rMin Current Sample: ");
		Serial.println(min_current);
		Serial.print("\rTemperatura: ");
		Serial.println(temp_measured);
		
		Serial.print("\n\n");
		digitalWrite(CTRLpin_OnOff, LOW);
		analogWrite(CTRLpin_PWM, 0);
	}
	terminate=true;
}

