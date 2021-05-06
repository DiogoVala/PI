#include <Nextion.h> 

extern volatile uint32_t temp_user_setpoint;
extern volatile uint32_t temp_preheat;
extern volatile uint32_t temp_measured;
extern volatile float current_rms;
extern volatile float voltage_rms;

void InitDisplay();
void updateGraphs();
void eventCheck();
