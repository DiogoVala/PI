#include <Nextion.h> 

extern volatile uint32_t temp_sealing;
extern volatile uint32_t temp_preheat;
extern volatile uint32_t temp_measured;
extern volatile float current_rms;
extern volatile float voltage_rms;

void InitDisplay();
void updateHome(int x);
void updateGraphs();
void eventCheck();
void errorPage();
void resetDisplay();
