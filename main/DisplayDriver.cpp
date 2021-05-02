#include "DisplayDriver.h"

static NexButton btn_param		= NexButton(0, 1, "b0"); // Button added
static NexButton btn_validate	= NexButton(1, 8, "b1"); // Button added
static NexNumber box_preheat 	= NexNumber(1, 2, "n0"); // Text box added, so we can read it
static NexNumber box_seal 		= NexNumber(1, 6, "n1"); // Text box added, so we can read it
static NexNumber box_calV 		= NexNumber(1, 11, "n2"); // Text box added, so we can read it
static NexNumber box_calI 		= NexNumber(1, 12, "n3"); // Text box added, so we can read it
static NexWaveform graph_temp	= NexWaveform(2, 1, "s0");
static NexWaveform graph_V 		= NexWaveform(3, 1, "s0");
static NexWaveform graph_I 		= NexWaveform(4, 1, "s0");

static NexTouch *nex_listen_list[] = 
{
	&btn_param,
	&btn_validate,  // Button added
  NULL  // String terminated
};  // End of touch event list

void btn_param_event(void *ptr)
{
  uint32_t val=200;
	//box_preheat.setValue(&val);
  Serial1.print("n0.val=");
  Serial1.print(val);
  Serial1.print("0xff");
  Serial1.print("0xff");
  Serial1.print("0xff");
	box_seal.setValue(&temp_user_setpoint);

}  // End of press even

void btn_validate_event(void *ptr)
{
  box_preheat.getValue(&temp_preheat);
  box_seal.getValue(&temp_user_setpoint);

}  // End of press event

void InitDisplay() {
  Serial1.begin(115200);
  //nexInit();
  btn_param.attachPush(btn_param_event);
  btn_validate.attachPush(btn_validate_event);  // Validate Button

}

void sendGraphVal(){
	graph_temp.addValue(0,temp_measured);
}

void checkDisplayEvent(){
	nexLoop(nex_listen_list);  // Check for any touch event
}
