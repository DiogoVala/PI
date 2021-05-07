#include "DisplayDriver.h"
#include "config.h"

volatile char state[][20]=
{ "IDLE", 
"CYCLESTART", 
"PREHEAT", 
"SEALING"
};

/*Home page*/
NexPage page_home = NexPage(1, 0, "Home");  // Page added as a touch event

/*Parameter page*/
NexPage page_param = NexPage(2, 0, "Parameter");  // Page added as a touch event
NexButton btn_validate = NexButton(2, 8, "b1");  // Button added
NexNumber num_preheat = NexNumber(2, 2, "n0"); // Text box added, so we can read it
NexNumber num_sealing = NexNumber(2, 6, "n1"); // Text box added, so we can read it
/*Missing the cal. number boxes*/

/*Graph1 page*/
NexPage page_graph1 = NexPage(3, 0, "Graph1");  // Page added as a touch event
NexWaveform graph_temp = NexWaveform(3, 1, "s0");

/*Graph2 page*/
NexPage page_graph2 = NexPage(4, 0, "Graph2");  // Page added as a touch event
NexWaveform graph_voltage = NexWaveform(4, 4, "s0");

/*Graph3 page*/
NexPage page_graph3 = NexPage(5, 0, "Graph3");  // Page added as a touch event
NexWaveform graph_current = NexWaveform(5, 4, "s0");

/*Error page*/

NexTouch *nex_listen_list[] = 
{
  &btn_validate,  // Button added
  &page_home,
  &page_param,
  &page_graph1,
  &page_graph2,
  &page_graph3,
  NULL  // String terminated
};  // End of touch event list

volatile uint32_t current_page = 0;

void btn_validate_PopCallback(void *ptr)
{
  uint32_t local_preheat=0;
  uint32_t local_sealing=0;
  num_preheat.getValue(&local_preheat);
  num_sealing.getValue(&local_sealing);

  temp_preheat=local_preheat;
  temp_sealing=local_sealing;

}  // End of press event

void HomePushCallback(void *ptr)
{
  current_page = 1;  // Set variable as 0 so from now on arduino knows page 0 is loaded on the display
  Serial.println(current_page);
}  // End of press event

void ParamPushCallback(void *ptr)
{
  current_page = 2; 

  Serial1.print("n0.val=");
  Serial1.print(temp_preheat);
  Serial1.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("n1.val=");
  Serial1.print(temp_sealing);
  Serial1.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial1.write(0xff);
  Serial1.write(0xff);
}

void Graph1PushCallback(void *ptr)
{
  current_page = 3;  // Set variable as 0 so from now on arduino knows page 0 is loaded on the display
  Serial.println(current_page);
}  // End of press event

void Graph2PushCallback(void *ptr)
{
  current_page = 4;  // Set variable as 0 so from now on arduino knows page 0 is loaded on the display
  Serial.println(current_page);
}  // End of press event

void Graph3PushCallback(void *ptr)
{
  current_page = 5;  // Set variable as 0 so from now on arduino knows page 0 is loaded on the display
  Serial.println(current_page);
}  // End of press event

void InitDisplay() {
  // put your setup code here, to run once:
  Serial1.begin(DISPLAY_BAUDRATE);  // Display Baudrate

  btn_validate.attachPop(btn_validate_PopCallback);
  page_home.attachPush(HomePushCallback);  // Page press event
  page_param.attachPush(ParamPushCallback);  // Page press event
  page_graph1.attachPush(Graph1PushCallback);  // Page press event
  page_graph2.attachPush(Graph2PushCallback);  // Page press event
  page_graph3.attachPush(Graph3PushCallback);  // Page press event

  resetDisplay();
}

void updateHome(int state, int input_start,int input_preheat, int input_sealing){
  if(current_page==1)
  {
    Serial1.print("t2.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
    Serial1.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
    switch(state)
    {
      case 3: 
      Serial1.print("IDLE");
      break;
      case 4: 
      Serial1.print("CYCLESTART");
      break;
      case 5: 
      Serial1.print("PREHEATING");
      break;
      case 6: 
      Serial1.print("SEALING");
      break;
      default:
      Serial1.print("IDLE");
      break;
    } 
    Serial1.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
    Serial1.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
    Serial1.write(0xff);
    Serial1.write(0xff);

    Serial1.print("n0.val=");
    Serial1.print(temp_measured);
    Serial1.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
    Serial1.write(0xff);
    Serial1.write(0xff);

    Serial1.print("n1.val=");
    Serial1.print(temp_preheat);
    Serial1.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
    Serial1.write(0xff);
    Serial1.write(0xff);

    Serial1.print("n2.val=");
    Serial1.print(temp_sealing);
    Serial1.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
    Serial1.write(0xff);
    Serial1.write(0xff);

    if(input_start == 1){
      Serial1.print("t6.bco=47090");
    }
    else{
      Serial1.print("t6.bco=58417");
    }
    Serial1.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
    Serial1.write(0xff);
    Serial1.write(0xff);

    if(input_preheat == 1){
      Serial1.print("t7.bco=47090");
    }
    else{
      Serial1.print("t7.bco=58417");
    }
    Serial1.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
    Serial1.write(0xff);
    Serial1.write(0xff);

    if(input_sealing == 1){
      Serial1.print("t8.bco=47090");
    }
    else{
      Serial1.print("t8.bco=58417");
    }
    Serial1.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
    Serial1.write(0xff);
    Serial1.write(0xff);
  }
}

void updateGraphs(){
  uint16_t local_temp=temp_measured;
  if(local_temp>300)
  {
    local_temp=300;
  }
  if(current_page==3)
  {
    graph_temp.addValue(0,local_temp*160/300);
    Serial1.print("n0.val=");
    Serial1.print(local_temp);
    Serial1.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
    Serial1.write(0xff);
    Serial1.write(0xff);
  }
  else if(current_page==4)
  {
    graph_voltage.addValue(0,(int)(voltage_rms*160/6788));
  }
  else if(current_page==5)
  {
    graph_current.addValue(0,(int)(current_rms*160/5600));
  }
}

void errorPage()
{
  Serial1.print("page ");
  Serial1.print("Error");
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("t1.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
  Serial1.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial1.print("MAX TEMP EXCEEDED");  // This is the text you want to send to that object and atribute mentioned before.
  Serial1.print("\"");  // Since we are sending text, and not a number, we need to send double quote before and after the actual text.
  Serial1.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
  Serial1.write(0xff);
  Serial1.write(0xff);
}

void resetDisplay()
{
  Serial1.print("page ");
  Serial1.print("Start");
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);
}

void eventCheck() {
  nexLoop(nex_listen_list);  // Check for any touch event
}
