#include "DisplayDriver.h"
#include <string.h>
#include "ethernet.h"
#include "statemachine.h"
#include "error_handler.h"
#include "config.h"
#include <EEPROM.h>
#include "EEPROM_utils.h"
#include <NativeEthernet.h>

#define DISPLAY_BAUDRATE 115200
#define GRAPH_Y_RESOLUTION 160
#define GRAPH_MAX_TEMPERATURE 320
#define GRAPH_MAX_VOLTAGE 50
#define GRAPH_MAX_CURRENT 50

volatile uint32_t current_page = 0;
volatile uint8_t network_state = 0;
volatile char* local_IP={0};
volatile char* local_linkstatus={0};
static volatile int16_t error_count = 1; 

char buffer[30] = {0};
char error_log[500] = {0};

/*Home page*/
NexPage page_home = NexPage(1, 0, "Home");  // Page added as a touch event

/*Parameter page*/
NexPage page_param = NexPage(2, 0, "Parameter");  // Page added as a touch event
NexButton btn_validate = NexButton(2, 8, "b1");  // Button added
NexNumber num_preheat = NexNumber(2, 2, "n0"); // Text box added, so we can read it
NexNumber num_sealing = NexNumber(2, 6, "n1"); // Text box added, so we can read it
NexNumber num_tempcoef = NexNumber(2, 27, "x0"); // Text box added, so we can read it
NexNumber num_r_zero = NexNumber(2, 28, "x1"); // Text box added, so we can read it

/*Graph1 page*/
NexPage page_graph1 = NexPage(3, 0, "Graph1");  // Page added as a touch event
NexWaveform graph_temp = NexWaveform(3, 1, "s0");

/*Graph2 page*/
NexPage page_graph2 = NexPage(4, 0, "Graph2");  // Page added as a touch event
NexWaveform graph_voltage = NexWaveform(4, 4, "s0");

/*Graph3 page*/
NexPage page_graph3 = NexPage(5, 0, "Graph3");  // Page added as a touch event
NexWaveform graph_current = NexWaveform(5, 4, "s0");

/*Info page*/
NexPage page_info = NexPage(6, 0, "Info");  // Page added as a touch event

/*Help page*/
NexPage page_help = NexPage(7, 0, "Help");  // Page added as a touch event

/*Error page*/
NexPage page_error = NexPage(8, 0, "Error");  // Page added as a touch event

/*Network page*/
NexPage page_network = NexPage(9, 0, "Network");  // Page added as a touch event
NexButton btn_netonoff = NexButton(9, 3, "onoff");  // Button added
NexNumber num_port = NexNumber(9, 5, "portval"); // Text box added, so we can read it
NexText txt_static_ip = NexText(9, 17, "txt_static_ipval");  // Text box added, so we can read it

/*PID page*/
NexPage page_pid = NexPage(16, 0, "PID");  // Page added as a touch event
NexButton btn_valpid = NexButton(10, 6, "valpid");  // Button added
NexNumber num_KP = NexNumber(16, 25, "x0"); // Text box added, so we can read it
NexNumber num_KI = NexNumber(16, 26, "x1"); // Text box added, so we can read it
NexNumber num_KD = NexNumber(16, 27, "x2"); // Text box added, so we can read it
NexNumber num_IntegralLim = NexNumber(16, 28, "n0"); // Text box added, so we can read it

/*Log page*/
NexPage page_log = NexPage(17, 0, "Log");  // Page added as a touch event
NexButton btn_clear = NexButton(11, 4, "clear");  // Button added

NexTouch *nex_listen_list[] = 
{
  &btn_validate,  // Button added
  &btn_netonoff,
  &btn_valpid,
  &btn_clear,
  &page_home,
  &page_param,
  &page_graph1,
  &page_graph2,
  &page_graph3,
  &page_info,
  &page_help,
  &page_error,
  &page_network,
  &page_pid,
  &page_log,
  NULL  // String terminated
};  // End of touch event list

/* Function prototypes */
extern void pauseSystem();
extern void unpauseSystem();
void updateNetworkPage();
void terminateMessage();

/*Object Callbacks*/

void btn_validate_PopCallback(void *ptr)
{
  uint32_t local_preheat=0;
  uint32_t local_sealing=0;
  uint32_t local_temp_coef=0;
  uint32_t local_r_zero=0;

  num_preheat.getValue(&local_preheat);
  num_sealing.getValue(&local_sealing);
  num_tempcoef.getValue(&local_temp_coef);
  num_r_zero.getValue(&local_r_zero);

  writeInt32ToEEPROM(ADDR_TEMP_PREHEAT, local_preheat);
  writeInt32ToEEPROM(ADDR_TEMP_SEALING, local_sealing);
  writeInt32ToEEPROM(ADDR_TEMP_COEF, local_temp_coef);
  writeInt16ToEEPROM(ADDR_R_ZERO,local_r_zero);

  temp_preheat=local_preheat;
  temp_sealing=local_sealing;
  temp_coef=(float)local_temp_coef/NUMBOX_TEMP_COEF;
  r_zero=(float)local_r_zero/NUMBOX_R_ZERO;
}

void btn_netonoff_PopCallback(void *ptr)
{
  uint32_t local_port;

  if(network_state == 0)
  { 
    memset(buffer, 0, sizeof(buffer)); /* Clear buffer for IP */
    txt_static_ip.getText(buffer, sizeof(buffer)); /* Get IP string from textbox */
    strcat(buffer,"\0");

    if(setIP(buffer)!=0)
    {
       Serial1.print("txt_static_ipval.txt=\"IP Inválido\"");
       terminateMessage();
    }

    num_port.getValue(&local_port);

    if(setNetPort(local_port)==ERROR_INVALID_NETWORK_PORT)
    {
       Serial1.print("t3.txt=\"Porta Inválida\"");
       terminateMessage();
    }

    Serial1.print("portval.val=");
    Serial1.print(local_port);
    terminateMessage();

    Serial1.print("t3.txt=\"A conectar ...\"");
    terminateMessage();

    pauseSystem();

    InitEthernet();

    unpauseSystem();
    network_state = 1;
  }
  else
  {
    stopEthernet();
    network_state = 0;
  }
}

void btn_valpid_PopCallback(void *ptr)
{
  uint32_t local_pid_kp=0;
  uint32_t local_pid_ki=0;  
  uint32_t local_pid_kd=0;
  uint32_t local_pid_int_limit=0;

  num_KP.getValue(&local_pid_kp);
  num_KI.getValue(&local_pid_ki);  
  num_KD.getValue(&local_pid_kd);
  num_IntegralLim.getValue(&local_pid_int_limit);

  writeInt32ToEEPROM(ADDR_PID_KP, local_pid_kp);
  writeInt32ToEEPROM(ADDR_PID_KI, local_pid_ki);
  writeInt32ToEEPROM(ADDR_PID_KD, local_pid_kd);
  writeInt32ToEEPROM(ADDR_PID_INT_LIM, local_pid_int_limit);

  pid_kp=(float)local_pid_kp/NUMBOX_PID_K;
  pid_ki=(float)local_pid_ki/NUMBOX_PID_K;
  pid_kd=(float)local_pid_kd/NUMBOX_PID_K;
  pid_int_limit=local_pid_int_limit;
}

void btn_clear_PopCallback(void *ptr)
{
  memset(error_log, 0, sizeof(error_log)); /* Clear buffer for error log */
  error_count=1;
}

/*Page change Callbacks*/
void HomePagePushCallback(void *ptr)
{
  current_page = pg_HOME;
}

void ParamPagePushCallback(void *ptr)
{
  current_page = pg_PARAM;
  Serial1.print("n0.val=");
  Serial1.print(temp_preheat);
  terminateMessage();

  Serial1.print("n1.val=");
  Serial1.print(temp_sealing);
  terminateMessage();

  Serial1.print("x0.val=");
  Serial1.print((int)(temp_coef*NUMBOX_TEMP_COEF));
  terminateMessage();

  Serial1.print("x1.val=");
  Serial1.print((int)(r_zero*NUMBOX_R_ZERO));
  terminateMessage();
}

void Graph1PagePushCallback(void *ptr)
{
  current_page = pg_GRAPH1; 
  Serial.println(current_page);
}

void Graph2PagePushCallback(void *ptr)
{
  current_page = pg_GRAPH2;
  Serial.println(current_page);
} 

void Graph3PagePushCallback(void *ptr)
{
  current_page = pg_GRAPH3;
  Serial.println(current_page);
}

void InfoPagePushCallback(void *ptr)
{
  current_page = pg_INFO;
  Serial.println(current_page);
}

void HelpPagePushCallback(void *ptr)
{
  current_page = pg_HELP;
  Serial.println(current_page);
}

void ErrorPagePushCallback(void *ptr)
{
  current_page = pg_ERROR;
  Serial.println(current_page);
}

void NetworkPagePushCallback(void *ptr)
{
  current_page = pg_NETWORK;

  Serial1.print("portval.val=");
  Serial1.print(network_port);
  terminateMessage();

  Serial1.print("txt_static_ipval.txt=\"");
  for(uint8_t i=0; i<IP_ARRAY_SIZE; i++)
  {
    Serial1.print(static_ip_arr[i]);
    if (i < (IP_ARRAY_SIZE -1)) 
    {
      Serial1.print(".");
    }
  }
  Serial1.print("\"");
  terminateMessage();
}

void PIDPagePushCallback(void *ptr)
{
  current_page = pg_PID;

  Serial1.print("x0.val=");
  Serial1.print((int)(pid_kp*NUMBOX_PID_K));
  terminateMessage();

  Serial1.print("x1.val=");
  Serial1.print((int)(pid_ki*NUMBOX_PID_K));
  terminateMessage();

  Serial1.print("x2.val=");
  Serial1.print((int)(pid_kd*NUMBOX_PID_K));
  terminateMessage();

  Serial1.print("n0.val=");
  Serial1.print(pid_int_limit);
  terminateMessage();
}

void LogPagePushCallback(void *ptr)
{;
  current_page = pg_PID;
  Serial1.print("log.txt=");
  Serial1.print("\"");
  Serial1.print(error_log);
  Serial1.print("\"");
  terminateMessage();
}

/* Start Display */
void InitDisplay() {

  Serial1.begin(DISPLAY_BAUDRATE);

  /*Attach callback functions to objects*/
  btn_validate.attachPop(btn_validate_PopCallback);
  btn_netonoff.attachPop(btn_netonoff_PopCallback);
  btn_valpid.attachPop(btn_valpid_PopCallback);
  btn_clear.attachPop(btn_clear_PopCallback);
  page_home.attachPush(HomePagePushCallback);
  page_param.attachPush(ParamPagePushCallback);
  page_graph1.attachPush(Graph1PagePushCallback);
  page_graph2.attachPush(Graph2PagePushCallback);
  page_graph3.attachPush(Graph3PagePushCallback); 
  page_info.attachPush(InfoPagePushCallback);
  page_help.attachPush(HelpPagePushCallback);
  page_error.attachPush(ErrorPagePushCallback);
  page_network.attachPush(NetworkPagePushCallback);
  page_pid.attachPush(PIDPagePushCallback);
  page_log.attachPush(LogPagePushCallback);

  memset(error_log, 0, sizeof(error_log)); /* Clear buffer for error log */

  /*Force reset*/
  resetDisplay();
}

void updateDisplay(int state, int input_start, int input_preheat, int input_sealing) {

  uint32_t local_temp=temp_measured;
  uint32_t local_voltage=voltage_rms/100; /*Original value was multiplied by 100 for precision*/
  uint32_t local_current=current_rms/100;

  if(current_page == pg_HOME)
  {
    Serial1.print("t2.txt=\"");
    switch(state)
    {
      case st_IDLE: 
      Serial1.print("Inativo");
      break;
      case st_CYCLESTART: 
      Serial1.print("Pronto");
      break;
      case st_PREHEATING: 
      Serial1.print("Pré-aquec.");
      break;
      case st_SEAL: 
      Serial1.print("Selagem");
      break;
      default:
      Serial1.print("Inativo");
      break;
    } 
    Serial1.print("\"");
    terminateMessage();

    Serial1.print("tempval.val=");
    Serial1.print(temp_measured);
    terminateMessage();

    Serial1.print("preheatval.val=");
    Serial1.print(temp_preheat);
    terminateMessage();

    Serial1.print("sealval.val=");
    Serial1.print(temp_sealing);
    terminateMessage();

    if(input_start == HIGH){
      Serial1.print("t6.bco=47090");
    }
    else{
      Serial1.print("t6.bco=58417");
    }
    terminateMessage();

    if(input_preheat == HIGH){
      Serial1.print("t7.bco=47090");
    }
    else{
      Serial1.print("t7.bco=58417");
    }
    terminateMessage();

    if(input_sealing == HIGH){
      Serial1.print("t8.bco=47090");
    }
    else{
      Serial1.print("t8.bco=58417");
    }
    terminateMessage();
  }
  else if(current_page == pg_PARAM) {  
  }

  else if(current_page == pg_GRAPH1)
  {
    graph_temp.addValue(0,(int)local_temp*GRAPH_Y_RESOLUTION/GRAPH_MAX_TEMPERATURE);
    Serial1.print("n0.val=");
    Serial1.print(local_temp);
    terminateMessage();
  }
  else if(current_page == pg_GRAPH2)
  {
    graph_voltage.addValue(0,(int)(local_voltage*GRAPH_Y_RESOLUTION/GRAPH_MAX_VOLTAGE));
    Serial1.print("n0.val=");
    Serial1.print((int)(local_voltage));
    terminateMessage();
  }
  else if(current_page == pg_GRAPH3)
  {
    graph_current.addValue(0,(int)(local_current*GRAPH_Y_RESOLUTION/GRAPH_MAX_CURRENT));
    Serial1.print("n0.val=");
    Serial1.print((int)(local_current));
    terminateMessage();
  }
  else if(current_page == pg_NETWORK){
    if(network_state==0){
      btn_netonoff.setText("Ligar");

      Serial1.print("t3.txt=\"Offline\"");
      terminateMessage();

      Serial1.print("t4.txt=\"0.0.0.0\"");
      terminateMessage();
    }
    else
    {
      Serial1.print("t3.txt=\"");
      if(linkStatus()==0){ 
        Serial1.print("Online"); 
        Serial1.print("\"");
        terminateMessage();
        btn_netonoff.setText("Desligar");
      }
      else{
        Serial1.print("Offline");
        Serial1.print("\"");
        terminateMessage(); 
      }

      Serial1.print("t4.txt=\"Servidor em: ");
      getIP();
      Serial1.print(":");
      Serial1.print(network_port);
      Serial1.print("\"");
      terminateMessage();
    }
  }
}

void errorPage(int16_t error_code)
{
  static int16_t last_error = 0;
  if(last_error != error_code)
  {
    char error_code_str[ERROR_CODE_SIZE]={0};
    sprintf(error_code_str, "%02d: %d; ", error_count, error_code);
    strcat(error_log, error_code_str);
    if(error_count%5==0)
    {
      strcat(error_log, "\r\n");
    }
    last_error=error_code;
    error_count++;
  }

  //Serial1.print("page Error");
  //terminateMessage();
}

void resetDisplay()
{
  //current_page=pg_START;
  //Serial1.print("rest"); //Reset display
  //terminateMessage();

  //stopEthernet();
  //network_state = 0;
}

void eventCheck() {
  nexLoop(nex_listen_list);  // Check for any touch event
}

void terminateMessage(){
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);
}
