#ifndef ETHERNET_API_H
#define ETHERNET_API_H

// Include Arduino header
#include "Arduino.h"

#define OUTPUT_BUFFER_SIZE 10000
//#endif

// Hardware data
#define HARDWARE "Teensy 4.1"

// Size of name & ID
#define NAME_SIZE 20
#define ID_SIZE 10

// Use AREST_PARAMS_MODE to control how parameters are parsed when using the function() method.
// Use 0 for standard operation, where everything before the first "=" is stripped before passing the parameter string to the function.
// Useful for simple functions, where only the value is important
//    function?params=hello    ==> hello gets passed to the function
//
// Use 1 to pass the entire parameter string to the function, which will be responsible for parsing the parameter string
// Useful for more complex situations, where the key name as well as its value is important, or there are mutliple key-value pairs
//    function?params=hello    ==> params=hello gets passed to the function
#ifndef AREST_PARAMS_MODE
#define AREST_PARAMS_MODE 1
#endif

// Use light answer mode
#ifndef LIGHTWEIGHT
#define LIGHTWEIGHT 1
#endif 

// Default number of max. exposed variables
#define NUMBER_VARIABLES 10

// Default number of max. exposed functions
#define NUMBER_FUNCTIONS 10

#ifdef AREST_BUFFER_SIZE
  #define OUTPUT_BUFFER_SIZE AREST_BUFFER_SIZE
#endif


class aREST {

private:
  struct Variable {
    virtual void addToBuffer(aREST *arest) const = 0;
  };


template<typename T>
  struct TypedVariable: Variable {
    T *var;
    bool quotable;

    TypedVariable(T *v, bool q) : var{v} { quotable = q; }

    void addToBuffer(aREST *arest) const override { 
      arest->addToBuffer(*var, quotable);
    }  
  };

public:

public:

  aREST() {
    initialize();
  }

  aREST(char* rest_remote_server, int rest_port) {

    initialize();

    remote_server = rest_remote_server;
    port = rest_port;

  }


template<typename T>
  void variable(const char *name, T *var, bool quotable) { 
    variables[variables_index] = new TypedVariable<T>(var, quotable);
    variable_names[variables_index] = name;
    variables_index++;
  }

template<typename T>
  void variable(const char *name, T *var) { 
    variable(name, var, true);
  }


private:

  void initialize() {
    reset();
    status_led_pin = 255;
  }

// Used when resetting object back to oringial state
  void reset() {
    command = 'u';
    state = 'u';
    pin_selected = false;
  }

public:

  void addToBufferF(const __FlashStringHelper *toAdd){

    PGM_P p = reinterpret_cast<PGM_P>(toAdd);

    for ( unsigned char c = pgm_read_byte(p++);
      c != 0 && index < OUTPUT_BUFFER_SIZE;
      c = pgm_read_byte(p++), index++) {
      buffer[index] = c;
  }
}


// Reset variables after a request
void reset_status() {

  reset();
  answer = "";
  arguments = "";

  index = 0;
}

// Handle request for the Arduino Ethernet shield
#ifdef ethernet_h_
void handle(EthernetClient& client){

  if (client.available()) {

    // Handle request
    handle_proto(client,true,0,false);

    // Answer
    sendBuffer(client,50,0);
    client.stop();

    // Reset variables for the next command
    reset_status();
  }
}

template <typename T>
void publish(EthernetClient& client, const String& eventName, T value) {

  // Publish request
  publish_proto(client, eventName, value);

}
#endif

void handle(char * string) {

  // Process String
  handle_proto(string);

  // Reset variables for the next command
  reset_status();
}

void handle_proto(char * string) {
  // Check if there is data available to read
  for (int i = 0; i < (int)strlen(string); i++){

    char c = string[i];
    answer = answer + c;

    // Process data
    process(c);

  }

  // Send command
  send_command(false, false);
}

template <typename T>
void handle_proto(T& serial, bool headers, uint8_t read_delay, bool decode)
{

  // Check if there is data available to read
  while (serial.available()) {

    // Get the server answer
    char c = serial.read();
    delay(read_delay);
    answer = answer + c;

    // Process data
    process(c);

  }

   // Send command
  send_command(headers, decode);
}

void process(char c) {

  // Check if we are receveing useful data and process it

  if(state != 'u')
    return;

  if(c != '/' && c != '\r')
    return;

  // Variable or function request received ?
  if (command == 'u') {

    // Check if function name is in array
    for (uint8_t i = 0; i < functions_index; i++) {
      if (answer.startsWith(functions_names[i])) {

        // End here
        pin_selected = true;
        state = 'x';

        // Set state
        command = 'f';
        value = i;

        answer.trim();

        // We're expecting a string of the form <functionName>?xxxxx=<arguments>, where xxxxx can be almost anything as long as it's followed by an '='
        // Get command -- Anything following the first '=' in answer will be put in the arguments string.
        arguments = "";
        uint16_t header_length = strlen(functions_names[i]);
        if (answer.substring(header_length, header_length + 1) == "?") {
          uint16_t footer_start = answer.length();
          if (answer.endsWith(" HTTP/"))
            footer_start -= 6; // length of " HTTP/"

          // Standard operation --> strip off anything preceeding the first "=", pass the rest to the function
          if(AREST_PARAMS_MODE == 0) {
            uint16_t eq_position = answer.indexOf('=', header_length); // Replacing 'magic number' 8 for fixed location of '='
            if (eq_position != -1)
              arguments = answer.substring(eq_position + 1, footer_start);
          } 
          // All params mode --> pass all parameters, if any, to the function.  Function will be resonsible for parsing
          else if(AREST_PARAMS_MODE == 1) {
            arguments = answer.substring(header_length + 1, footer_start);
          }
        }

        break; // We found what we're looking for
      }
    }

    if (command == 'u' && answer[0] == ' ') {

      // Set state
      command = 'r';

      // End here
      pin_selected = true;
      state = 'x';
    }
  }

  answer = "";
}

// Modifies arguments in place
void urldecode(String &arguments) {
  char a, b;
  int j = 0;
  for(int i = 0; i < (int)arguments.length(); i++) {
    // %20 ==> arguments[i] = '%', a = '2', b = '0'
    if ((arguments[i] == '%') && ((a = arguments[i + 1]) && (b = arguments[i + 2])) && (isxdigit(a) && isxdigit(b))) {
      if (a >= 'a') a -= 'a'-'A';
      if (a >= 'A') a -= ('A' - 10);
      else          a -= '0';

      if (b >= 'a') b -= 'a'-'A';
      if (b >= 'A') b -= ('A' - 10);
      else          b -= '0';

      arguments[j] = char(16 * a + b);
      i += 2;   // Skip ahead
    } else if (arguments[i] == '+') {
      arguments[j] = ' ';
    } else {
     arguments[j] = arguments[i];
   }
   j++;
 }

  arguments.remove(j);    // Truncate string to new possibly reduced length
}


bool send_command(bool headers, bool decodeArgs) {

  // Start of message

  // Function selected
  if (command == 'f') {

    // Execute function
    if (decodeArgs)
      urldecode(arguments); // Modifies arguments

    int result = functions[value](arguments);

    // Send feedback to client
    addToBuffer(result, true);
  }

  if (command == 'r' || command == 'u') {
    root_answer();
  }

  // End of message
  addToBufferF(F("\r\n"));

  return true;
}


virtual void root_answer() {
  addToBufferF(F("{"));
  addHardwareToBuffer();
}


void function(char * function_name, int (*f)(String)){

  functions_names[functions_index] = function_name;
  functions[functions_index] = f;
  functions_index++;
}

// Set device ID
void set_id(const String& device_id) {

  id = device_id.substring(0, ID_SIZE);
}

// Set device name
void set_name(char *device_name){
  strcpy(name, device_name);
}

// Set device name
void set_name(const String& device_name){
  device_name.toCharArray(name, NAME_SIZE);
}

// Remove last char from buffer
void removeLastBufferChar() {
  index = index - 1;

}

void addQuote() {
  if(index < OUTPUT_BUFFER_SIZE) {
    buffer[index] = '"';
    index++;
  }  
}

void addStringToBuffer(const char * toAdd, bool quotable){

  if(quotable) {
    addQuote();
  }

  for (int i = 0; i < (int)strlen(toAdd) && index < OUTPUT_BUFFER_SIZE; i++, index++) {
    // Handle quoting quotes and backslashes
    if(quotable && (toAdd[i] == '"' || toAdd[i] == '\\')) {
      if(index == OUTPUT_BUFFER_SIZE - 1)   // No room!
        return;
      buffer[index] = '\\';
      index++;
    }

    buffer[index] = toAdd[i];
  }

  if(quotable) {
    addQuote();
  }
}

// Add to output buffer
template <typename T>
void addToBuffer(T toAdd, bool quotable=false) {
  addStringToBuffer(String(toAdd).c_str(), false);   // Except for our overrides, this will be adding numbers, which don't get quoted
}

// Register a function instead of a plain old variable!
template <typename T>
void addToBuffer(T(*toAdd)(), bool quotable=true) { 
  addToBuffer(toAdd(), quotable);
} 

template <typename T>
void sendBuffer(T& client, uint8_t chunkSize, uint8_t wait_time) {

  // Send all of it
  if (chunkSize == 0) {
    client.print(buffer);
  }

  // Send chunk by chunk
  else {

    // Max iteration
    uint8_t max_iteration = (int)(index/chunkSize) + 1;

    // Send data
    for (uint8_t i = 0; i < max_iteration; i++) {
      char intermediate_buffer[chunkSize+1];
      memcpy(intermediate_buffer, buffer + i*chunkSize, chunkSize);
      intermediate_buffer[chunkSize] = '\0';

      // Send intermediate buffer
      #ifdef ADAFRUIT_CC3000_H
      client.fastrprint(intermediate_buffer);
      #else
      client.print(intermediate_buffer);
      #endif

      // Wait for client to get data
      delay(wait_time);

    }
  }

    // Reset the buffer
  resetBuffer();
}

char * getBuffer() {
  return buffer;
}

void resetBuffer(){

  memset(&buffer[0], 0, sizeof(buffer));
  // free(buffer);

}


void addVariableToBuffer(uint8_t index) {
  addStringToBuffer(variable_names[index], true);
  addToBufferF(F(": "));
  variables[index]->addToBuffer(this);
}

void addHardwareToBuffer() {

  addToBufferF(F("\"name\": "));
  addStringToBuffer(name, true);
  addToBufferF(F(", \"hardware\": "));
  addStringToBuffer(HARDWARE, true);
  addToBufferF(F(", \"connected\": true}"));
}


private:
  String answer;
  char command;
  uint8_t pin;
  uint8_t message_pin;
  char state;
  uint16_t value;
  boolean pin_selected;

  char* remote_server;
  int port;

  char name[NAME_SIZE];
  String id;
  String proKey;
  String arguments;

  // Output uffer
  char buffer[OUTPUT_BUFFER_SIZE];
  uint16_t index;

  // Status LED
  uint8_t status_led_pin;

  // Interval
  uint32_t previousMillis;
  uint32_t interval = 1000;

  // Int variables arrays
  uint8_t variables_index;
  Variable* variables[NUMBER_VARIABLES];
  const char * variable_names[NUMBER_VARIABLES];

  // Functions array
  uint8_t functions_index;
  int (*functions[NUMBER_FUNCTIONS])(String);
  char * functions_names[NUMBER_FUNCTIONS];

};


// Some specializations of our template
template <>
void aREST::addToBuffer(bool toAdd, bool quotable) {
  addStringToBuffer(toAdd ? "true" : "false", false);   // Booleans aren't quoted in JSON
}


template <>
void aREST::addToBuffer(const char *toAdd, bool quotable) {
  addStringToBuffer(toAdd, quotable);                       // Strings must be quoted
}


template <>
void aREST::addToBuffer(const String *toAdd, bool quotable) {
  addStringToBuffer(toAdd->c_str(), quotable);           // Strings must be quoted
}


template <>
void aREST::addToBuffer(const String toAdd, bool quotable) {
  addStringToBuffer(toAdd.c_str(), quotable);           // Strings must be quoted
}


template <>
void aREST::addToBuffer(char toAdd[], bool quotable) {
  addStringToBuffer(toAdd, quotable);           // Strings must be quoted
}

#endif
