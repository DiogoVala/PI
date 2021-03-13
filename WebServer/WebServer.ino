//https://forum.arduino.cc/index.php?topic=94861.0

//zoomkat 1-10-11
//web LED code
//for use with IDE 1.0
//open serial monitor to see what the arduino receives
//use the \ slash to escape the " in the html
//address will look like http://192.168.1.102:84/ when submited
//for use with W5100 based ethernet shields

#include <SPI.h>
#include <NativeEthernet.h>

#define pot 14  // pino do potenciómetro

volatile int val;

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED }; //physical mac address
byte ip[] = { 192, 168, 1, 126 }; // ip in lan
byte gateway[] = { 192, 168, 1, 1 }; // internet access via router
byte subnet[] = { 255, 255, 255, 0 }; //subnet mask
EthernetServer server(80); //server port

String readString;

//////////////////////

void setup(){

  //pinMode(pot, INPUT);
  
  //start Ethernet
  Ethernet.begin(mac, ip, gateway, subnet);
  server.begin();
  analogReadRes(12);
  
  //enable serial data print
  Serial.begin(9600);
  Serial.println("servertest1"); // so I can keep track of what is loaded
}

void loop(){
  // Create a client connection
  EthernetClient client = server.available(); // verifica se existe um cliente (no nosso caso, o browser) a tentar conectar

  val = analogRead(pot);
  Serial.println(val);
  
  if (client) {   // se houver algum cliente disponível
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();

        //read char by char HTTP request
        if (readString.length() < 100) {

          //store characters to string
          readString += c;
          //Serial.print(c); //uncomment to see in serial monitor
        }

        //if HTTP request has ended
        if (c == '\n') {

          ///////////////
          Serial.println(readString);

          //now output HTML data header

          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/html");
          client.println();

          //envia página WEB
          client.println("<HTML>");
          client.println("<HEAD>");
          client.println("<meta http-equiv=\"refresh\" content=\"1\">");
          client.println("<TITLE>Ler Pot.</TITLE>");
          client.println("</HEAD>");
          client.println("<BODY>");

          client.println("<H1>Leitura do valor de um potenci&oacutemetro</H1>");

          client.println("<FORM ACTION=\"http://192.168.1.126:80\" method=get >");
          
          client.print("Valor do Potenci&oacutemetro: ");
          client.println(val);          
          
          client.println("</FORM>");

          client.println("<BR>");

          client.println("</BODY>");
          client.println("</HTML>");

          delay(1);   // para permitir que o rowser receba toda a informação que lhe é enviada
          //stopping client
          client.stop();

          
          //clearing string for next read
          readString="";

        }
      }
    }
  }
}
