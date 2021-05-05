#include <SPI.h>
#include <NativeEthernet.h>

#define pot 14  // Pot pin

volatile int val; // ADC val

//Network config
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED }; // Mac address
byte ip[] = { 192, 168, 2, 100 }; // Lan IP
byte gateway[] = { 192, 168, 2, 1 }; // Default Gateway
byte subnet[] = { 255, 255, 255, 0 }; //Subnet Mask
EthernetServer server(80); //server port

void setup() {
  
  // Start Ethernet
  Ethernet.begin(mac, ip, gateway, subnet);
  server.begin();

  //ADC resolution
  analogReadRes(12);

  // Enable serial data print
  Serial.begin(9600);
  Serial.println("Server Test");
}

void loop() {
  // Create a client connection
  EthernetClient client = server.available(); // verifica se existe um cliente (no nosso caso, o browser) a tentar conectar

  val = analogRead(pot);
  //Serial.println(val);

  if (client) {   // se houver algum cliente disponível
    while (client.connected()) {
      if (client.available()) {
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

        client.println("<FORM ACTION=\"http://192.168.2.100:80\" method=get >");

        client.print("Valor do Potenci&oacutemetro: ");
        client.println(val);

        client.println("</FORM>");

        client.println("<BR>");

        client.println("</BODY>");
        client.println("</HTML>");

        delay(1);   // para permitir que o rowser receba toda a informação que lhe é enviada
        //stopping client
        client.stop();
      }
    }
  }
}
