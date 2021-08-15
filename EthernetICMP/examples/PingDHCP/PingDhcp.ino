#include <EthernetICMP.h>
#include <Ethernet.h>
#include <SPI.h>

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
int timeout = 1; // in seconds

IPAddress pingAddr(1, 1, 1, 1); // ip address to ping
const SOCKET pingSocket = 0;
EthernetICMPPing pingip(pingSocket, (uint8_t)random(0, 255));

char buffer [256];

void setup() {
  Serial.begin(9600);
  pingip.setTimeout(timeout * 1000);

  // DHCP
  while (true) {
    Serial.println("Initialize Ethernet with DHCP..");
    if (Ethernet.begin(mac) == 0) {
      Serial.println("Failed to configure Ethernet using DHCP");
      if (Ethernet.hardwareStatus() == EthernetNoHardware) {
        Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
      } else if (Ethernet.linkStatus() == LinkOFF) {
        Serial.println("Ethernet cable is not connected.");
      }
    } else {
      // print your local IP address:
      Serial.print("IP address: ");
      Serial.println(Ethernet.localIP());
      break;
    }
    Serial.println();
  }
}

void loop() {
	EthernetICMPEchoReply echoReply = pingip(pingAddr, 1);
  if (echoReply.status == SUCCESS){
    sprintf(buffer,
            "Reply[%d] from: %d.%d.%d.%d: bytes=%d time=%ldms TTL=%d",
            echoReply.data.seq,
            echoReply.addr[0],
            echoReply.addr[1],
            echoReply.addr[2],
            echoReply.addr[3],
            REQ_DATASIZE,
            millis() - echoReply.data.time,
            echoReply.ttl);
  } else {
    sprintf(buffer, "Echo request failed; %d", echoReply.status);
  }
  Serial.println(buffer);
  delay(500);

  // Maintain DHCP
  switch (Ethernet.maintain()) {
    case 1:
      Serial.println("Error: renewed fail"); // renewal failed
      break;
    case 2:
      Serial.println("Renewed success"); //renewed success
      Serial.print("IP address: ");
      Serial.println(Ethernet.localIP());
      break;
    case 3:
      Serial.println("Error: rebind fail"); //rebind fail
      break;
    case 4:
      Serial.println("Rebind success"); //rebind success
      Serial.print("IP address: ");
      Serial.println(Ethernet.localIP());
      break;
    default:
      break;
  }
}