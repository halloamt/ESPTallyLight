#include <Arduino.h>
#include <Ethernet.h>
#include <FastLED.h>
#include <esp_now.h>
#include <WiFi.h>
#include <EthernetBonjour.h>
#include <ATEMstd.h>

#define SCK  22
#define MISO 23
#define MOSI 33
#define CS   19

#define NUM_LEDS  1
#define DATA_PIN1 27

#define TALLY_FLAG_OFF                  0
#define TALLY_FLAG_PROGRAM              1
#define TALLY_FLAG_PREVIEW              2

/*
Server: 4C:75:25:D6:89:D4
1: 4C:75:25:95:1D:C0
2: 4C:75:25:95:B6:D0
3: 4C:75:25:95:1D:0C
*/

#define NUM_PEERS 4
uint8_t MACs[NUM_PEERS][6] = {
  {0x4C, 0x75, 0x25, 0xD6, 0x89, 0xD4},
  {0x4C, 0x75, 0x25, 0x95, 0x1D, 0xC0},
  {0x4C, 0x75, 0x25, 0x95, 0xB6, 0xD0},
  {0x4C, 0x75, 0x25, 0x95, 0x1D, 0x0C}
};
esp_now_peer_info_t peerInfo;

byte mac[] = {0xDE, 0xAD, 0x83, 0x29, 0x01, 0x01};
byte myMac[6];
CRGB leds[NUM_LEDS];
IPAddress ip(192, 168, 35, 177);
IPAddress myDns(192, 168, 35, 1);
IPAddress gateway(192, 168, 35, 1);
IPAddress subnet(255, 255, 255, 0);
bool Server = true;
bool SentOK = true;
bool ServerError = false;
bool firstRun = true;
int LightNumber = -1;
int program = 0;
int preview = 0;
int sendAgain = 0;
#define SEND_INTERVAL 20
#define DELAY 50

byte switcherIP[4] = {0, 0, 0, 0};
ATEMstd atemSwitcher;

enum Status {
  Searching = 0,
  Connected = 1,
  Error = 2
};

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  //char mac[32];
  int program;
  int preview;
  Status state;

} struct_message;
struct_message myData;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");

  if(status == ESP_NOW_SEND_SUCCESS)
  {
    SentOK = true;
  }
  else
  {
    SentOK = false;
  }
  //FastLED.show();
}

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  if(!Server)
  {
    memcpy(&myData, incomingData, sizeof(myData));
    //Serial.printf("Preview: %d - Program: %d", myData.preview, myData.program);
    //Serial.println();
    SentOK = true;
  }
}

// This is just a little utility function to format an IP address as a string.
const char* ip_to_str(const uint8_t* ipAddr)
{
  static char buf[16];
  sprintf(buf, "%d.%d.%d.%d\0", ipAddr[0], ipAddr[1], ipAddr[2], ipAddr[3]);
  return buf;
}

// This function is called when a name is resolved via MDNS/Bonjour. We set
// this up in the setup() function above. The name you give to this callback
// function does not matter at all, but it must take exactly these arguments
// as below.
// If a service is discovered, name, ipAddr, port and (if available) txtContent
// will be set.
// If your specified discovery timeout is reached, the function will be called
// with name (and all successive arguments) being set to NULL.
void serviceFound(const char* type, MDNSServiceProtocol proto,
                  const char* name, const byte ipAddr[4],
                  unsigned short port,
                  const char* txtContent)
{
  if (NULL == name) {
	Serial.print("Finished discovering services of type ");
	Serial.println(type);
  } else {
    Serial.print("Found: '");
    Serial.print(name);
    Serial.print("' at ");
    Serial.print(ip_to_str(ipAddr));
    Serial.print(", port ");
    Serial.print(port);
    Serial.println(" (TCP)");
    switcherIP[0] = ipAddr[0];
    switcherIP[1] = ipAddr[1];
    switcherIP[2] = ipAddr[2];
    switcherIP[3] = ipAddr[3];

    // Check out http://www.zeroconf.org/Rendezvous/txtrecords.html for a
    // primer on the structure of TXT records. Note that the Bonjour
    // library will always return the txt content as a zero-terminated
    // string, even if the specification does not require this.
    if (txtContent) {
      Serial.print("\ttxt record: ");
      
      char buf[256];
      char len = *txtContent++, i=0;;
      while (len) {
        i = 0;
        while (len--)
          buf[i++] = *txtContent++;
        buf[i] = '\0';
        Serial.print(buf);
        len = *txtContent++;
        
        if (len)
          Serial.print(", ");
        else
          Serial.println();
      }
    }
  }
}

void mdnsSetup() {
  EthernetBonjour.begin("ESPTallyLight");
  EthernetBonjour.setServiceFoundCallback(serviceFound);

    // You can use the "isDiscoveringService()" function to find out whether the
  // Bonjour library is currently discovering service instances.
  // If so, we skip this input, since we want our previous request to continue.
  if (!EthernetBonjour.isDiscoveringService()) {
    byte ipAddr[4];

    Serial.print("Discovering services of type '");
    Serial.print("blackmagic");
    Serial.println("' via Multi-Cast DNS (Bonjour)...");

    // Now we tell the Bonjour library to discover the service. Below, I have
    // hardcoded the TCP protocol, but you can also specify to discover UDP
    // services.
    // The last argument is a duration (in milliseconds) for which we will
    // search (specify 0 to run the discovery indefinitely).
    // Note that the library will resend the discovery message every 10
    // seconds, so if you search for longer than that, you will receive
    // duplicate instances.

    EthernetBonjour.startDiscoveringService("_blackmagic", MDNSServiceTCP, 5000);
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  FastLED.addLeds<NEOPIXEL, DATA_PIN1>(leds, NUM_LEDS);  // RGB ordering is assumed
  //SPI.begin(SCK, MISO, MOSI, -1);
  leds[0] = CRGB::Orange;
  FastLED.show();
  
  Ethernet.init(CS);

    // start the Ethernet connection:
  Serial.println("Trying to get an IP address using DHCP");
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
    // Check for Ethernet hardware present
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      Serial.println("Seems like I'm a light");
      Server = false;
    }
    else {
      Serial.println("Seems like I'm the server");
      Server = true;
    }
    if (Ethernet.linkStatus() == LinkOFF) {
      Serial.println("Ethernet cable is not connected.");
    }
    // initialize the Ethernet device not using DHCP:
    Ethernet.begin(mac, ip, myDns, gateway, subnet);
  }
  if(Server) {
    // print your local IP address:
    Serial.print("My IP address: ");
    Serial.println(Ethernet.localIP());
  }
   // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);
  // Add peers
  // register peer
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  if(Server) {
    for(int i = 1; i < NUM_PEERS; ++i)
    {
      memcpy(peerInfo.peer_addr, MACs[i], 6);
      if (esp_now_add_peer(&peerInfo) != ESP_OK){
        Serial.print("Failed to add peer: ");
        Serial.println(i);
        //return;
      }
    }

    myData.state = Searching;
    mdnsSetup();
  }
  else {
        // MAC comparison to identify light number
    Serial.println(WiFi.macAddress());
    sscanf(WiFi.macAddress().c_str(),"%2hhx:%2hhx:%2hhx:%2hhx:%2hhx:%2hhx",&myMac[0],&myMac[1],&myMac[2],&myMac[3],&myMac[4],&myMac[5]);  
 
    for(int i=1;i<NUM_PEERS; ++i)
    {
      if(
        myMac[0] == MACs[i][0]
        && myMac[1] == MACs[i][1]
        && myMac[2] == MACs[i][2]
        && myMac[3] == MACs[i][3]
        && myMac[4] == MACs[i][4]
        && myMac[5] == MACs[i][5]){
        LightNumber = i;
        Serial.print("I am light number ");
        Serial.println(LightNumber);
      }
    }

    memcpy(peerInfo.peer_addr, MACs[0], 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.println("Failed to contact server");
      return;
    }
  }
}

void loop() {
  if(Server)
  {
    // put your main code here, to run repeatedly:
    auto link = Ethernet.linkStatus();
    //Serial.print("Link status: ");
    switch (link) {
      case Unknown:
        //Serial.println("Unknown");
        leds[0] = CRGB::Blue;
        break;
      case LinkON:
        //Serial.println("ON");
        //leds[0] = CRGB::Lime;
        leds[0] = CRGB::Blue;
        break;
      case LinkOFF:
        //Serial.println("OFF");
        leds[0] = CRGB::Red;
        break;
    }
    
    //Serial.println(ip_to_str(switcherIP));
    if(myData.state == Searching) {
      if(switcherIP[3] == 0) {
        Serial.println("Searching...");
        EthernetBonjour.run();
        myData.state = Searching;
        delay(1000);
      }
      else {
        // Initialize a connection to the switcher:
        if (firstRun) {
            atemSwitcher.begin(switcherIP);
            //atemSwitcher.serialOutput(0xFF); //Makes Atem library print debug info
            Serial.println("------------------------");
            Serial.println("Connecting to switcher...");
            Serial.println((String)"Switcher IP:         " + switcherIP[0] + "." + switcherIP[1] + "." + switcherIP[2] + "." + switcherIP[3]);
            firstRun = false;
        }
        atemSwitcher.runLoop();
        if (atemSwitcher.isConnected()) {
            //changeState(STATE_RUNNING);
            myData.state = Connected;
            Serial.println("Connected to switcher");
        }
        delay(1000);
      }
    }
    if(myData.state == Connected) {
      atemSwitcher.runLoop();
      int tallySources = atemSwitcher.getTallyByIndexSources();
      //Serial.print(tallySources);
      leds[0] = CRGB::Lime;
      
      /*
      for(int i=0;i<NUM_PEERS;++i) {
        //Serial.printf("%d: ", i);
        uint8_t tallyFlag = atemSwitcher.getTallyByIndexTallyFlags(i);
        if (tallyFlag & TALLY_FLAG_PROGRAM) {
          myData.program = i;
          Serial.print("Programm: ");
          //Serial.print(i);
        }
        else if (tallyFlag & TALLY_FLAG_PREVIEW) {
          myData.preview = i;
          Serial.print(" - Preview: ");
          //Serial.print(i);
        }
      }
      */
      
      program = atemSwitcher.getProgramInput();
      preview = atemSwitcher.getPreviewInput();
      //Serial.printf("Program: %d - Preview: %d", myData.program, myData.preview);
      //Serial.println();
      if (!atemSwitcher.isConnected()) { // will return false if the connection was lost
        Serial.println("------------------------");
        Serial.println("Connection to Switcher lost...");
        myData.state = Searching;

        //Reset tally server's tally flags, so clients turn off their lights.
        //tallyServer.resetTallyFlags();
      }
      //Serial.println();

      if(program != myData.program || preview != myData.preview || sendAgain==SEND_INTERVAL) {
        myData.preview = preview;
        myData.program = program;
        for( int i=1;i<NUM_PEERS;++i){
          //Serial.println(i);
          esp_err_t result = esp_now_send(MACs[i], (uint8_t *) &myData, sizeof(myData));
          delay(10);
        }
        sendAgain = 0;
      }
      ++sendAgain;
    }
  }
  else {
    if(!SentOK || ServerError) {
      //if(leds[0]) {
      //  leds[0] = CRGB::Black;
      //}
      //else {
        leds[0] = CRGB::Blue;
      //}
    }
    else {
      leds[0] = CRGB::Black;
      if(myData.state == Connected) {
        ServerError = false;
        if(myData.preview == LightNumber) {
          leds[0] = CRGB::Lime;
        }
        if(myData.program == LightNumber) {
          leds[0] = CRGB::Red;
        }
      }
      else if(myData.state == Searching) {
        leds[0] = CRGB::Blue;
        ServerError = false;
      }
      else if(myData.state == Error) {
        ServerError = true;
      }
    }
    ++sendAgain;
    if(sendAgain==SEND_INTERVAL)
    {
      esp_now_send(MACs[0], (uint8_t *) &myData, sizeof(myData));
      sendAgain = 0;
    }
  }
  delay(DELAY);
  FastLED.show();
}
