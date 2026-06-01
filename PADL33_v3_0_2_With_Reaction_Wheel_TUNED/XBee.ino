/************************************************
      PADL-33 XBee Set Up Code
      Created by: Ashton Posey
************************************************/
//Purpose: General functions to run the XBee connected on the back of the PCB with 1mm headers
//         Must be enabled in variables.h -> bool usingXBee

////////////////////////////////////// XBee Set Up //////////////////////////////////////
// void xbeeSetUp();
// bool enterATmode();
// bool exitATmode();
// String atCommand(String command);

void xbeeSetUp(){
    if (usingXBee){
        XBee.begin(XBEE_BAUD);      // Open the Serial line to the XBee.
        XBee.setTimeout(1000);

        // To make sure the XBee is connected, the red and green leds will flash at the same time on a successful start up.
        enterATmode();
        atCommand("ATRE"); // Reset the configuration on your XBee, this means if your xbee is not configured correctly in any way, 
                          // it doesn't care. XTCU is only needed to update the firmware. YOUR XBEES MUST HAVE THE SAME FIRMWARE (preferably up to date)
        atCommand("ATCH" + channelID);  // Chanel of the xbee, similar to a TV it won't even see XBees on another channel.
        atCommand("ATID" + networkID);  // Can have multiple networks on a channel, but it's not ideal.
        atCommand("ATSC" + scanChID);   // Needs to be the same on different XBees but never needs to be changed. 
        atCommand("ATDL1");             // Configure XBee as a payload   DL=1   MY=0
        atCommand("ATMY0");             // Can have multiple paylaods that are polled for data.
        exitATmode();

        XBee.setTimeout(5);  // SUPER IMPORTANT: The XBee runs off of Serial, serial has a time out
                            // meaning it will wait a default time of 1 second after an ex. readString() or readStringUntil("\n") w\o recieving a "\n"
                            // is recieved. This needs to be changed to a barely noticable 5ms so
                            // that any recieved data will 

        Serial.println("Payload Mode active");
    }
}

// XBee Configuration Functions
bool enterATmode() {
  String response;
  for (byte i = 0; response.equals("") && (i < 10); i++) {
      XBee.print("+++");
      Serial.println("+++"); // Printed to serial monitor for error correction.
      delay(100);
      response = XBee.readStringUntil('\r');
  }
  return response.equals("OK");
}

bool exitATmode() {
  String response = atCommand("ATWR");
  atCommand("ATCN");
  return response.equals("OK");
}

String atCommand(String command) {
  XBee.print(command);
  XBee.write('\r');
  Serial.println(command); // Printed to serial monitor for error correction.
  Serial.write('\r');
  return XBee.readStringUntil('\r');
}