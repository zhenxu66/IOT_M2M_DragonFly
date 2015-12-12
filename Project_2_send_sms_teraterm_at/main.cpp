#include "mbed.h"

int main()
{
    // external serial port
  /* Serial ext(USBTX,USBRX ); */   //AT commands via USB port on Dragonfly  
  /* Serial ext(dbgTX,dbgRX); */     //AT commands via debug port (same as below but bad reference text
   Serial ext(PB_6, PB_7);           //AT commands via debug port that we use for jtag programming also.
  /* Serial ext(PA_2, PA_3);  */     //AT commands not sure where they go yet.  

    // internal serial port to radio
    Serial radio(RADIO_TX, RADIO_RX);  

    ext.baud(115200);   // can be changed to a higher rate, Windows is ok with 115200
    radio.baud(115200);  // already configured in multitech device

    while (true) {          // push in data to In radio
        if (ext.readable())
            radio.putc(ext.getc());
        if (radio.readable())          //push jout data from radio to out radio
            ext.putc(radio.getc());
    }
}


/* 
    Enter command in Teraterm
    Below are the AT commands that will Foot notes for class
    NOTE: BACKSPACE will cause the instructions to FAIL.  Just hit enter and re-type 
    
at                      [if working, command responds with ¡°OK¡±]
at+cpin?            [Check SIM is detected] READY
at+cgdcont=1,¡±IP¡±,¡±xxxxxxxxxxxxxxxxxx¡±   [Enter the carrier SIM APN into modem]  "m2m.com.attz" or "broadband with ATT sim card"
at+cgdcont?       [verify APN is correct]
at+csq               [Check signal strength >10, but 8 will still work] 
at+creg?            [Check for successful network registration] +CREG:0,1 or +CREG:0,5
at+cnmi=2,2,0,1,0   [Configure to route received SMS text direct to serial interface]
at+cmgf=1         [text mode for SMS]
at+cmgs=¡±1##########¡±<cr>  [enter phone number # to send SMS text message] "US phone it must be 1##########"
     >type sms message at greater than prompt<Control-Z> to send 
  Verify SMS is received. On phone that received SMS, respond by sending SMS back to modem number.


AT+creg? Responses
-------------------------+CREG: 0,2 not registered to a network but looking   (device hasn¡¯t established a link to the carrier yet)
+CREG: 0,1 registered on the home network
+CREG: 0,5 registered but roaming  
+CREG: 0,3 unknown (bad)
+CREG: 0,4 unknown (bad)
+CREG: 0,0 unregistered and not looking  (sim isn¡¯t: installed, inplace correctly , functional, active) 

AT+csq:  >10 is better but it will function at lower levels.
Reference Signal Levels
 0 = -113 dBm, 1 = -111 dBm, 2 = -109 dBm, 3 = -107 dBm, 4 = -105 dBm
 5 = -103 dBm, 6 = -101 dBm, 7 = -99 dBm,   8 = -97 dBm,  9 = -95 dBm
10 = -93 dBm, 11 = -91 dBm, 12 = -89 dBm, 13 = -87 dBm, 14 = -85 dBm
15 = -83 dBm, 16 = -81 dBm, 17 = -79 dBm, 18 = -77 dBm, 19 = -75 dBm
20 = -73 dBm, 21 = -71 dBm, 22 = -69 dBm, 23 = -67 dBm, 24 = -65 dBm
25 = -63 dBm, 26 = -61 dBm, 27 = -59 dBm, 28 = -57 dBm, 29 = -55 dBm
30 = -53 dBm, 31 = -51 dBm
*/
