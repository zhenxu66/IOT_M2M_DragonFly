/** Dragonfly Cellular SMS Example
 * Configures the cellular radio, sends a SMS message to the configured number, and displays any received messages.
 *
 * NOTE: This example changes the baud rate of the debug port to 115200 baud!
 */

#include "mbed.h"
#include "mtsas.h"
#include <string>
#include <vector>
#include <iterator>

bool init_mtsas();

// The MTSSerialFlowControl object represents the physical serial link between the processor and the cellular radio.
mts::MTSSerialFlowControl* io;
// The Cellular object represents the cellular radio.
mts::Cellular* radio;

// An APN is required for GSM radios.
static const char apn[] = "";

// A valid phone number must be configured in order to successfully send SMS messages.
// The phone number must have the 1 in front of it (11 digits total).
static std::string phone_number = "1xxxxxxxxxx";

bool radio_ok = false;

int main() {
    // Change the baud rate of the debug port from the default 9600 to 115200.
    Serial debug(USBTX, USBRX);
    debug.baud(115200);
    
    //Sets the log level to INFO, higher log levels produce more log output.
    //Possible levels: NONE, FATAL, ERROR, WARNING, INFO, DEBUG, TRACE
    mts::MTSLog::setLogLevel(mts::MTSLog::INFO_LEVEL);
    
    logInfo("initializing cellular radio");
    radio_ok = init_mtsas();
    if (! radio_ok) {
        while (true) {
            logError("failed to initialize cellular radio");
            wait(1);
        }
    }
    
    logInfo("setting APN");
    if (radio->setApn(apn) != MTS_SUCCESS)
        logError("failed to set APN to \"%s\"", apn);
        
    logInfo("sending SMS to %s", phone_number.c_str());
    mts::Cellular::Sms msg;
    msg.phoneNumber = phone_number;
    msg.message = "Hello from MultiTech Dragonfly!";
    if (radio->sendSMS(msg) != MTS_SUCCESS)
        logError("sending SMS failed");
        
    // Display any received SMS messages.
    while (true) {
        std::vector<mts::Cellular::Sms> msgs = radio->getReceivedSms();
        for (std::vector<mts::Cellular::Sms>::iterator it = msgs.begin(); it != msgs.end(); it++) {
            logInfo("[%s][%s]\r\n%s\r\n", it->phoneNumber.c_str(), it->timestamp.c_str(), it->message.c_str());
        }
        
        radio->deleteOnlyReceivedReadSms();
        
        wait(5);
    }
    
    return 0;
}

bool init_mtsas() {
    io = new mts::MTSSerialFlowControl(RADIO_TX, RADIO_RX, RADIO_RTS, RADIO_CTS);
    if (! io)
        return false;
        
    // radio default baud rate is 115200
    io->baud(115200);
    radio = mts::CellularFactory::create(io);
    if (! radio)
        return false;
        
    // Transport must be set properly before any TCPSocketConnection or UDPSocket objects are created
    Transport::setTransport(radio);
    
    return true;
}
