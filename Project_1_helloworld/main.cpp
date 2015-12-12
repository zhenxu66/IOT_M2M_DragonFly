#include "mbed.h"


/* Simple Hello World program that outputs "Hello World!" every 
    five seconds to the serial debug port, and blinks at the user
    defined hertz
    In Teraterm, set Serial Baud rate:9600
*/

// LED blink rate: higher -> faster blinking
#define LED_BLINK_RATE 8  //Hertz

//Define the LED pin output
//DigitalOut data00(D0); 
DigitalOut data00(PA_3);     //Blinds D0 LED
//DigitalOut data01(D1);
DigitalOut data01(PA_2);     //Blinds D1 LED
//DigitalOut data02(D2);
//DigitalOut data02(PB_15);    //Blinds D2 LED
//DigitalOut data03(D3);
DigitalOut data03(PA_0);     //Blinds D3 LED
//DigitalOut data04(D4);
DigitalOut data04(PA_7);     //Blinds D4 LED
//DigitalOut data05(D5);
DigitalOut data05(PA_9);     //Blinds D5 LED
//DigitalOut data06(D6);
DigitalOut data06(PA_1);     //Blinds D6 LED
//DigitalOut data07(D7);
DigitalOut data07(PA_8);     //Blinds D7 LED
//DigitalOut data08(D8); 
DigitalOut data08(PB_1);     //Blinds D8 LED
int temp;

//Define timers
Timer print_timer;
Timer led_timer;

int main() {
    data00 = 1;                            //Initialize LED off
    data01 = 0;                            //Initialize LED on
//    data02 = 1;                            //Initialize LED off
    data03 = 1;                            //Initialize LED off
    data04 = 1;                            //Initialize LED off
    data05 = 1;                            //Initialize LED off
    data06 = 1;                            //Initialize LED off
    data07 = 1;                            //Initialize LED off
    data08 = 1;                            //Initialize LED off
    print_timer.start();                //Start timers, will count until stopped
    led_timer.start();
    
    while (1) {
        if (print_timer.read() >= 5) {  //print_timer.read() returns time in seconds
            printf("Hello World!\n");
            print_timer.reset();        //Resets timer count to 0
        }
        
        //Calculates interval needed for specified frequency
        if ( led_timer.read_ms() >= (2000.0/(2*LED_BLINK_RATE))) {     
            temp   =  data01;                 //Invert LED output
            data01 =  data00;                 //Invert LED output
            data00 =  data03;                 //Invert LED output
            data03 =  data06;                 //Invert LED output
            data06 =  data08;                 //Invert LED output
            data08 =  data05;                 //Invert LED output
            data05 =  data04;                 //Invert LED output
            data04 =  data07;                 //Invert LED output
            data07 =  temp;                   //Invert LED output
            led_timer.reset();          //Resets timer count to 0
        }
    }
}


