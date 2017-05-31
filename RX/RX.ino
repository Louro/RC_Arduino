

// Written by Nick Gammon
// February 2011
/**
 * Send arbitrary number of bits at whatever clock rate (tested at 500 KHZ and 500 HZ).
 * This script will capture the SPI bytes, when a '\n' is recieved it will then output
 * the captured byte stream via the serial.
 */
#include <avr/interrupt.h>
#include <SPI.h>
//#include <FlexiTimer2.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "a7105.h"
#include "a7105_tx_rx.h"

// If using software SPI (the default case):
#define OLED_MOSI   9
#define OLED_CLK   10
#define OLED_DC    11
#define OLED_CS    12
#define OLED_RESET 13
//Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2


#define GPIO1_PIN 3			// A7105 GPIO1 pin connected to arduino, same pin that will generate interrupt
#define GPIO1_INT 1			// Interrupt number, for pin 3 is 1 http://arduino.cc/en/Reference/attachInterrupt

#define LED_PIN 7			// Led pin
#define SPK_PIN 7			// Speaker pin

#define DBG1_PIN 7			// Debug pin 1
#define DBG2_PIN 7			// Debug pin 2

#define SRV_PIN 2			// Servo out pin

#define delay_uS(...) delayMicroseconds(__VA_ARGS__)
#define delay_mS(...) delay(__VA_ARGS__)

//uint8_t rx_channels[16] = { 0x0a,0x69,0x19,0x7d,0x2d,0x55,0x3c,0x73,0x23,0x87,0x37,0x91,0x41,0x32,0x4b,0x5f};
uint8_t rx_channels[16] = { 0x10,0x6a,0x1a,0x7e,0x2e,0x56,0x3d,0x74,0x24,0x88,0x38,0x92,0x42,0x33,0x4c,0x60};
//                                      x              x             x               x              x    x
//uint8_t rx_channels[16] = { 0x11,0x6b,0x1b,0x7f,0x2f,0x57,0x3e,0x75,0x25,0x89,0x39,0x93,0x43,0x34,0x4e,0x61};

uint8_t tx_channels[16] = { 0x11,0x6b,0x1b,0x7f,0x2f,0x57,0x3e,0x75,0x25,0x89,0x39,0x93,0x43,0x34,0x4e,0x61};


unsigned int curchannel =0;



int flysky_init();
void beep(unsigned char speakerPin, int frequencyInHertz, long timeInMilliseconds);
void process_function();
void print_info();
void print_hex(uint8_t value);
void print_dec(uint16_t value);
void A7105_FrameDetected();


unsigned int sigdet =0;
unsigned int scan =1;



//#if (SSD1306_LCDHEIGHT != 64)
//#error("Height incorrect, please fix Adafruit_SSD1306.h!");
//#endif

void drawLCD();

void setup (void)
{
	Serial.begin (115200);   // debugging


 // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  //display.begin(SSD1306_SWITCHCAPVCC);
  // init done

  pinMode(8, INPUT);
  pinMode(9, INPUT);
  pinMode(10, INPUT);
  pinMode(11, INPUT);
  pinMode(12, INPUT);


	/************** PPM READER *********/  
	//pinMode(ppm_pin, INPUT);
	//attachInterrupt(ppm_pin - 2, read_ppm, FALLING);

	/************** TIMER 1 *********/  
//	TCCR1A = 0;  //reset timer1
//	TCCR1B = 0;
//	TCCR1B |= (1 << CS11);  //set timer1 to increment every 0,5 us


	/************** SPEAKER *********/ 
	//pinMode(SPK_PIN, OUTPUT);
	//beep(SPK_PIN,4093,50);

	/************** LED & DEBUG *********/ 
	pinMode(LED_PIN, OUTPUT); 
	pinMode(DBG1_PIN, OUTPUT);  
	pinMode(DBG2_PIN, OUTPUT);  

	/************* OTHER STUFF *****************/

	

  Serial.println("Booting...");
}  // end of setup
 

 
 
 int flysky_init()
{

    u8 if_calibration1;
    u8 vco_calibration0;
    u8 vco_calibration1;
    //u8 vco_current;
   Serial.println("flysky_init");

	A7105_Reset();
  A7105_WriteID(0x5475C52A);
	
	A7105_WriteReg(A7105_01_MODE_CONTROL, 0x42  ); //8A
	A7105_WriteReg(A7105_02_CALC, 0x00  );
	A7105_WriteReg(A7105_03_FIFOI, 0x14  );  //0x80
	A7105_WriteReg(A7105_04_FIFOII, 0x00  );//80
	A7105_WriteReg(A7105_07_RC_OSC_I, 0x00  );
	A7105_WriteReg(A7105_08_RC_OSC_II, 0x00  );
	A7105_WriteReg(A7105_09_RC_OSC_III, 0x00	);
	A7105_WriteReg(A7105_0A_CK0_PIN, 0x00  );
	A7105_WriteReg(A7105_0B_GPIO1_PIN1, 0x01  ); //0x19//0x01
	A7105_WriteReg(A7105_0C_GPIO2_PIN_II, 0x21  ); //0x05
	A7105_WriteReg(A7105_0D_CLOCK, 0x05  );
	A7105_WriteReg(A7105_0E_DATA_RATE, 0x00  );
	A7105_WriteReg(A7105_0F_PLL_I, 0x50  );
	A7105_WriteReg(A7105_10_PLL_II, 0x9E  );
	A7105_WriteReg(A7105_11_PLL_III, 0x4B  );
	A7105_WriteReg(A7105_12_PLL_IV, 0x00  );
	A7105_WriteReg(A7105_13_PLL_V, 0x02  );
	A7105_WriteReg(A7105_14_TX_I, 0x16  );
	A7105_WriteReg(A7105_15_TX_II, 0x2B  );
	A7105_WriteReg(A7105_16_DELAY_I, 0x12  );
	A7105_WriteReg(A7105_17_DELAY_II, 0x00  );
	A7105_WriteReg(A7105_18_RX, 0x62  );
	A7105_WriteReg(A7105_19_RX_GAIN_I, 0x80  );
	A7105_WriteReg(A7105_1A_RX_GAIN_II, 0x80  );
	A7105_WriteReg(A7105_1B_RX_GAIN_III, 0x00  );
	A7105_WriteReg(A7105_1C_RX_GAIN_IV, 0x0A  );
	A7105_WriteReg(A7105_1D_RSSI_THOLD, 0x32  );
	A7105_WriteReg(A7105_1E_ADC, 0xC3  );
	A7105_WriteReg(A7105_1F_CODE_I, 0x0F  );
	A7105_WriteReg(A7105_20_CODE_II, 0x13  );
	A7105_WriteReg(A7105_21_CODE_III, 0xC3  );
	A7105_WriteReg(A7105_22_IF_CALIB_I, 0x00  );
//	A7105_WriteReg(A7105_23_IF_CALIB_II, 0x00  );
	A7105_WriteReg(A7105_24_VCO_CURCAL, 0x00  );
	A7105_WriteReg(A7105_25_VCO_SBCAL_I, 0x00  );
	A7105_WriteReg(A7105_26_VCO_SBCAL_II, 0x3B  );
	A7105_WriteReg(A7105_27_BATTERY_DET, 0x00  );
	A7105_WriteReg(A7105_28_TX_TEST, 0x17  );
	A7105_WriteReg(A7105_29_RX_DEM_TEST_I, 0x47  );
	A7105_WriteReg(A7105_2A_RX_DEM_TEST_II, 0x80  );
	A7105_WriteReg(A7105_2B_CPC, 0x03  );
	A7105_WriteReg(A7105_2C_XTAL_TEST, 0x01  );
	A7105_WriteReg(A7105_2D_PLL_TEST, 0x45  );
	A7105_WriteReg(A7105_2E_VCO_TEST_I, 0x18  );
	A7105_WriteReg(A7105_2F_VCO_TEST_II, 0x00  );
	A7105_WriteReg(A7105_30_IFAT, 0x01  );
	A7105_WriteReg(A7105_31_RSCALE, 0x0F  );
	//A7105_WriteReg(A7105_32_FILTER_TEST, 0x  );
	
	
    A7105_Strobe(A7105_STANDBY);

    //IF Filter Bank Calibration
    A7105_WriteReg(0x02, 1);
    //vco_current =
    A7105_ReadReg(0x02);
    unsigned long ms = millis();

    while(millis()  - ms < 500) {
        if(! A7105_ReadReg(0x02))
            break;
    }
    if (millis() - ms >= 500)
        return 0;
	
    if_calibration1 = A7105_ReadReg(A7105_22_IF_CALIB_I);
    if(if_calibration1 & A7105_MASK_FBCF) {
        //Calibration failed...what do we do?
        return 0;
    }
	
	//VCO Current Calibration
  A7105_WriteReg(0x24, 0x13); //Recomended calibration from A7105 Datasheet
    
	A7105_WriteReg(0x25, 0x09);
	
	 //VCO Bank Calibration
    //A7105_WriteReg(0x26, 0x3b); //Recomended limits from A7105 Datasheet
	
	/*

    //VCO Bank Calibrate channel 0?
    //Set Channel
    A7105_WriteReg(A7105_0F_CHANNEL, 0x00);
    //VCO Calibration
    A7105_WriteReg(0x02, 2);
    ms = millis();
    while(millis()  - ms < 500) {
        if(! A7105_ReadReg(0x02))
            break;
    }
    if (millis() - ms >= 500)
        return 0;
    vco_calibration0 = A7105_ReadReg(A7105_25_VCO_SBCAL_I);
    if (vco_calibration0 & A7105_MASK_VBCF) {
        // Calibration failed...what do we do?
       return 0;
    }

    //Calibrate channel 0xa0?
    //Set Channel
    A7105_WriteReg(A7105_0F_CHANNEL, 0xa0);
    //VCO Calibration
    A7105_WriteReg(A7105_02_CALC, 2);
    ms = millis();
    while(millis()  - ms < 500) {
        if(! A7105_ReadReg(A7105_02_CALC))
            break;
    }
    if (millis() - ms >= 500)
        return 0;
    vco_calibration1 = A7105_ReadReg(A7105_25_VCO_SBCAL_I);
    if (vco_calibration1 & A7105_MASK_VBCF) {
        //Calibration failed...what do we do?
    }
*/
   // Reset VCO Band calibration
    //A7105_WriteReg(0x25, 0x08);

	//A7105_SetPower(TXPOWER_150mW);

	delay_uS(5000);
    return 1;
}



unsigned int servo_value = 1500;
unsigned int servo_value_old = 1500;

unsigned long loopTime = 0;


unsigned long ms =0;
unsigned long mics =0;
unsigned int i =0;



unsigned char RADIO_STA = RADIO_INIT;
unsigned char RADIO_IGN_INT = 0;
unsigned char RADIO_FRA_DET = 0;
unsigned char RADIO_NEW_DATA = 0;
unsigned char RADIO_crc =0;
unsigned char RADIO_data[20];
unsigned long cnt = 0;

float value = 0;
unsigned int dir = 1;

static uint8_t nunchuck_buf[6];   // array to store nunchuck data,


// main loop - wait for flag set in interrupt routine
void loop (void)
{
	
	RadioLink();

	if (RADIO_NEW_DATA) {
		if (cnt >= 10) {
			for (int i = 0; i < 11; i++)
			{
				Serial.print(RADIO_data[i], HEX);
				Serial.print(" ");
			}
			Serial.println();

			cnt = 0;

		}
		else
			cnt++;
		RADIO_NEW_DATA = 0;
	}

} // end of loop



void init_pcint();

void RadioLink(){
	
	switch(RADIO_STA){
		
		case RADIO_INIT: 
	
			/************** A7105 SETUP + INIT *********/ 
			A7105_Setup();
			pinMode(GPIO1_PIN, INPUT); 
			attachInterrupt(digitalPinToInterrupt(GPIO1_PIN), A7105_FrameDetected, FALLING); 
    


			while(1) {
				if (flysky_init()){
					//beep(SPK_PIN,4093,50);
					break;
				}
				//beep(SPK_PIN,4093,50);
				delay_mS(500);
			}
			RADIO_STA = RADIO_BIND;
			break;
		
		case RADIO_BIND:
		
			/************** FLYSKY BIND - CHANGE CHANNEL *********/
			//servo_value = 1500;
			
			RADIO_IGN_INT = 1;			
			A7105_Strobe(A7105_STANDBY);
			delay_uS(10);
			A7105_Strobe(A7105_RST_RDPTR);

    
			A7105_WriteReg(A7105_0F_PLL_I, rx_channels[curchannel++]);
      if(curchannel>=16) 
        curchannel = 0;  

			A7105_Strobe(A7105_RX);
			RADIO_IGN_INT = 0;
			
			RADIO_STA = RADIO_RX_WAIT;
			mics = micros();
			break;
			
		case RADIO_RX_WAIT:		
		  
			/************** FLYSKY RX WAIT - WAIT FOR SIGNAL *********/		
			if(RADIO_FRA_DET){
				delayMicroseconds(200);
				RADIO_STA = RADIO_RX;
				break;
			}
			if(micros() - mics >= 1600) {
				RADIO_STA = RADIO_BIND;
				break;
			}
			break;
		
		case RADIO_RX:
		  
			/************** FLYSKY FRAME RECEIVED *********/
			RADIO_FRA_DET = 0;
			RADIO_crc = A7105_ReadReg(0x00);
				
			if(!(RADIO_crc & A7105_MASK_CRC)){

				A7105_ReadData(RADIO_data, 11);

				RADIO_NEW_DATA = 1;
        
				//servo_value_old = servo_value;
				//servo_value = (((RADIO_data[8]<<8)&0xFF00) + RADIO_data[7]);
				//servo_value = (unsigned int)(((servo_value*10*0.05) + (servo_value_old*10*0.95))/10);
				
			}
			RADIO_IGN_INT = 1;
			A7105_Strobe(A7105_STANDBY);
			A7105_Strobe(A7105_RST_RDPTR);
			A7105_WriteReg(A7105_0F_PLL_I, rx_channels[curchannel++]);
			if(curchannel>=16)
				curchannel=0;
			A7105_Strobe(A7105_RX);
			RADIO_IGN_INT = 0;
				
			RADIO_STA = RADIO_RX_WAIT;
      digitalWrite(DBG1_PIN,LOW);
      EIFR = 0xFF;
     // interrupts();
      attachInterrupt(digitalPinToInterrupt(GPIO1_PIN), A7105_FrameDetected, FALLING);
      //sei();
			mics = micros();
			break;
			
		case RADIO_IDLE: break;



			
	}
}


unsigned char a = 0;

void A7105_FrameDetected(){
  detachInterrupt(digitalPinToInterrupt(GPIO1_PIN)); 

  //cli();
  //PCIFR = 0x00;

  digitalWrite(DBG1_PIN,HIGH);
//	if(!RADIO_IGN_INT){
		RADIO_FRA_DET = 1;
//  }
}


void beep(unsigned char speakerPin, int frequencyInHertz, long timeInMilliseconds)     // the sound producing function
{	 
          int x;	 
          long delayAmount = (long)(1000000/frequencyInHertz);
          long loopTime = (long)((timeInMilliseconds*1000)/(delayAmount*2));
          for (x=0;x<loopTime;x++)	 
          {	 
              digitalWrite(speakerPin,HIGH);
              delayMicroseconds(delayAmount);
              digitalWrite(speakerPin,LOW);
              delayMicroseconds(delayAmount);
          }	 
}


