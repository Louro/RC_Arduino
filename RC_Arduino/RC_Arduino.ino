/*
 Name:		RC_Arduino.ino
 Created:	5/30/2017 6:01:40 PM
 Author:	pjc
*/


#include <SPI.h>
//#include <FlexiTimer2.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "a7105.h"
#include "a7105_tx.h"

// If using software SPI (the default case):
#define OLED_MOSI   9
#define OLED_CLK   10
#define OLED_DC    11
#define OLED_CS    12
#define OLED_RESET 13
Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2


#define GPIO1_PIN 3			// A7105 GPIO1 pin connected to arduino, same pin that will generate interrupt
#define GPIO1_INT 1			// Interrupt number, for pin 3 is 1 http://arduino.cc/en/Reference/attachInterrupt

#define LED_PIN 8			// Led pin
#define SPK_PIN 8			// Speaker pin

#define DBG1_PIN 8			// Debug pin 1
#define DBG2_PIN 8			// Debug pin 2

#define SRV_PIN 2			// Servo out pin

#define delay_uS(...) delayMicroseconds(__VA_ARGS__)
#define delay_mS(...) delay(__VA_ARGS__)

//uint8_t rx_channels[16] = { 0x0a,0x69,0x19,0x7d,0x2d,0x55,0x3c,0x73,0x23,0x87,0x37,0x91,0x41,0x32,0x4b,0x5f};
uint8_t rx_channels[16] = { 0x10,0x6a,0x1a,0x7e,0x2e,0x56,0x3d,0x74,0x24,0x88,0x38,0x92,0x42,0x33,0x4c,0x60 };
//                                      x              x             x               x              x    x
//uint8_t rx_channels[16] = { 0x11,0x6b,0x1b,0x7f,0x2f,0x57,0x3e,0x75,0x25,0x89,0x39,0x93,0x43,0x34,0x4e,0x61};

uint8_t tx_channels[16] = { 0x11,0x6b,0x1b,0x7f,0x2f,0x57,0x3e,0x75,0x25,0x89,0x39,0x93,0x43,0x34,0x4e,0x61 };


unsigned int curchannel = 0;



int flysky_init();
void beep(unsigned char speakerPin, int frequencyInHertz, long timeInMilliseconds);
void process_function();
void print_info();
void print_hex(uint8_t value);
void print_dec(uint16_t value);
void A7105_FrameDetected();


unsigned int sigdet = 0;
unsigned int scan = 1;

Servo myservo;


#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

void drawLCD();

void setup(void)
{
	Serial.begin(115200);   // debugging


							// by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
	display.begin(SSD1306_SWITCHCAPVCC);
	// init done

	/************** PPM READER *********/
	//pinMode(ppm_pin, INPUT);
	//attachInterrupt(ppm_pin - 2, read_ppm, FALLING);

	/************** TIMER 1 *********/
	//	TCCR1A = 0;  //reset timer1
	//	TCCR1B = 0;
	//	TCCR1B |= (1 << CS11);  //set timer1 to increment every 0,5 us


	/************** SPEAKER *********/
	pinMode(SPK_PIN, OUTPUT);
	beep(SPK_PIN, 4093, 50);

	/************** LED & DEBUG *********/
	pinMode(LED_PIN, OUTPUT);
	pinMode(DBG1_PIN, OUTPUT);
	pinMode(DBG2_PIN, OUTPUT);

	/************* OTHER STUFF *****************/

	pinMode(SRV_PIN, OUTPUT);
	myservo.attach(2);

	nunchuck_init(); // send the initilization handshake

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

	A7105_WriteReg(A7105_01_MODE_CONTROL, 0x42); //8A
	A7105_WriteReg(A7105_02_CALC, 0x00);
	A7105_WriteReg(A7105_03_FIFOI, 0x14);  //0x80
	A7105_WriteReg(A7105_04_FIFOII, 0x00);//80
	A7105_WriteReg(A7105_07_RC_OSC_I, 0x00);
	A7105_WriteReg(A7105_08_RC_OSC_II, 0x00);
	A7105_WriteReg(A7105_09_RC_OSC_III, 0x00);
	A7105_WriteReg(A7105_0A_CK0_PIN, 0x00);
	A7105_WriteReg(A7105_0B_GPIO1_PIN1, 0x01); //0x19//0x01
	A7105_WriteReg(A7105_0C_GPIO2_PIN_II, 0x21); //0x05
	A7105_WriteReg(A7105_0D_CLOCK, 0x05);
	A7105_WriteReg(A7105_0E_DATA_RATE, 0x00);
	A7105_WriteReg(A7105_0F_PLL_I, 0x50);
	A7105_WriteReg(A7105_10_PLL_II, 0x9E);
	A7105_WriteReg(A7105_11_PLL_III, 0x4B);
	A7105_WriteReg(A7105_12_PLL_IV, 0x00);
	A7105_WriteReg(A7105_13_PLL_V, 0x02);
	A7105_WriteReg(A7105_14_TX_I, 0x16);
	A7105_WriteReg(A7105_15_TX_II, 0x2B);
	A7105_WriteReg(A7105_16_DELAY_I, 0x12);
	A7105_WriteReg(A7105_17_DELAY_II, 0x00);
	A7105_WriteReg(A7105_18_RX, 0x62);
	A7105_WriteReg(A7105_19_RX_GAIN_I, 0x80);
	A7105_WriteReg(A7105_1A_RX_GAIN_II, 0x80);
	A7105_WriteReg(A7105_1B_RX_GAIN_III, 0x00);
	A7105_WriteReg(A7105_1C_RX_GAIN_IV, 0x0A);
	A7105_WriteReg(A7105_1D_RSSI_THOLD, 0x32);
	A7105_WriteReg(A7105_1E_ADC, 0xC3);
	A7105_WriteReg(A7105_1F_CODE_I, 0x0F);
	A7105_WriteReg(A7105_20_CODE_II, 0x13);
	A7105_WriteReg(A7105_21_CODE_III, 0xC3);
	A7105_WriteReg(A7105_22_IF_CALIB_I, 0x00);
	//	A7105_WriteReg(A7105_23_IF_CALIB_II, 0x00  );
	A7105_WriteReg(A7105_24_VCO_CURCAL, 0x00);
	A7105_WriteReg(A7105_25_VCO_SBCAL_I, 0x00);
	A7105_WriteReg(A7105_26_VCO_SBCAL_II, 0x3B);
	A7105_WriteReg(A7105_27_BATTERY_DET, 0x00);
	A7105_WriteReg(A7105_28_TX_TEST, 0x17);
	A7105_WriteReg(A7105_29_RX_DEM_TEST_I, 0x47);
	A7105_WriteReg(A7105_2A_RX_DEM_TEST_II, 0x80);
	A7105_WriteReg(A7105_2B_CPC, 0x03);
	A7105_WriteReg(A7105_2C_XTAL_TEST, 0x01);
	A7105_WriteReg(A7105_2D_PLL_TEST, 0x45);
	A7105_WriteReg(A7105_2E_VCO_TEST_I, 0x18);
	A7105_WriteReg(A7105_2F_VCO_TEST_II, 0x00);
	A7105_WriteReg(A7105_30_IFAT, 0x01);
	A7105_WriteReg(A7105_31_RSCALE, 0x0F);
	//A7105_WriteReg(A7105_32_FILTER_TEST, 0x  );


	A7105_Strobe(A7105_STANDBY);

	//IF Filter Bank Calibration
	A7105_WriteReg(0x02, 1);
	//vco_current =
	A7105_ReadReg(0x02);
	unsigned long ms = millis();

	while (millis() - ms < 500) {
		if (!A7105_ReadReg(0x02))
			break;
	}
	if (millis() - ms >= 500)
		return 0;

	if_calibration1 = A7105_ReadReg(A7105_22_IF_CALIB_I);
	if (if_calibration1 & A7105_MASK_FBCF) {
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

void ServoOut();

unsigned int servo_value = 1500;
unsigned int servo_value_old = 1500;

unsigned long loopTime = 0;


unsigned long ms = 0;
unsigned long mics = 0;
unsigned int i = 0;



unsigned char RADIO_STA = RADIO_INIT;
unsigned char RADIO_IGN_INT = 0;
unsigned char RADIO_FRA_DET = 0;
unsigned char RADIO_crc = 0;
unsigned char RADIO_data[20];
unsigned long cnt = 0;

float value = 0;
unsigned int dir = 1;

static uint8_t nunchuck_buf[6];   // array to store nunchuck data,

								  // main loop - wait for flag set in interrupt routine
void loop(void)
{
	RadioLink();
	ServoOut();
	if (micros() - loopTime >= 1500)
	{
		drawLCD();
		nunchuck_get_data();
		//nunchuck_print_data();  

		RADIO_STA = RADIO_TX;

		loopTime = micros();
	}

} // end of loop





void RadioLink() {

	switch (RADIO_STA) {

	case RADIO_INIT:

		/************** A7105 SETUP + INIT *********/
		A7105_Setup();
		//pinMode(GPIO1_PIN, INPUT); 
		//attachInterrupt(GPIO1_INT, A7105_FrameDetected, FALLING); 

		while (1) {
			if (flysky_init()) {
				beep(SPK_PIN, 4093, 50);
				break;
			}
			beep(SPK_PIN, 4093, 50);
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

		/* if(curchannel==16)
		curchannel = 0;
		curchannel += 3;
		if(curchannel>16)
		curchannel=16;
		*/

		A7105_WriteReg(A7105_0F_PLL_I, rx_channels[curchannel++]);
		if (curchannel >= 16)
			curchannel = 0;
		A7105_Strobe(A7105_RX);
		RADIO_IGN_INT = 0;

		RADIO_STA = RADIO_RX_WAIT;
		mics = micros();
		break;

	case RADIO_RX_WAIT:

		/************** FLYSKY RX WAIT - WAIT FOR SIGNAL *********/
		if (RADIO_FRA_DET) {
			//RADIO_STA = RADIO_RX;
			break;
		}
		if (micros() - mics >= 1600) {
			//RADIO_STA = RADIO_BIND;
			break;
		}
		break;

	case RADIO_RX:

		/************** FLYSKY FRAME RECEIVED *********/
		RADIO_FRA_DET = 0;
		RADIO_crc = A7105_ReadReg(0x00);

		if (!(RADIO_crc & A7105_MASK_CRC)) {

			A7105_ReadData(RADIO_data, 11);
			servo_value = (((RADIO_data[8] << 8) & 0xFF00) + RADIO_data[7]);

			if (cnt == 0) {
				for (int i = 0; i < 11; i++)
				{
					Serial.print(RADIO_data[i], HEX);
					Serial.print(" ");
				}
				Serial.print(" - ");

				Serial.print(servo_value, DEC);
				Serial.println();
				myservo.writeMicroseconds(servo_value);
				cnt = 0;

			}
			else
				cnt++;

			//servo_value_old = servo_value;
			//servo_value = (((RADIO_data[8]<<8)&0xFF00) + RADIO_data[7]);
			//servo_value = (unsigned int)(((servo_value*10*0.05) + (servo_value_old*10*0.95))/10);

		}
		RADIO_IGN_INT = 1;
		A7105_Strobe(A7105_STANDBY);
		A7105_Strobe(A7105_RST_RDPTR);
		A7105_WriteReg(A7105_0F_PLL_I, rx_channels[curchannel++]);
		if (curchannel >= 16)
			curchannel = 0;
		A7105_Strobe(A7105_RX);
		RADIO_IGN_INT = 0;

		RADIO_STA = RADIO_RX_WAIT;
		mics = micros();
		break;

	case RADIO_IDLE: break;

	case RADIO_TX:
		A7105_Transmitt(tx_channels[curchannel++]);
		if (curchannel >= 16)
			curchannel = 0;
		RADIO_STA = RADIO_RX_WAIT;
		mics = micros();
		break;


	}
}

unsigned long mics_servoout = 0;
unsigned int ServoOut_Status = 0;
void ServoOut() {
	//myservo.writeMicroseconds(servo_value);
	/*if(ServoOut_Status == 0){

	if(micros() - mics_servoout >= 11000) {
	mics_servoout = micros();
	ServoOut_Status = 1;
	digitalWrite(SRV_PIN,HIGH);
	}

	}
	else
	if(ServoOut_Status == 1){
	if(micros() - mics_servoout >= servo_value){
	mics_servoout = micros();
	ServoOut_Status = 0;
	digitalWrite(SRV_PIN,LOW);
	}
	}*/
}




void A7105_FrameDetected() {
	if (!RADIO_IGN_INT)
		RADIO_FRA_DET = 1;
}


void beep(unsigned char speakerPin, int frequencyInHertz, long timeInMilliseconds)     // the sound producing function
{
	int x;
	long delayAmount = (long)(1000000 / frequencyInHertz);
	long loopTime = (long)((timeInMilliseconds * 1000) / (delayAmount * 2));
	for (x = 0; x<loopTime; x++)
	{
		digitalWrite(speakerPin, HIGH);
		delayMicroseconds(delayAmount);
		digitalWrite(speakerPin, LOW);
		delayMicroseconds(delayAmount);
	}
}




void print_info() {


}

void print_hex(uint8_t value) {
	Serial.print("0x");
	if (value < 0x10)
		Serial.print('0');
	Serial.print(value, HEX);
	Serial.print("\n\r");
}

void print_dec(uint16_t value) {
	if (value < 10)
		Serial.print('0');
	Serial.print(value, DEC);
}

void printVertical(char * str, unsigned int x, unsigned int y);

float bat_volt[6] = { 3.23,3.34,3.12,3.25,3.36,3.40 };
float pack_info[6] = { 21.58,99.50,9.58,0.96,96.56,8.58 };

#define BAR_HIGHT 9
#define BAR_WIDTH 40
#define BAR_X 10
#define BAR_Y_OFFSET 11

void drawLCD() {
	float v;

	v = (((float)servo_value) / 1000.0)*2.5;
	for (int i = 0; i < 6; i++)
		bat_volt[i] = (float)(nunchuck_buf[0]) / 50.0;

	display.clearDisplay();

	display.setTextSize(1);
	display.setTextColor(WHITE);

	// Draw rect bar
	for (int x = 1, y = 0; y < BAR_Y_OFFSET * 6; x++, y = y + BAR_Y_OFFSET) {
		display.setCursor(0, y + 1);
		display.print(x);
		display.drawRect(BAR_X, y, BAR_WIDTH, BAR_HIGHT, WHITE);
	}

	// Fill rect with cell voltage and display cell voltage
	for (int x = 0, y = 0; y < BAR_Y_OFFSET * 6; x++, y = y + BAR_Y_OFFSET) {
		display.fillRect(BAR_X, y, bar_size(bat_volt[x]), BAR_HIGHT, WHITE);
		display.setCursor(BAR_X + BAR_WIDTH + 4, y + 1);
		display.print(bat_volt[x]);
		display.setCursor(BAR_X + BAR_WIDTH + 29, y + 1);
		display.print('V');
	}

	// Draw actual current level
	//display.drawRect(90,  0, 15, 55, WHITE);
	//printVertical("1345mA", 95, 5);

	// Draw charged left in battery
	//display.drawRect(110,  0, 15, 55, WHITE);
	//printVertical("9854mAh", 115, 5);


	// Draw battery pack voltage
	display.setCursor(90, 0);
	display.print(v);//display.print(pack_info[0]);
	display.setCursor(122, 0);
	display.print("V");

	// Draw current draw
	display.setCursor(90, 11);
	display.print(pack_info[1]);
	display.setCursor(122, 11);
	display.print("A");

	// Draw battery pack charge left
	display.setCursor(90, 22);
	display.print(pack_info[2]);
	display.setCursor(116, 22);
	display.print("Ah");

	// Draw battery pack charge used
	display.setCursor(90, 33);
	display.print(pack_info[3]);
	display.setCursor(116, 33);
	display.print("Ah");

	// Draw battery pack % left
	display.setCursor(90, 44);
	display.print(pack_info[4]);
	display.setCursor(122, 44);
	display.print("%");

	// Draw remote control battery pack voltage
	display.setCursor(95, 55);
	display.print(pack_info[5]);
	display.setCursor(122, 55);
	display.print("V");

	display.display();

}

unsigned int bar_size(float bat_volt) {
	unsigned int value = 0;

	if (bat_volt > 2.5) {
		value = (bat_volt - 2.5) * 40 / 1.7;
	}

	return value;
}

void printVertical(char * str, unsigned int x, unsigned int y) {
	char * ptr;
	int i;
	ptr = str;
	for (i = 0; *ptr != NULL; ptr++, i += 8) {
		display.setCursor(x, y + i);
		display.print(*ptr);
	}
}



// initialize the I2C system, join the I2C bus,
// and tell the nunchuck we're talking to it
void nunchuck_init()
{
	Wire.begin();                      // join i2c bus as master
	Wire.beginTransmission(0x52);     // transmit to device 0x52
	Wire.write(0x40);            // sends memory address
	Wire.write(0x00);            // sends sent a zero.  
	Wire.endTransmission();     // stop transmitting
}

// Send a request for data to the nunchuck
// was "send_zero()"
void nunchuck_send_request()
{
	Wire.beginTransmission(0x52);     // transmit to device 0x52
	Wire.write(0x00);            // sends one byte
	Wire.endTransmission();     // stop transmitting
}

// Receive data back from the nunchuck, 
int nunchuck_get_data()
{
	int cnt = 0;
	Wire.requestFrom(0x52, 6);     // request data from nunchuck
	while (Wire.available()) {
		// receive byte as an integer
		nunchuck_buf[cnt] = nunchuk_decode_byte(Wire.read());
		cnt++;
	}
	nunchuck_send_request();  // send request for next data payload
							  // If we recieved the 6 bytes, then go print them
	if (cnt >= 5) {
		return 1;   // success
	}
	return 0; //failure
}

// Print the input data we have recieved
// accel data is 10 bits long
// so we read 8 bits, then we have to add
// on the last 2 bits.  That is why I
// multiply them by 2 * 2
void nunchuck_print_data()
{
	static int i = 0;
	int joy_x_axis = nunchuck_buf[0];
	int joy_y_axis = nunchuck_buf[1];

	int accel_x_axis = nunchuck_buf[2]; // * 2 * 2; 
	int accel_y_axis = nunchuck_buf[3]; // * 2 * 2;
	int accel_z_axis = nunchuck_buf[4]; // * 2 * 2;


	int z_button = 0;
	int c_button = 0;

	// byte nunchuck_buf[5] contains bits for z and c buttons
	// it also contains the least significant bits for the accelerometer data
	// so we have to check each bit of byte outbuf[5]
	if ((nunchuck_buf[5] >> 0) & 1)
		z_button = 1;
	if ((nunchuck_buf[5] >> 1) & 1)
		c_button = 1;

	if ((nunchuck_buf[5] >> 2) & 1)
		accel_x_axis += 2;
	if ((nunchuck_buf[5] >> 3) & 1)
		accel_x_axis += 1;

	if ((nunchuck_buf[5] >> 4) & 1)
		accel_y_axis += 2;
	if ((nunchuck_buf[5] >> 5) & 1)
		accel_y_axis += 1;

	if ((nunchuck_buf[5] >> 6) & 1)
		accel_z_axis += 2;
	if ((nunchuck_buf[5] >> 7) & 1)
		accel_z_axis += 1;

	Serial.print(i, DEC);
	Serial.print("\t");

	Serial.print("joy:");
	Serial.print(joy_x_axis, DEC);
	Serial.print(",");
	Serial.print(joy_y_axis, DEC);
	Serial.print("  \t");

	Serial.print("acc:");
	Serial.print(accel_x_axis, DEC);
	Serial.print(",");
	Serial.print(accel_y_axis, DEC);
	Serial.print(",");
	Serial.print(accel_z_axis, DEC);
	Serial.print("\t");

	Serial.print("but:");
	Serial.print(z_button, DEC);
	Serial.print(",");
	Serial.print(c_button, DEC);

	Serial.print("\r\n");  // newline
	i++;
}

// Encode data to format that most wiimote drivers except
// only needed if you use one of the regular wiimote drivers
char nunchuk_decode_byte(char x)
{
	x = (x ^ 0x17) + 0x17;
	return x;
}

/*
// SPI interrupt routine
ISR (SPI_STC_vect)
{
byte c = SPDR;  // grab byte from SPI Data Register

if(!process_it){
buf_frame[0] = buf_frame[1];
buf_frame[1] = buf_frame[2];
buf_frame[2] = buf_frame[3];
buf_frame[3] = c;

if(buf_frame[0] == 0x07 && buf_frame[1] == 0x00 && buf_frame[2] == 0x27 && buf_frame[3] == 0x2E && frame_detected == false) {
frame_detected = true;
buf[0] = buf_frame[0];
buf[1] = buf_frame[1];
buf[2] = buf_frame[2];
buf[3] = buf_frame[3];
pos = 4;
return;
}

if(frame_detected) {
buf[pos++] = c;
if(pos==25)
process_it = true;

}
}
}  // end of interrupt routine SPI_STC_vect
*/

/*
void print_frame() {
unsigned int i;
for(i = 0 ; i < 24; i++){
if(frame[i] < 0x10)
Serial.print('0');
Serial.print(frame[i],HEX);
Serial.print(' ');
}
Serial.print('\n');
}
*/

/*
while(count < 16){  //print out the servo values
Serial.print("Channel ");
Serial.print(count+1, DEC);
Serial.print(" - ");
Serial.print(ppm[count]);
Serial.print("\n");
count++;
}
print_frame();
*/


/************* SPI SLAVE **********
// have to send on master in, *slave out*
pinMode(MISO, OUTPUT);
pinMode(MOSI, INPUT);

// turn on SPI in slave mode
SPCR |= _BV(SPE);

// get ready for an interrupt
pos = 0;   // buffer empty
process_it = false;
counter = 0;
frame_detected = false;
// now turn on interrupts
SPI.attachInterrupt();
**************** SPI SLAVE *********/

/************* SPI MASTER **********/
// Put SCK, MOSI, SS pins into output mode
// also put SCK, MOSI into LOW state, and SS into HIGH state.
// Then put SPI hardware into Master mode and turn SPI on
//SPI.begin ();
//SPI.setClockDivider(SPI_CLOCK_DIV64);
/************* SPI MASTER **********/
/*
unsigned char buf_frame[4];
unsigned char buf [100];
volatile byte pos;
volatile boolean process_it;
volatile boolean frame_detected;
volatile byte counter;
*/




/*
void print_info_() {
Serial.print("Throttle=");
Serial.print(throttle,HEX);
Serial.print('\n');
Serial.print("Yaw=");
Serial.print(yaw,HEX);
Serial.print('\n');
Serial.print("Pitch=");
Serial.print(pitch,HEX);
Serial.print('\n');
Serial.print("Roll=");
Serial.print(roll,HEX);
Serial.print('\n');
Serial.print("Flags=");
Serial.print(flags,HEX);
Serial.print('\n');
}
*/
/*
ISR(TIMER0_COMPA_vect){//timer0 interrupt 2kHz toggles pin 8
timer0_cnt=1;
}*/


/*   count = 0;
Serial.print("Channel ");
while(count < 4){  //print out the servo values

Serial.print(count+1, DEC);
Serial.print(" - ");
Serial.print(ppm[count]);
Serial.print("    ");
count++;
}
Serial.print("\n");
*/
