
#include <SoftwareSerial.h>

//this programm will put out a PPM signal
//////////////////////CONFIGURATION///////////////////////////////
#define chanel_number 16  //set the number of chanels
#define default_servo_value 1500  //set the default servo value
#define PPM_FrLen 22500  //set the PPM frame length in microseconds (1ms = 1000µs)
#define PPM_PulseLen 300  //set the pulse length
#define onState 1  //set polarity of the pulses: 1 is positive, 0 is negative
// #define sigPin PD5  //set PPM signal output pin on the arduino
#define sigPin 10
//////////////////////////////////////////////////////////////////
// LiquidCrystal_I2C lcd(0x27,16,2);
// SoftwareSerial Serial3(PD0,PD1);

/*this array holds the servo values for the ppm signal
 change theese values in your code (usually servo values move between 1000 and 2000)*/
int ppm[chanel_number];

#define USART Serial3

void setup(){  
  // lcd.init();
  // lcd.backlight();
  //initiallize default ppm values
  for(int i=0; i < chanel_number; i++){
    ppm[i]= default_servo_value;
  }
  while(!USART);
  Serial.begin(115200);
  USART.begin(115200);
  // USART.listen();

  pinMode(sigPin, OUTPUT);
  digitalWrite(sigPin, !onState);  //set the PPM signal pin to the default state (off)
  
  cli();
  TCCR1A = 0; // set entire TCCR1 register to 0
  TCCR1B = 0;
  
  OCR1A = 100;  // compare match register, change this
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  sei();
}



void loop(){
  uint8_t buffer[35];
  USART.readBytes(buffer, sizeof(buffer));
  delay(5);
  if (buffer[0] == 0x0F) {
  ppm[0] = (buffer[1]<<8 | buffer[2]);
  ppm[1] = (buffer[3]<<8 | buffer[4]);
  ppm[2] = (buffer[5]<<8 | buffer[6]);
  ppm[3] = (buffer[7]<<8 | buffer[8]);
  ppm[4] = (buffer[9]<<8 | buffer[10]);
  ppm[5] = (buffer[11]<<8 | buffer[12]);
  // ppm[6] = (buffer[13]<<8 | buffer[14]);
  // ppm[7] = (buffer[15]<<8 | buffer[16]);
  // ppm[8] = (buffer[17]<<8 | buffer[18]);
  // ppm[9] = (buffer[19]<<8 | buffer[20]);
  // ppm[10] = (buffer[21]<<8 | buffer[22]);
  // ppm[11] = (buffer[23]<<8 | buffer[24]);
  // ppm[12] = (buffer[25]<<8 | buffer[26]);
  ppm[13] = (buffer[27]<<8 | buffer[28]);
  // ppm[14] = (buffer[29]<<8 | buffer[30]);
  // ppm[15] = (buffer[31]<<8 | buffer[32]);
  }
  



  // Serial.print("ppm[0]: ");
  // Serial.println(ppm[0]);
  // Serial.print(" ");
  // Serial.print("ppm[1]: ");
  // Serial.println(ppm[1]);
  // Serial.print(" ");
  // Serial.print("ppm[2]: ");
  // Serial.println(ppm[2]);
  // Serial.print(" ");
  // Serial.print("ppm[3]: ");
  // Serial.println(ppm[3]);
  // Serial.print(" ");
  // Serial.print("ppm[4]: ");
  // Serial.println(ppm[4]);
  // Serial.print(" ");
  // Serial.print("ppm[5]: ");
  // Serial.println(ppm[5]);
  // Serial.print(" ");
  // Serial.println();

  // delay(1);
}

ISR(TIMER1_COMPA_vect){  //leave this alone
  static boolean state = true;
  
  TCNT1 = 0;
  
  if(state) {  //start pulse
    digitalWrite(sigPin, onState);
    OCR1A = PPM_PulseLen * 2;
    state = false;
  }
  else{  //end pulse and calculate when to start the next pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;
  
    digitalWrite(sigPin, !onState);
    state = true;

    if(cur_chan_numb >= chanel_number){
      cur_chan_numb = 0;
      calc_rest = calc_rest + PPM_PulseLen;// 
      OCR1A = (PPM_FrLen - calc_rest) * 2;
      calc_rest = 0;
    }
    else{
      OCR1A = (ppm[cur_chan_numb] - PPM_PulseLen) * 2;
      calc_rest = calc_rest + ppm[cur_chan_numb];
      cur_chan_numb++;
    }     
  }
}
