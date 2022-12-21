/*
  This is the SDR ver01 running on STM32f103 blue pill board 
  Requires serial communication to control frequency and several other HAM settings  
  Developed by S52UV 22.1.2018
*/
//#define LOG_OUT 1 // use the log output function
//#define FFT_N 64 // set to 256 point fft

//#include <FFT.h> // include the library

#include<stdlib.h>
#include <STM32ADC.h>
//#include "fft.c"

STM32ADC myADC(ADC1);

#define BOARD_LED PC13 //this is for blue pill
#define MYPWM PB0  // na bordu pise B0
#define TICP PA8  // pin za opazovanje trajanja interupta

#define AD9850_CLOCK 125000000         // Module crystal frequency. Tweak here for accuracy.za 125Mhz je 5V probam z 3.3V (110Mhz max)
#define W_CLK PB12                        // AD9850 Module pins.    
#define FQ_UD PB13       
#define DATA  PB14       
#define RESET PB15

#define MFFREQ (5500 * 4)   // 4 x vec kot medfrekvenca vedno


//Channels to be acquired. 
//uint8 pins[] = {11,10,9,8,7,6,5,4};
uint8 pins[] = {PA4,PA5};

//PWM konfiguracija
HardwareTimer pwmtimer(3);


const int maxSamples = 2; // 8 channels 

// Array for the ADC data
uint16_t dataPoints[maxSamples];
signed short outbuferL[64];
signed short outbuferR[64];
signed short outbuferfft[64];
signed short vhodL;
signed short vhodR;
signed short vhodL2;
signed short vhodR2;
unsigned char cnt = 0;
unsigned char cnt2 = 0;
unsigned char cikel = 0;
unsigned char cikel2 = 0;
unsigned char ssbmode=0;
unsigned char reqgraph=1;
unsigned char scan=0;
long acc;
long acc1;
long acc2;
    //short *coeffp; // pointer to coefficients
    //short *Linputp; // pointer to input samples for Left channel
    //short *Rinputp; // pointer to input samples for Left channel
    //short filout1[64];
    short filout2[64];
    //short realNumbers[64]; 
    //short imaginaryNumbers[64];
    unsigned char spekter[320];
    short n;
    short k;
    short ii;
    short i;
short zacasna;
short afpwm=512;
short zacasnai1;
short zacasnai2;
short zacasnai3;
//short fftava=0;


// tukaj spodaj so v bistvu -45 
const short LowpassAM [40] = {
0,
0,
0,
-1,
-2,
-2,
-1,
2,
6,
7,
5,
-3,
-14,
-21,
-19,
-3,
29,
68,
105,
127,
127,
105,
68,
29,
-3,
-19,
-21,
-14,
-3,
5,
7,
6,
2,
-1,
-2,
-2,
-1,
0,
0,
0};

const short Hilbcoefn45 [48] = {
-1,
-1,
-2,
-3,
-3,
-3,
-2,
-1,
0,
0,
-2,
-6,
-13,
-23,
-34,
-45,
-53,
-54,
-48,
-32,
-7,
23,
56,
86,
110,
125,
127,
119,
102,
79,
55,
33,
15,
4,
-2,
-3,
-2,
1,
4,
6,
6,
6,
4,
3,
2,
1,
0,
0};

const short HilbcoefnMF45 [48] = {
-1,
-1,
-1,
-1,
-1,
-1,
0,
-1,
-2,
-5,
-10,
-17,
-25,
-34,
-42,
-47,
-48,
-43,
-32,
-14,
9,
36,
64,
89,
109,
122,
127,
123,
112,
95,
76,
55,
37,
22,
11,
3,
0,
-1,
0,
1,
3,
3,
4,
4,
3,
2,
1,
1};

const short Hilb3khz [40] = {
0,
-1,
0,
0,
1,
1,
-1,
-4,
-5,
-2,
5,
10,
6,
-7,
-22,
-24,
-3,
41,
93,
127,
127,
93,
41,
-3,
-24,
-22,
-7,
6,
10,
5,
-2,
-5,
-4,
-1,
1,
1,
0,
0,
-1,
0};

const short Hilb6khz [40] = {
0,
0,
0,
-1,
-1,
1,
1,
-2,
-3,
2,
5,
-2,
-9,
0,
14,
4,
-24,
-18,
52,
127,
127,
52,
-18,
-24,
4,
14,
0,
-9,
-2,
5,
2,
-3,
-2,
1,
1,
-1,
-1,
0,
0,
0};

const short oscilo[64] = {
127,
90,
0,
-90,
-127,
-90,
0,
90,
127,
90,
0,
-90,
-127,
-90,
0,
90,
127,
90,
0,
-90,
-127,
-90,
0,
90,
127,
90,
0,
-90,
-127,
-90,
0,
90,
127,
90,
0,
-90,
-127,
-90,
0,
90,
127,
90,
0,
-90,
-127,
-90,
0,
90,
127,
90,
0,
-90,
-127,
-90,
0,
90,
127,
90,
0,
-90,
-127,
-90,
0,
90};


//int inByte = 65;                     // Byte read from Serial1
int correction = 0;
int inData = 0;
unsigned int mytimer=0;
unsigned int mytimer2=0;
unsigned short msignal=1;
unsigned short mmax=512;
//unsigned int Vscailed0;
//unsigned long freq = 14523600; // frequency in Hz
unsigned long freq = 14700000 - MFFREQ; // frequency in Hz
unsigned long freqmin = 1000000;
unsigned long freqmax = 40000000;
unsigned long freqold;
//long calibration_factor = 10000000 ;// 10002285;
long calibration_constant = 0; // 1700;
//unsigned long long hilfsf;
unsigned long korak;
uint32_t xkoren;

/*#define stepPin1 A3                    // Set 'Step' rotary encoder pins
#define stepPin2 A2
int forceHzStep = A4;                  // 'Step' rotary encoder's push button - Set 1 Hz steps.
int forcekHz = 4;                     // Interrupt-driven encoder's push button - force 1kHz freq.

Rotary i = Rotary(stepPin1, stepPin2); // Rotart encoder for setting increment.
Rotary r = Rotary(2, 3);               // Rotary encoder for frequency connects to interrupt pins
*/
//long int timer;

#define pulseHigh(pin) {digitalWrite(pin, HIGH); digitalWrite(pin, LOW); }


 // transfers a byte, a bit at a time, LSB first to the 9850 via serial DATA line
void tfr_byte(byte data) {
  for (int i = 0; i < 8; i++, data >>= 1) {
    digitalWrite(DATA, data & 0x01);
    pulseHigh(W_CLK);   //after each bit sent, CLK is pulsed high
  }
}

void sendFrequency(double frequency) {
  int32_t freq1 = frequency * 4294967295/AD9850_CLOCK;  // note 125 MHz clock on 9850
  for (int b = 0; b < 4; b++, freq1 >>= 8) {
    tfr_byte(freq1 & 0xFF);
  }
  tfr_byte(0x000);                     // Final control byte za w0, all 0 for 9850 chip
  pulseHigh(FQ_UD);                    // Done!  Should see output
}

void setup() {
  Serial.begin(9600);  //ne vem ce je potrebno ampak s tem dela
  Serial1.begin(9600);   // A9 je TX A10 je RX za serijsko z displayem
  //Serial2.begin(9600);   // A2 je TX A3 je RX za  upravljati ad9859 vfo pll
  pinMode(BOARD_LED, OUTPUT);
  pinMode(TICP, OUTPUT);
  pinMode(FQ_UD, OUTPUT);              // Configure pins for output to AD9850 module.
  pinMode(W_CLK, OUTPUT);
  pinMode(DATA, OUTPUT);
  pinMode(RESET, OUTPUT);
  //pinMode(PB0, INPUT);
  //Initialise the incoming serial number to zero
//  serial_input_number=0;
//startup blink... good idea from Pig-O-Scope
  digitalWrite(BOARD_LED, LOW);  // led je invertiran! LOW sveti!
  delay(100);
  digitalWrite(BOARD_LED, HIGH);  //Ugasnem!
  delay(100);

//calibrate ADC
  myADC.calibrate();

  // Set up our analog pin(s)
  for (unsigned int j = 0; j <2; j++) 
    pinMode(pins[j], INPUT_ANALOG);

    
    
  myADC.setSampleRate(ADC_SMPR_1_5);//set the Sample Rate
  myADC.setScanMode();              //set the ADC in Scan mode. 
  myADC.setPins(pins, 2);           //set how many and which pins to convert.
  myADC.setContinuous();            //set the ADC in continuous mode.

//set the DMA transfer for the ADC. 
//in this case we want to increment the memory side and run it in circular mode
//By doing this, we can read the last value sampled from the channels by reading the dataPoints array
  myADC.setDMA(dataPoints, 2, (DMA_MINC_MODE | DMA_CIRC_MODE), NULL);

//start the conversion. 
//because the ADC is set as continuous mode and in circular fashion, this can be done 
//on setup().  
  myADC.startConversion(); 

 
//PWM setup
  pwmtimer.setPrescaleFactor(1);   // 1 je 70kHz, 2 je 35kHz
  TIMER3_BASE->ARR = 1024; //10000/10000=1Hz, 1088 bo dalo 22050Hz
  pinMode(MYPWM, PWM);
  pwmWritefast(MYPWM, 512); 

    //timer 2 setup
  
  //TIMER2_BASE->CR1 = 0x0000; //Tdts=Tint, upcounter, timer disabled
  //TIMER2_BASE->CR2 = 0x0000; //already set to 0 by default
  //TIMER2_BASE->PSC = 7200; //72MHz/7200=10,000 Hz
  TIMER2_BASE->PSC = 1; //72MHz/7200=10,000 Hz 
  TIMER2_BASE->ARR = 816; //10000/10000=1Hz, 1088 bo dalo 22050Hz
  TIMER2_BASE->CNT = 0;  //clear counter
  
  timer_attach_interrupt(TIMER2, 0, handler_tim2); //interrupt on timer update
  TIMER2_BASE->CR1 |= 0x0001; //enable timer.
  
  //reset DDS
  pulseHigh(RESET);
  pulseHigh(W_CLK);
  pulseHigh(FQ_UD);    // this pulse enables serial mode - Datasheet page 12 figure 10
    
  //updateDisplay();       // Update the TFT display.stojan
  sendFrequency(freq);
  
}

uint16_t asqrt(uint32_t x) {
  /*      From http://medialab.freaknet.org/martin/src/sqrt/sqrt.c
   *  Logically, these are unsigned. We need the sign bit to test
   *  whether (op - res - one) underflowed.
   */
  int32_t op, res, one;

  op = x;
  res = 0;

  /* "one" starts at the highest power of four <= than the argument. */

  one = 1 << 30;  /* second-to-top bit set */
  while (one > op) one >>= 2;

  while (one != 0) {
    if (op >= res + one) {
      op = op - (res + one);
      res = res +  2 * one;
    }
    res /= 2;
    one /= 4;
  }
  return (uint16_t) (res);
}

//timer 2 interrupt
void handler_tim2(void){
  //TIMER2_BASE->SR &= 0xFFFE; //clear update interrupt flag
  
        //digitalWrite(BOARD_LED, LOW);
        //GPIOC_BASE->ODR ^= 0x2000; //toggle PC13
        //talejeGPIOA_BASE->ODR ^= 0x100; //toggle PA8  
        
      // apply the filter to each input sample
    
        //coeffp = &Hilbcoef45[0];
        //Linputp = &outbuferL[cnt];
        /*acc = 0;
        for ( k = 0; k < 30; k++ ) {
            //acc += (*coeffp++) * (*Linputp--);
            acc += Hilbcoef45[k] * ((outbuferL[(cnt-k)& 0x3F])+(outbuferR[(cnt-29+k)& 0x3F]));
        }*/
   switch (ssbmode)
   {
   case 0:
                 
         //outbuferR[cnt] = ((dataPoints[0]>>4)-127);  // 8 bitno
         //outbuferL[cnt] = ((dataPoints[1]>>4)-127);

         vhodR = ((dataPoints[0]>>4)-127);  // 8 bitno
         vhodL = ((dataPoints[1]>>4)-127); 
         vhodR2 = (vhodR*oscilo[cnt])>>7;  // 8 bitno
         vhodL2 = (vhodL*oscilo[(cnt+6)&0x3F])>>7;
         outbuferR[cnt] = vhodR2 - vhodL2; // lahko daš minus za drugo stran miror
         vhodR2 = (vhodR*oscilo[(cnt+6)&0x3F])>>7;  // 8 bitno
         vhodL2 = (vhodL*oscilo[cnt])>>7;
         outbuferL[cnt] = vhodR2 + vhodL2;  // lahko daš minus za drugo stran mirror

         /*vhodR = ((dataPoints[0]>>4)-127);  // 8 bitno
         vhodL = ((dataPoints[1]>>4)-127); 
         outbuferR[cnt] = (vhodR*oscilo[cnt])>>7;  // 8 bitno
         outbuferL[cnt] = (vhodL*oscilo[cnt])>>7;*/
             
      //tukaj Hilbert filtri -45 deg
            if (cikel == 0){
      acc=0; 
      acc += (short)(HilbcoefnMF45[47] * (outbuferL[(cnt-47)& 0x3F]-outbuferR[cnt]));
      acc += (short)(HilbcoefnMF45[46] * (outbuferL[(cnt-46)& 0x3F]-outbuferR[(cnt-1)& 0x3F]));
      acc += (short)(HilbcoefnMF45[45] * (outbuferL[(cnt-45)& 0x3F]-outbuferR[(cnt-2)& 0x3F]));
      acc += (short)(HilbcoefnMF45[44] * (outbuferL[(cnt-44)& 0x3F]-outbuferR[(cnt-3)& 0x3F]));
      acc += (short)(HilbcoefnMF45[43] * (outbuferL[(cnt-43)& 0x3F]-outbuferR[(cnt-4)& 0x3F]));
      acc += (short)(HilbcoefnMF45[42] * (outbuferL[(cnt-42)& 0x3F]-outbuferR[(cnt-5)& 0x3F]));
      acc += (short)(HilbcoefnMF45[41] * (outbuferL[(cnt-41)& 0x3F]-outbuferR[(cnt-6)& 0x3F]));
      acc += (short)(HilbcoefnMF45[40] * (outbuferL[(cnt-40)& 0x3F]-outbuferR[(cnt-7)& 0x3F]));
      acc += (short)(HilbcoefnMF45[39] * (outbuferL[(cnt-39)& 0x3F]-outbuferR[(cnt-8)& 0x3F]));
      //acc += (short)(HilbcoefnMF45[38] * (outbuferL[(cnt-38)& 0x3F]-outbuferR[(cnt-9)& 0x3F]));
      acc += (short)(HilbcoefnMF45[37] * (outbuferL[(cnt-37)& 0x3F]-outbuferR[(cnt-10)& 0x3F]));
      //acc += (short)(HilbcoefnMF45[36] * (outbuferL[(cnt-36)& 0x3F]-outbuferR[(cnt-11)& 0x3F]));
      acc += (short)(HilbcoefnMF45[35] * (outbuferL[(cnt-35)& 0x3F]-outbuferR[(cnt-12)& 0x3F]));
      acc += (short)(HilbcoefnMF45[34] * (outbuferL[(cnt-34)& 0x3F]-outbuferR[(cnt-13)& 0x3F]));
      acc += (short)(HilbcoefnMF45[33] * (outbuferL[(cnt-33)& 0x3F]-outbuferR[(cnt-14)& 0x3F]));
      acc += (short)(HilbcoefnMF45[32] * (outbuferL[(cnt-32)& 0x3F]-outbuferR[(cnt-15)& 0x3F]));
      acc += (short)(HilbcoefnMF45[31] * (outbuferL[(cnt-31)& 0x3F]-outbuferR[(cnt-16)& 0x3F]));
      acc += (short)(HilbcoefnMF45[30] * (outbuferL[(cnt-30)& 0x3F]-outbuferR[(cnt-17)& 0x3F]));
      acc += (short)(HilbcoefnMF45[29] * (outbuferL[(cnt-29)& 0x3F]-outbuferR[(cnt-18)& 0x3F]));
      acc += (short)(HilbcoefnMF45[28] * (outbuferL[(cnt-28)& 0x3F]-outbuferR[(cnt-19)& 0x3F]));
      acc += (short)(HilbcoefnMF45[27] * (outbuferL[(cnt-27)& 0x3F]-outbuferR[(cnt-20)& 0x3F]));
      acc += (short)(HilbcoefnMF45[26] * (outbuferL[(cnt-26)& 0x3F]-outbuferR[(cnt-21)& 0x3F]));
      acc += (short)(HilbcoefnMF45[25] * (outbuferL[(cnt-25)& 0x3F]-outbuferR[(cnt-22)& 0x3F]));
      acc += (short)(HilbcoefnMF45[24] * (outbuferL[(cnt-24)& 0x3F]-outbuferR[(cnt-23)& 0x3F]));
      acc += (short)(HilbcoefnMF45[23] * (outbuferL[(cnt-23)& 0x3F]-outbuferR[(cnt-24)& 0x3F]));
      acc += (short)(HilbcoefnMF45[22] * (outbuferL[(cnt-22)& 0x3F]-outbuferR[(cnt-25)& 0x3F]));     
      acc += (short)(HilbcoefnMF45[21] * (outbuferL[(cnt-21)& 0x3F]-outbuferR[(cnt-26)& 0x3F]));
      acc += (short)(HilbcoefnMF45[20] * (outbuferL[(cnt-20)& 0x3F]-outbuferR[(cnt-27)& 0x3F]));
      acc += (short)(HilbcoefnMF45[19] * (outbuferL[(cnt-19)& 0x3F]-outbuferR[(cnt-28)& 0x3F]));
      acc += (short)(HilbcoefnMF45[18] * (outbuferL[(cnt-18)& 0x3F]-outbuferR[(cnt-29)& 0x3F]));
      acc += (short)(HilbcoefnMF45[17] * (outbuferL[(cnt-17)& 0x3F]-outbuferR[(cnt-30)& 0x3F]));
      acc += (short)(HilbcoefnMF45[16] * (outbuferL[(cnt-16)& 0x3F]-outbuferR[(cnt-31)& 0x3F]));
      acc += (short)(HilbcoefnMF45[15] * (outbuferL[(cnt-15)& 0x3F]-outbuferR[(cnt-32)& 0x3F]));
      acc += (short)(HilbcoefnMF45[14] * (outbuferL[(cnt-14)& 0x3F]-outbuferR[(cnt-33)& 0x3F]));
      acc += (short)(HilbcoefnMF45[13] * (outbuferL[(cnt-13)& 0x3F]-outbuferR[(cnt-34)& 0x3F]));
      acc += (short)(HilbcoefnMF45[12] * (outbuferL[(cnt-12)& 0x3F]-outbuferR[(cnt-35)& 0x3F]));
      acc += (short)(HilbcoefnMF45[11] * (outbuferL[(cnt-11)& 0x3F]-outbuferR[(cnt-36)& 0x3F]));
      acc += (short)(HilbcoefnMF45[10] * (outbuferL[(cnt-10)& 0x3F]-outbuferR[(cnt-37)& 0x3F]));
      acc += (short)(HilbcoefnMF45[9] * (outbuferL[(cnt-9)& 0x3F]-outbuferR[(cnt-38)& 0x3F]));
      acc += (short)(HilbcoefnMF45[8] * (outbuferL[(cnt-8)& 0x3F]-outbuferR[(cnt-39)& 0x3F]));
      acc += (short)(HilbcoefnMF45[7] * (outbuferL[(cnt-7)& 0x3F]-outbuferR[(cnt-40)& 0x3F]));
      //acc += (short)(HilbcoefnMF45[6] * (outbuferL[(cnt-6)& 0x3F]-outbuferR[(cnt-41)& 0x3F]));
      acc += (short)(HilbcoefnMF45[5] * (outbuferL[(cnt-5)& 0x3F]-outbuferR[(cnt-42)& 0x3F]));
      acc += (short)(HilbcoefnMF45[4] * (outbuferL[(cnt-4)& 0x3F]-outbuferR[(cnt-43)& 0x3F]));
      acc += (short)(HilbcoefnMF45[3] * (outbuferL[(cnt-3)& 0x3F]-outbuferR[(cnt-44)& 0x3F]));
      acc += (short)(HilbcoefnMF45[2] * (outbuferL[(cnt-2)& 0x3F]-outbuferR[(cnt-45)& 0x3F]));
      acc += (short)(HilbcoefnMF45[1] * (outbuferL[(cnt-1)& 0x3F]-outbuferR[(cnt-46)& 0x3F]));
      acc += (short)(HilbcoefnMF45[0] * (outbuferL[cnt]-outbuferR[(cnt-47)& 0x3F]));
      
      }
      else{

       filout2[cnt2] = (short)(acc>>8);  //oscilator 5,5kHz       
       //filout2[cnt2] = (zacasnai3 * oscilo[cnt2])>>7; //oscilator je 5167,96875 Hz jih je 15 v 64korakih
        
        //cikel2++;
        //cikel2 &= 0x03; // da steje samo do 4 
      
      // lowpass 3khz filter standard
      acc2 = 0;
        //acc2 += (short)(Hilb3khz[39] * (filout2[(cnt2-39)& 0x3F]+filout2[cnt2]));
        acc2 += (short)(Hilb3khz[38] * (filout2[(cnt2-38)& 0x3F]+filout2[(cnt2-1)& 0x3F]));
        //acc2 += (short)(Hilb3khz[37] * (filout2[(cnt2-37)& 0x3F]+filout2[(cnt2-2)& 0x3F]));
        //acc2 += (short)(Hilb3khz[36] * (filout2[(cnt2-36)& 0x3F]+filout2[(cnt2-3)& 0x3F]));
        acc2 += (short)(Hilb3khz[35] * (filout2[(cnt2-35)& 0x3F]+filout2[(cnt2-4)& 0x3F]));
        acc2 += (short)(Hilb3khz[34] * (filout2[(cnt2-34)& 0x3F]+filout2[(cnt2-5)& 0x3F]));
        acc2 += (short)(Hilb3khz[33] * (filout2[(cnt2-33)& 0x3F]+filout2[(cnt2-6)& 0x3F]));
        acc2 += (short)(Hilb3khz[32] * (filout2[(cnt2-32)& 0x3F]+filout2[(cnt2-7)& 0x3F]));   
        acc2 += (short)(Hilb3khz[31] * (filout2[(cnt2-31)& 0x3F]+filout2[(cnt2-8)& 0x3F]));
        acc2 += (short)(Hilb3khz[30] * (filout2[(cnt2-30)& 0x3F]+filout2[(cnt2-9)& 0x3F]));
        acc2 += (short)(Hilb3khz[29] * (filout2[(cnt2-29)& 0x3F]+filout2[(cnt2-10)& 0x3F]));
        acc2 += (short)(Hilb3khz[28] * (filout2[(cnt2-28)& 0x3F]+filout2[(cnt2-11)& 0x3F]));
        acc2 += (short)(Hilb3khz[27] * (filout2[(cnt2-27)& 0x3F]+filout2[(cnt2-12)& 0x3F]));
        acc2 += (short)(Hilb3khz[26] * (filout2[(cnt2-26)& 0x3F]+filout2[(cnt2-13)& 0x3F]));   
        acc2 += (short)(Hilb3khz[25] * (filout2[(cnt2-25)& 0x3F]+filout2[(cnt2-14)& 0x3F]));
        acc2 += (short)(Hilb3khz[24] * (filout2[(cnt2-24)& 0x3F]+filout2[(cnt2-15)& 0x3F]));
        acc2 += (short)(Hilb3khz[23] * (filout2[(cnt2-23)& 0x3F]+filout2[(cnt2-16)& 0x3F]));
        acc2 += (short)(Hilb3khz[22] * (filout2[(cnt2-22)& 0x3F]+filout2[(cnt2-17)& 0x3F]));
        acc2 += (short)(Hilb3khz[21] * (filout2[(cnt2-21)& 0x3F]+filout2[(cnt2-18)& 0x3F]));
        acc2 += (short)(Hilb3khz[20] * (filout2[(cnt2-20)& 0x3F]+filout2[(cnt2-19)& 0x3F]));

               
        afpwm = 512 + (short)(acc2>>7); //nova audio točka vsakih 45us         
        pwmWritefast(MYPWM,afpwm);  

      cnt2++;
      if (cnt2>=64) {cnt2=0;}
      }
      break;
       
      case 1:                 
         outbuferR[cnt] = ((dataPoints[0]>>4)-127);  // 8 bitno
         outbuferL[cnt] = ((dataPoints[1]>>4)-127); 
       
      //tukaj Hilbert filtri -45 deg
            if (cikel == 0){
      acc=0; 
      //acc += (short)(Hilbcoefn45[47] * (outbuferL[(cnt-47)& 0x3F]-outbuferR[cnt]));
      //acc += (short)(Hilbcoefn45[46] * (outbuferL[(cnt-46)& 0x3F]-outbuferR[(cnt-1)& 0x3F]));
      acc += (short)(Hilbcoefn45[45] * (outbuferL[(cnt-45)& 0x3F]-outbuferR[(cnt-2)& 0x3F]));
      acc += (short)(Hilbcoefn45[44] * (outbuferL[(cnt-44)& 0x3F]-outbuferR[(cnt-3)& 0x3F]));
      acc += (short)(Hilbcoefn45[43] * (outbuferL[(cnt-43)& 0x3F]-outbuferR[(cnt-4)& 0x3F]));
      acc += (short)(Hilbcoefn45[42] * (outbuferL[(cnt-42)& 0x3F]-outbuferR[(cnt-5)& 0x3F]));
      acc += (short)(Hilbcoefn45[41] * (outbuferL[(cnt-41)& 0x3F]-outbuferR[(cnt-6)& 0x3F]));
      acc += (short)(Hilbcoefn45[40] * (outbuferL[(cnt-40)& 0x3F]-outbuferR[(cnt-7)& 0x3F]));
      acc += (short)(Hilbcoefn45[39] * (outbuferL[(cnt-39)& 0x3F]-outbuferR[(cnt-8)& 0x3F]));
      acc += (short)(Hilbcoefn45[38] * (outbuferL[(cnt-38)& 0x3F]-outbuferR[(cnt-9)& 0x3F]));
      acc += (short)(Hilbcoefn45[37] * (outbuferL[(cnt-37)& 0x3F]-outbuferR[(cnt-10)& 0x3F]));
      acc += (short)(Hilbcoefn45[36] * (outbuferL[(cnt-36)& 0x3F]-outbuferR[(cnt-11)& 0x3F]));
      acc += (short)(Hilbcoefn45[35] * (outbuferL[(cnt-35)& 0x3F]-outbuferR[(cnt-12)& 0x3F]));
      acc += (short)(Hilbcoefn45[34] * (outbuferL[(cnt-34)& 0x3F]-outbuferR[(cnt-13)& 0x3F]));
      acc += (short)(Hilbcoefn45[33] * (outbuferL[(cnt-33)& 0x3F]-outbuferR[(cnt-14)& 0x3F]));
      acc += (short)(Hilbcoefn45[32] * (outbuferL[(cnt-32)& 0x3F]-outbuferR[(cnt-15)& 0x3F]));
      acc += (short)(Hilbcoefn45[31] * (outbuferL[(cnt-31)& 0x3F]-outbuferR[(cnt-16)& 0x3F]));
      acc += (short)(Hilbcoefn45[30] * (outbuferL[(cnt-30)& 0x3F]-outbuferR[(cnt-17)& 0x3F]));
      acc += (short)(Hilbcoefn45[29] * (outbuferL[(cnt-29)& 0x3F]-outbuferR[(cnt-18)& 0x3F]));
      acc += (short)(Hilbcoefn45[28] * (outbuferL[(cnt-28)& 0x3F]-outbuferR[(cnt-19)& 0x3F]));
      acc += (short)(Hilbcoefn45[27] * (outbuferL[(cnt-27)& 0x3F]-outbuferR[(cnt-20)& 0x3F]));
      acc += (short)(Hilbcoefn45[26] * (outbuferL[(cnt-26)& 0x3F]-outbuferR[(cnt-21)& 0x3F]));
      acc += (short)(Hilbcoefn45[25] * (outbuferL[(cnt-25)& 0x3F]-outbuferR[(cnt-22)& 0x3F]));
      acc += (short)(Hilbcoefn45[24] * (outbuferL[(cnt-24)& 0x3F]-outbuferR[(cnt-23)& 0x3F]));
      acc += (short)(Hilbcoefn45[23] * (outbuferL[(cnt-23)& 0x3F]-outbuferR[(cnt-24)& 0x3F]));
      acc += (short)(Hilbcoefn45[22] * (outbuferL[(cnt-22)& 0x3F]-outbuferR[(cnt-25)& 0x3F]));     
      acc += (short)(Hilbcoefn45[21] * (outbuferL[(cnt-21)& 0x3F]-outbuferR[(cnt-26)& 0x3F]));
      acc += (short)(Hilbcoefn45[20] * (outbuferL[(cnt-20)& 0x3F]-outbuferR[(cnt-27)& 0x3F]));
      acc += (short)(Hilbcoefn45[19] * (outbuferL[(cnt-19)& 0x3F]-outbuferR[(cnt-28)& 0x3F]));
      acc += (short)(Hilbcoefn45[18] * (outbuferL[(cnt-18)& 0x3F]-outbuferR[(cnt-29)& 0x3F]));
      acc += (short)(Hilbcoefn45[17] * (outbuferL[(cnt-17)& 0x3F]-outbuferR[(cnt-30)& 0x3F]));
      acc += (short)(Hilbcoefn45[16] * (outbuferL[(cnt-16)& 0x3F]-outbuferR[(cnt-31)& 0x3F]));
      acc += (short)(Hilbcoefn45[15] * (outbuferL[(cnt-15)& 0x3F]-outbuferR[(cnt-32)& 0x3F]));
      acc += (short)(Hilbcoefn45[14] * (outbuferL[(cnt-14)& 0x3F]-outbuferR[(cnt-33)& 0x3F]));
      acc += (short)(Hilbcoefn45[13] * (outbuferL[(cnt-13)& 0x3F]-outbuferR[(cnt-34)& 0x3F]));
      acc += (short)(Hilbcoefn45[12] * (outbuferL[(cnt-12)& 0x3F]-outbuferR[(cnt-35)& 0x3F]));
      acc += (short)(Hilbcoefn45[11] * (outbuferL[(cnt-11)& 0x3F]-outbuferR[(cnt-36)& 0x3F]));
      acc += (short)(Hilbcoefn45[10] * (outbuferL[(cnt-10)& 0x3F]-outbuferR[(cnt-37)& 0x3F]));
      //acc += (short)(Hilbcoefn45[9] * (outbuferL[(cnt-9)& 0x3F]-outbuferR[(cnt-38)& 0x3F]));
      //acc += (short)(Hilbcoefn45[8] * (outbuferL[(cnt-8)& 0x3F]-outbuferR[(cnt-39)& 0x3F]));
      acc += (short)(Hilbcoefn45[7] * (outbuferL[(cnt-7)& 0x3F]-outbuferR[(cnt-40)& 0x3F]));
      acc += (short)(Hilbcoefn45[6] * (outbuferL[(cnt-6)& 0x3F]-outbuferR[(cnt-41)& 0x3F]));
      acc += (short)(Hilbcoefn45[5] * (outbuferL[(cnt-5)& 0x3F]-outbuferR[(cnt-42)& 0x3F]));
      acc += (short)(Hilbcoefn45[4] * (outbuferL[(cnt-4)& 0x3F]-outbuferR[(cnt-43)& 0x3F]));
      acc += (short)(Hilbcoefn45[3] * (outbuferL[(cnt-3)& 0x3F]-outbuferR[(cnt-44)& 0x3F]));
      acc += (short)(Hilbcoefn45[2] * (outbuferL[(cnt-2)& 0x3F]-outbuferR[(cnt-45)& 0x3F]));
      acc += (short)(Hilbcoefn45[1] * (outbuferL[(cnt-1)& 0x3F]-outbuferR[(cnt-46)& 0x3F]));
      acc += (short)(Hilbcoefn45[0] * (outbuferL[cnt]-outbuferR[(cnt-47)& 0x3F]));
      
      }
      else{

      //filout2[cnt2]=outbuferL[cnt]; 
        filout2[cnt2]= (short)(acc>>9); // sampli v vrsti pred 3khz filtrom so nalozeni z 22050hz
      
      // lowpass 3khz filter standard
      acc2 = 0;
        //acc2 += (short)(Hilb3khz[39] * (filout2[(cnt2-39)& 0x3F]+filout2[cnt2]));
        acc2 += (short)(Hilb3khz[38] * (filout2[(cnt2-38)& 0x3F]+filout2[(cnt2-1)& 0x3F]));
        //acc2 += (short)(Hilb3khz[37] * (filout2[(cnt2-37)& 0x3F]+filout2[(cnt2-2)& 0x3F]));
        //acc2 += (short)(Hilb3khz[36] * (filout2[(cnt2-36)& 0x3F]+filout2[(cnt2-3)& 0x3F]));
        acc2 += (short)(Hilb3khz[35] * (filout2[(cnt2-35)& 0x3F]+filout2[(cnt2-4)& 0x3F]));
        acc2 += (short)(Hilb3khz[34] * (filout2[(cnt2-34)& 0x3F]+filout2[(cnt2-5)& 0x3F]));
        acc2 += (short)(Hilb3khz[33] * (filout2[(cnt2-33)& 0x3F]+filout2[(cnt2-6)& 0x3F]));
        acc2 += (short)(Hilb3khz[32] * (filout2[(cnt2-32)& 0x3F]+filout2[(cnt2-7)& 0x3F]));   
        acc2 += (short)(Hilb3khz[31] * (filout2[(cnt2-31)& 0x3F]+filout2[(cnt2-8)& 0x3F]));
        acc2 += (short)(Hilb3khz[30] * (filout2[(cnt2-30)& 0x3F]+filout2[(cnt2-9)& 0x3F]));
        acc2 += (short)(Hilb3khz[29] * (filout2[(cnt2-29)& 0x3F]+filout2[(cnt2-10)& 0x3F]));
        acc2 += (short)(Hilb3khz[28] * (filout2[(cnt2-28)& 0x3F]+filout2[(cnt2-11)& 0x3F]));
        acc2 += (short)(Hilb3khz[27] * (filout2[(cnt2-27)& 0x3F]+filout2[(cnt2-12)& 0x3F]));
        acc2 += (short)(Hilb3khz[26] * (filout2[(cnt2-26)& 0x3F]+filout2[(cnt2-13)& 0x3F]));   
        acc2 += (short)(Hilb3khz[25] * (filout2[(cnt2-25)& 0x3F]+filout2[(cnt2-14)& 0x3F]));
        acc2 += (short)(Hilb3khz[24] * (filout2[(cnt2-24)& 0x3F]+filout2[(cnt2-15)& 0x3F]));
        acc2 += (short)(Hilb3khz[23] * (filout2[(cnt2-23)& 0x3F]+filout2[(cnt2-16)& 0x3F]));
        acc2 += (short)(Hilb3khz[22] * (filout2[(cnt2-22)& 0x3F]+filout2[(cnt2-17)& 0x3F]));
        acc2 += (short)(Hilb3khz[21] * (filout2[(cnt2-21)& 0x3F]+filout2[(cnt2-18)& 0x3F]));
        acc2 += (short)(Hilb3khz[20] * (filout2[(cnt2-20)& 0x3F]+filout2[(cnt2-19)& 0x3F]));

               
        afpwm = 512 + (short)(acc2>>7); //nova audio točka vsakih 45us         
        pwmWritefast(MYPWM,afpwm);  

      cnt2++;
      if (cnt2>=64) {cnt2=0;}
      }
      break;
      case 2:
         //outbuferR[cnt] = ((dataPoints[0]>>4)-127);  // 8 bitno
         //outbuferL[cnt] = ((dataPoints[1]>>4)-127);         
         vhodR = ((dataPoints[0]>>4)-127);  // 8 bitno
         vhodL = ((dataPoints[1]>>4)-127); 
         vhodR2 = (vhodR*oscilo[cnt])>>7;  // 8 bitno
         vhodL2 = (vhodL*oscilo[(cnt+6)&0x3F])>>7;
         outbuferR[cnt] = vhodR2 - vhodL2; // lahko daš minus za drugo stran miror
         vhodR2 = (vhodR*oscilo[(cnt+6)&0x3F])>>7;  // 8 bitno
         vhodL2 = (vhodL*oscilo[cnt])>>7;
         outbuferL[cnt] = vhodR2 + vhodL2;  // lahko daš minus za drugo stran mirror
                 
         //filout1[cnt] = outbuferR[cnt] + outbuferL[cnt] 
       
      //tukaj Hilbert filtri low pass
            if (cikel == 0){
      acc=0; 
        //acc += (short)(LowpassAM[39] * (outbuferL[(cnt-39)& 0x3F]+outbuferL[cnt]));
       // acc += (short)(LowpassAM[38] * (outbuferL[(cnt-38)& 0x3F]+outbuferL[(cnt-1)& 0x3F]));
        //acc += (short)(LowpassAM[37] * (outbuferL[(cnt-37)& 0x3F]+outbuferL[(cnt-2)& 0x3F]));
        acc += (short)(LowpassAM[36] * (outbuferL[(cnt-36)& 0x3F]+outbuferL[(cnt-3)& 0x3F]));
        acc += (short)(LowpassAM[35] * (outbuferL[(cnt-35)& 0x3F]+outbuferL[(cnt-4)& 0x3F]));
        acc += (short)(LowpassAM[34] * (outbuferL[(cnt-34)& 0x3F]+outbuferL[(cnt-5)& 0x3F]));
        acc += (short)(LowpassAM[33] * (outbuferL[(cnt-33)& 0x3F]+outbuferL[(cnt-6)& 0x3F]));
        acc += (short)(LowpassAM[32] * (outbuferL[(cnt-32)& 0x3F]+outbuferL[(cnt-7)& 0x3F]));   
        acc += (short)(LowpassAM[31] * (outbuferL[(cnt-31)& 0x3F]+outbuferL[(cnt-8)& 0x3F]));
        acc += (short)(LowpassAM[30] * (outbuferL[(cnt-30)& 0x3F]+outbuferL[(cnt-9)& 0x3F]));
        acc += (short)(LowpassAM[29] * (outbuferL[(cnt-29)& 0x3F]+outbuferL[(cnt-10)& 0x3F]));
        acc += (short)(LowpassAM[28] * (outbuferL[(cnt-28)& 0x3F]+outbuferL[(cnt-11)& 0x3F]));
        acc += (short)(LowpassAM[27] * (outbuferL[(cnt-27)& 0x3F]+outbuferL[(cnt-12)& 0x3F]));
        acc += (short)(LowpassAM[26] * (outbuferL[(cnt-26)& 0x3F]+outbuferL[(cnt-13)& 0x3F]));   
        acc += (short)(LowpassAM[25] * (outbuferL[(cnt-25)& 0x3F]+outbuferL[(cnt-14)& 0x3F]));
        acc += (short)(LowpassAM[24] * (outbuferL[(cnt-24)& 0x3F]+outbuferL[(cnt-15)& 0x3F]));
        acc += (short)(LowpassAM[23] * (outbuferL[(cnt-23)& 0x3F]+outbuferL[(cnt-16)& 0x3F]));
        acc += (short)(LowpassAM[22] * (outbuferL[(cnt-22)& 0x3F]+outbuferL[(cnt-17)& 0x3F]));
        acc += (short)(LowpassAM[21] * (outbuferL[(cnt-21)& 0x3F]+outbuferL[(cnt-18)& 0x3F]));
        acc += (short)(LowpassAM[20] * (outbuferL[(cnt-20)& 0x3F]+outbuferL[(cnt-19)& 0x3F]));

      acc1=0; 
        //acc1 += (short)(LowpassAM[39] * (outbuferR[(cnt-39)& 0x3F]+outbuferR[cnt]));
        //acc1 += (short)(LowpassAM[38] * (outbuferR[(cnt-38)& 0x3F]+outbuferR[(cnt-1)& 0x3F]));
       // acc1 += (short)(LowpassAM[37] * (outbuferR[(cnt-37)& 0x3F]+outbuferR[(cnt-2)& 0x3F]));
        acc1 += (short)(LowpassAM[36] * (outbuferR[(cnt-36)& 0x3F]+outbuferR[(cnt-3)& 0x3F]));
        acc1 += (short)(LowpassAM[35] * (outbuferR[(cnt-35)& 0x3F]+outbuferR[(cnt-4)& 0x3F]));
        acc1 += (short)(LowpassAM[34] * (outbuferR[(cnt-34)& 0x3F]+outbuferR[(cnt-5)& 0x3F]));
        acc1 += (short)(LowpassAM[33] * (outbuferR[(cnt-33)& 0x3F]+outbuferR[(cnt-6)& 0x3F]));
        acc1 += (short)(LowpassAM[32] * (outbuferR[(cnt-32)& 0x3F]+outbuferR[(cnt-7)& 0x3F]));   
        acc1 += (short)(LowpassAM[31] * (outbuferR[(cnt-31)& 0x3F]+outbuferR[(cnt-8)& 0x3F]));
        acc1 += (short)(LowpassAM[30] * (outbuferR[(cnt-30)& 0x3F]+outbuferR[(cnt-9)& 0x3F]));
        acc1 += (short)(LowpassAM[29] * (outbuferR[(cnt-29)& 0x3F]+outbuferR[(cnt-10)& 0x3F]));
        acc1 += (short)(LowpassAM[28] * (outbuferR[(cnt-28)& 0x3F]+outbuferR[(cnt-11)& 0x3F]));
        acc1 += (short)(LowpassAM[27] * (outbuferR[(cnt-27)& 0x3F]+outbuferR[(cnt-12)& 0x3F]));
        acc1 += (short)(LowpassAM[26] * (outbuferR[(cnt-26)& 0x3F]+outbuferR[(cnt-13)& 0x3F]));   
        acc1 += (short)(LowpassAM[25] * (outbuferR[(cnt-25)& 0x3F]+outbuferR[(cnt-14)& 0x3F]));
        acc1 += (short)(LowpassAM[24] * (outbuferR[(cnt-24)& 0x3F]+outbuferR[(cnt-15)& 0x3F]));
        acc1 += (short)(LowpassAM[23] * (outbuferR[(cnt-23)& 0x3F]+outbuferR[(cnt-16)& 0x3F]));
        acc1 += (short)(LowpassAM[22] * (outbuferR[(cnt-22)& 0x3F]+outbuferR[(cnt-17)& 0x3F]));
        acc1 += (short)(LowpassAM[21] * (outbuferR[(cnt-21)& 0x3F]+outbuferR[(cnt-18)& 0x3F]));
        acc1 += (short)(LowpassAM[20] * (outbuferR[(cnt-20)& 0x3F]+outbuferR[(cnt-19)& 0x3F]));

        zacasnai1= (short)(acc>>8);
        zacasnai2= (short)(acc1>>8);
        xkoren = (zacasnai1*zacasnai1) + (zacasnai2*zacasnai2);     
      }
      else{

        // zacasnai3 = (short)(acc>>7);  //oscilator 5,5kHz       
       //filout2[cnt2] = (zacasnai3 * oscilo[cnt2])>>7; //oscilator je 5167,96875 Hz jih je 15 v 64korakih
        
       // cikel2++;
       // cikel2 &= 0x03; // da steje samo do 4
       //filout2[cnt2]=zacasnai1 + zacasnai2; 
        filout2[cnt2]= asqrt(xkoren); // AM demodulation
      
      // lowpass 5khz filter standard
      acc2 = 0;
           
        //acc2 += (short)(Hilb6khz[39] * (filout2[(cnt2-39)& 0x3F]+filout2[cnt2]));
        //acc2 += (short)(Hilb6khz[38] * (filout2[(cnt2-38)& 0x3F]+filout2[(cnt2-1)& 0x3F]));
        //acc2 += (short)(Hilb6khz[37] * (filout2[(cnt2-37)& 0x3F]+filout2[(cnt2-2)& 0x3F]));
        acc2 += (short)(Hilb6khz[36] * (filout2[(cnt2-36)& 0x3F]+filout2[(cnt2-3)& 0x3F]));
        acc2 += (short)(Hilb6khz[35] * (filout2[(cnt2-35)& 0x3F]+filout2[(cnt2-4)& 0x3F]));
        acc2 += (short)(Hilb6khz[34] * (filout2[(cnt2-34)& 0x3F]+filout2[(cnt2-5)& 0x3F]));
        acc2 += (short)(Hilb6khz[33] * (filout2[(cnt2-33)& 0x3F]+filout2[(cnt2-6)& 0x3F]));
        acc2 += (short)(Hilb6khz[32] * (filout2[(cnt2-32)& 0x3F]+filout2[(cnt2-7)& 0x3F]));   
        acc2 += (short)(Hilb6khz[31] * (filout2[(cnt2-31)& 0x3F]+filout2[(cnt2-8)& 0x3F]));
        acc2 += (short)(Hilb6khz[30] * (filout2[(cnt2-30)& 0x3F]+filout2[(cnt2-9)& 0x3F]));
        acc2 += (short)(Hilb6khz[29] * (filout2[(cnt2-29)& 0x3F]+filout2[(cnt2-10)& 0x3F]));
        acc2 += (short)(Hilb6khz[28] * (filout2[(cnt2-28)& 0x3F]+filout2[(cnt2-11)& 0x3F]));
        acc2 += (short)(Hilb6khz[27] * (filout2[(cnt2-27)& 0x3F]+filout2[(cnt2-12)& 0x3F]));
        //acc2 += (short)(Hilb6khz[26] * (filout2[(cnt2-26)& 0x3F]+filout2[(cnt2-13)& 0x3F]));   
        acc2 += (short)(Hilb6khz[25] * (filout2[(cnt2-25)& 0x3F]+filout2[(cnt2-14)& 0x3F]));
        acc2 += (short)(Hilb6khz[24] * (filout2[(cnt2-24)& 0x3F]+filout2[(cnt2-15)& 0x3F]));
        acc2 += (short)(Hilb6khz[23] * (filout2[(cnt2-23)& 0x3F]+filout2[(cnt2-16)& 0x3F]));
        acc2 += (short)(Hilb6khz[22] * (filout2[(cnt2-22)& 0x3F]+filout2[(cnt2-17)& 0x3F]));
        acc2 += (short)(Hilb6khz[21] * (filout2[(cnt2-21)& 0x3F]+filout2[(cnt2-18)& 0x3F]));
        acc2 += (short)(Hilb6khz[20] * (filout2[(cnt2-20)& 0x3F]+filout2[(cnt2-19)& 0x3F]));

               
        afpwm = 512 + (short)(acc2>>7); //nova audio točka vsakih 45us         
        pwmWritefast(MYPWM,afpwm);  

      cnt2++;
      if (cnt2>=64) {cnt2=0;}
      }
      break; 
    default:
    
    break;
  }

     
    if (cnt==63 && reqgraph==1){
       for (ii=0;ii<64;ii++){
        outbuferfft[ii] = filout2[ii];
        //outbuferfft[ii] = outbuferR[ii];  
       }
        //for (i=0;i<128;i+=2){
        //fft_input[i] = outbuferL[i>>1]; // put real data into even bins
       // fft_input[i+1] = 0; // set odd bins to 0           
       //}       
       reqgraph = 0;
    }                                
    cikel++;
  cikel &= 0x01; // da steje samo do 2 
  cnt++;
  if (cnt>=64) {cnt=0;}
    
    mytimer++;
    mytimer2++;
  //*PFDATDIR &= CLR3;   //opazovalni pin=0
     // pwmWritefast(MYPWM, 512);   
        //cnt=cnt+1;
        //if (cnt == 64) {cnt = 0;}
        //delay(1000); 
        //digitalWrite(BOARD_LED, HIGH);
        //GPIOC_BASE->ODR ^= 0x2000; //toggle PC13
        //talejeGPIOA_BASE->ODR ^= 0x100; //toggle PA8 
}

void pwmWritefast(uint8 pin, uint16 duty_cycle) {
   timer_set_compare(PIN_MAP[pin].timer_device, PIN_MAP[pin].timer_channel, duty_cycle);
}

void sweep () {
  //digitalWrite(LED, LOW);   // turn the LED on (HIGH is the voltage level)
  freq = freqmin;
  for(unsigned short s = 0; s < 320; s++) {
  //tukaj nastavi frekvenco
      sendFrequency(freq);
      delay(4);              // wait for a second
      mmax = 512;
       for (unsigned short t = 0; t < 100; t++){
      calcsignal();
                  }
       msignal = (mmax - 512)>>2;
       if (msignal > 80){msignal = 80;}
       spekter[s]=msignal;
       mmax = 512;
      
        freq=freq + 2500;
        }
        //digitalWrite(LED, HIGH);    // turn the LED off by making the voltage LOW        
    }
    
void calcsignal(void)
{
  short ivalue;
  ivalue = afpwm;
   if (mmax < ivalue) {mmax = ivalue;}   
}

/*void fftrutina(void)
{
      short i = 0;
      short k;
      long place, root;
   // Get 64 samples at 50uS intervals
                // 50uS means our sampling rate is 20KHz which gives us
                // Nyquist limit of 10Khz
                
                for (i = 0; i < 64; i++)
                       {
                        // Perform the ADC conversion                
                        // Get the 10-bit ADC result and adjust the virtual ground of 2.5V
                        // back to 0Vs to make the input wave centered correctly around 0
                        // (i.e. -512 to +512)
                        
                        realNumbers[i] = (outbuferfft[i]*16);  //tukaj se skaliraj            
                        // Set the imaginary number to zero
                        imaginaryNumbers[i] = 0;
                        
                       }

    // Note: the '6' is the size of the input data (2 to the power of 6 = 64)
  
                fix_fft(realNumbers, imaginaryNumbers, 6);  //tole odkomentiraj, ko dodas fft.c in fft.h
                
                // Take the absolute value of the FFT results
                // Note: The FFT routine returns 'complex' numbers which consist of
                // both a real and imaginary part.  To find the 'absolute' value
                // of the returned data we have to calculate the complex number's
                // distance from zero which requires a little pythagoras and therefore
                // a square-root calculation too.  Since the PIC has multiplication
                // hardware, only the square-root needs to be optimised.

      for ( k=0; k < 32; k++)     // ce zelis nad vseh 32
        {       
        realNumbers[k] = (realNumbers[k] * realNumbers[k] + imaginaryNumbers[k] * imaginaryNumbers[k]);
                   
            // Now we find the square root of realNumbers[k] using a very
            // fast (but compiler dependent) integer approximation:
            // (adapted from: http://www.codecodex.com/wiki/Calculate_an_integer_square_root)
            place = 0x40000000;
            root = 0;

                        if (realNumbers[k] >= 0) // Ensure we don't have a negative number
                        {
                                while (place > realNumbers[k]) place = place >> 2;

                                while (place)
                                {
                                        if (realNumbers[k] >= root + place)
                                        {
                                                realNumbers[k] -= root + place;
                                                root += place * 2;
                                        }
                                        root = root >> 1;
                                        place = place >> 2;
                                }
                        }
                        realNumbers[k] = root;
            }
             
            // Now we have 32 buckets of audio frequency data represented as
            // linear intensity in realNumbers[]
            // Since the maximum input value (in theory) to the SQRT function is
            // 32767, the peak output at this stage is SQRT(32767) = 181.
 
        }*/


  
void loop(){
//send the latest data acquired when the button is pushed. 
      //if(digitalRead(PB0) == 1 ) {
      //Serial.println("begin");
      // Take our samples
      //delay(100);
      //digitalWrite(BOARD_LED, LOW);
      //for(unsigned int i = 0; i < maxSamples; i ++) {
        //Serial1.print("sample[");
        //Serial1.print(i);
        //Serial1.print("] = ");
        //Serial1.println(dataPoints[i]>>2);
        //Serial1.print("filout2=");
        //Serial1.println(filout2[0]);
        //Serial1.print("zacasna=");
        //Serial1.println(zacasna);
        //Serial2.println(outbuferR[0]);       
      //}         
        //}        
      //while(digitalRead(PB0) == 1); //stay here.  
      //}
      //digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
  inData = 0;
  if(Serial1.available() > 0)   // see if incoming serial data:
  {
    inData = Serial1.read();  // read oldest byte in serial buffer:
  }
  if(inData == 'A')
  {
    inData = 0;
     if (ssbmode == 1) {
      freq = freq - MFFREQ;
      sendFrequency(freq);
                      }  
    ssbmode=2;
  }

  /*if(inData == 'B')
  {
    freqmax = Serial1.parseInt();
    //Serial.print("freqmax= ");
    //Serial.println(freqmax);
    inData = 0;
  }

  if(inData == 'C')
  {
    boolean running1 = true;
    inData = 0; 
    korak = (freqmax - freqmin)/280;
    while(running1)
    {  
    freq = freqmin;
        Serial1.print("cle 3,0");
        Serial1.write(255);
        Serial1.write(255);       
        Serial1.write(255);
      sweep ();
        if(Serial1.available() > 0)   // see if incoming serial data:
        {
          inData = Serial1.read();  // read oldest byte in serial buffer:
          if(inData == 'Q')
          {
            running1 = false;
          }
          inData = 0;
        }
     }
         //delay(200);              // wait for a second
   }*/
     
   if(inData == 'f')
  {
    freq = Serial1.parseInt();
    //Serial.print("freqmax= ");
    //Serial1.println(freq);
    inData = 0;
    if (ssbmode == 0) {freq = freq - MFFREQ;}
    if (ssbmode == 2) {freq = freq - MFFREQ;}
    sendFrequency(freq);
  }      

   if(inData == 'L')
  {
    inData = 0;
    if (ssbmode == 1) {
      freq = freq - MFFREQ;
      sendFrequency(freq);
                      }
    ssbmode=0;
  }

   if(inData == 'S')
  {
    inData = 0;
    if (scan == 0) {
      freqold = freq;
      freqmin = freq-400000;
      scan = 1;
      }else
      {
      freq = freqold;
      sendFrequency(freq);
      scan = 0;
      }                   
  }

  
   if(inData == 'U')
  {
    inData = 0;
    if (ssbmode == 0 || ssbmode == 2) {
      freq = freq + MFFREQ;
      sendFrequency(freq);
                      }
    ssbmode=1;
  }

   if (scan == 1) {      
      if (mytimer2 >= 30000) {
    //GPIOC_BASE->ODR ^= 0x2000; //toggle PC13
      freqold = freq;
      freqmin = freq-400000;
        sweep();
        freq = freqold;
        sendFrequency(freq);
        Serial1.print("cle 4,0");
        Serial1.write(255);
        Serial1.write(255);       
        Serial1.write(255);
        Serial1.print("addt 4,0,320");
        Serial1.write(255);
        Serial1.write(255);       
        Serial1.write(255);
        delay(5);    //potrebno da se pripravi za transparent mode
       for (k=320;k>0;k--){
        //Serial1.print("add 4,0,");
        //Serial1.write((outbuferfft[k]+88)>>1);
        Serial1.write(spekter[k]);
        //Serial1.write((unsigned char)(realNumbers[k]));
        }
        Serial1.write(255);
        Serial1.write(255);       
        Serial1.write(255);
        mytimer2 = 0;
        }
      
    }


  calcsignal();    
    if (mytimer >= 12000) {
      mytimer=0;       
    msignal = (mmax - 512)>>3;
    if (msignal > 90){msignal = 90;}
       // while ((*SCICTL2 & 0x0040) == 0){znak=1;}
      //*SCITXBUF=(signal + 0x30);        //signal
        //Serial1.print("cle 3,0");
        Serial1.print("j0.val=");
        Serial1.print(msignal); 
        Serial1.write(255);
        Serial1.write(255);       
        Serial1.write(255);
    mmax = 512;            
    }
    
         
      //talejeGPIOC_BASE->ODR ^= 0x2000; //toggle PC13
}; //end loop

