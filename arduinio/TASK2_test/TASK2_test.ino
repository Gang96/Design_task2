#include <arduinoFFT.h>
#include <LiquidCrystal.h>
#include <MsTimer2.h>
#define sig_freq 4
#define v_offset 0 //assume there is a sensor, which scale and offset the signal
#define c_offset 0 //assume there is a sensor, which scale and offset the signal
#define samp_freq 1000
#define samp_data_NO 256  //2*12,(2T)signal frequency as 50Hz, 24 times of sig_freq, 20*50/50=20
#define vol_port 13 //13 as input to sample voltage
#define cur_port 14 //14 input to sample current
#define output 26 //26 output to control conpensation

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
// select the pins used on the LCD panel
const PROGMEM int oneP_NO= samp_freq/sig_freq;
const PROGMEM double scale = 0.004883; //5 / 1024
static int tick = 0; // count how many samples being sampled
int i=0;
const PROGMEM long samp_time = 1/samp_freq * 1000; //ms
//Arduino (Atmega) pins default to inputs

arduinoFFT FFT = arduinoFFT(); /* Create FFT object */
static unsigned char samp_flag = 0;
static unsigned char finish_flag = 0;
double vReal[samp_data_NO];
double cReal[samp_data_NO];
double Imag[samp_data_NO] = {0.0};
//double Imag[samp_data_NO] = {0.0};
double *vSamp_add = vReal;
double *cSamp_add = cReal;
/******************sample data every samp_time ms**********************/
int vol_samp, cur_samp;
int vol_max_val, cur_max_val;
int fake_count=0;
void sampling()
{

*vSamp_add = (analogRead(vol_port) - v_offset)*scale; // sample voltage
*cSamp_add = (analogRead(cur_port) - c_offset)*scale; //sample current
//*vSamp_add = (1024 - v_offset)*scale; // sample voltage
//*cSamp_add = (206 - c_offset)*scale; //sample current
//*vSamp_add = 5*sqrt(2)*sin(314*tick/samp_data_NO); // sample voltage
//*cSamp_add = 5*sqrt(2)*sin(314*tick/samp_data_NO+0.785); //sample current

tick++; vSamp_add++; cSamp_add++; //shit pointer and counter operation
if(tick == samp_data_NO)
{
vSamp_add = vReal; //point to the head
cSamp_add = cReal;
tick=0; //clearing counting
samp_flag = 1; //flag to tell enough data
MsTimer2::stop(); // enough data, stop sampling
}
}
/*****************data offset remove****************************/
double *data_offset_remove(double *raw_data,int data_len,int maximum )
{
double data[samp_data_NO];
double divisor = double(maximum) * scale/2;
for(i=0;i<data_len;i++)
{
data[i] = raw_data[i] - divisor;
}
}

/***************************************************************/
//RMS needed
float RMS_vol = 0, RMS_cur = 0;
float vol_temp, cur_temp;
/*************calculate RMS*******************/

double RMS_cal(double *input)
{
double sum=0.0;
//Serial.println("inRMS");
for(i=0;i<oneP_NO;i++)
{
sum += sq(input[i]);
//Serial.println(input[i]);
}
sum = sqrt(sum/oneP_NO);
//Serial.println("end");
//Serial.println("");
return sum;
}
/*******************real power***************************/
double P_cal(double *vol, double *cur)
{
//Serial.println("inP");
//Serial.println(muti);
//Serial.println("");
double p_pow;
for(i=0;i<oneP_NO;i++)
{
p_pow += vol[i]*cur[i];
//Serial.println(p_pow);
}
//Serial.println("");
p_pow = p_pow/oneP_NO;
//Serial.println(p_pow);
//Serial.println("end");
return p_pow;
}
/*************apparent power****************************/
double S_cal(double VRMS, double IRMS)
{
return VRMS * IRMS;
}
/*************q power****************************/
double Q_cal(double qpow, double spow)
{
return sqrt(sq(spow)-sq(qpow));
}
/*******************power factor****************************/
double PF_cal(double p_pow,double s_pow)
{
double tempPF=0;tempPF = p_pow/s_pow;
if(tempPF>1) tempPF=1;
if(tempPF<0) tempPF=0;
return tempPF;
}
//PF needed
/*******************cal PF***************************/
float PF;

//PQS needed
double S_pow, P_pow, Q_pow;
double S_pow1m, P_pow1m, Q_pow1m;
double S_pow3m, P_pow3m, Q_pow3m;

/******************find the harmonic peaks********************/
int peak_index[4]={0};
double harmonics[4]={0};
void find_peaks(double *input,int len)
{
double temp1=0,temp2=0,temp3=0,temp4=0;
double divisor = len/2;
//1st harmonic
for(i=1;i<len/2;i++)
{
if(input[i]>temp1){
temp1 = input[i];
peak_index[0]=i;
}
}
harmonics[0]=temp1/divisor;
//3rd harmonic
for(i=peak_index[0]+2;i<len;i++)
{
if(input[i]>temp2)
{
temp2 = input[i];
peak_index[1]=i;
}
}
harmonics[1]=temp2/divisor;
//5th harmonic
for(i=peak_index[1]+2;i<len;i++)
{
if(input[i]>temp3)
{
temp3 = input[i];
peak_index[2]=i;
}
}
harmonics[2]=temp3/divisor;
//7th harmonic
for(i=peak_index[2]+2;i<len;i++)
{
if(input[i]>temp4)
{
temp4 = input[i];
peak_index[3]=i;
}
}
harmonics[3]=temp4/divisor;
}
//harmonics needed

/**********************cal harmonics**************************/
//use after scaling
void cal_harmonics(double *real, double *image) //max can be cur or vol
{
//Serial.println("inFFT");
FFT.Windowing(real, samp_data_NO, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
FFT.Compute(real, image, samp_data_NO, FFT_FORWARD); /* Compute FFT */
FFT.ComplexToMagnitude(real, image, samp_data_NO); /* Compute magnitudes */
find_peaks(real, samp_data_NO/2);
//Serial.println("end");
//Serial.println(harmonics[0]);
//Serial.println(harmonics[1]);
//Serial.println(harmonics[2]);
//Serial.println(harmonics[3]);
//Serial.println("");
for(i=0;i<samp_data_NO;i++)
{
//Serial.println(real[i]);
real[i]=0;
image[i]=0;
}
//Serial.println("end1");

}
/*******************************cal thd******************************/
//used after harmonics
double thd()
{
//Serial.println("inTHD");
double res =0;
res = sqrt((sq(harmonics[1])+ sq(harmonics[2]) +
sq(harmonics[3])))/harmonics[0];
//Serial.println(res);
//Serial.println("end");
//Serial.println("");
return res;
}
/*******************************cal thd******************************/
float DPF(float thd_cur)
{
return 1/sqrt(1+sq(thd_cur));
}
/*******************************************************************/
void setup() {
// put your setup code here, to run once:
lcd.begin(16, 2);              // start the library
pinMode(26, OUTPUT);
//Serial.begin(9600);
//lcd operation
lcd.print("hello,Arduino!");
lcd.setCursor(0,1);
lcd.print("initialized");
digitalWrite(output,HIGH);
delay(500);
// timer interrupt
MsTimer2::set(samp_time, sampling); //set interrupt, with samp_time as sampling period
MsTimer2::start();
}
int test_count=0;
int mode_switch=1;
const int buttonPin = 0;

double vReal_scaled[samp_data_NO],cReal_scaled[samp_data_NO];
float DFT_cal;
double thdV, thdI;
double P_eng=0, Q_eng=0;
double P_eng1m=0, Q_eng1m=0;
double P_eng3m=0, Q_eng3m=0;
double time_begin=0,time_now,time_diff =0,time_last =0;
long time30s = 30000;
long time1min = 60000;
long time3min = 180000;
long tEnd=0;
void loop() {
// put your main code here, to run repeatedly:
//Serial.println("LOOP_IN");
//Serial.println(mode_switch);
if(samp_flag)
{
//Serial.println("SD=");
for(i=0;i<samp_data_NO;i++)
{
//Serial.println(vReal[i]);
}
//Serial.println("endSP");
//Serial.println("");
memcpy(vReal_scaled,vReal,sizeof(vReal));
memcpy(cReal_scaled,cReal,sizeof(cReal));


int reading = analogRead(buttonPin);
   if(reading<10){
     //Read switch up
     mode_switch++;
     if(mode_switch==9)
     mode_switch = 8;
   }else if(reading>300 && reading<500){
     //Read switch down
     mode_switch--;
     if(mode_switch==0)
     mode_switch = 1;}
     else if (reading>50 && reading<200){
     digitalWrite(output,LOW);
     }
     else if (reading>200 && reading<300){
     digitalWrite(output,HIGH);
     }
     else{
     //No button is pressed;
   }

//Serial.println(26);
   

switch(mode_switch)
{
case 1: //rms
//Serial.println("rms:");
//Serial.println(oneP_NO);
lcd.clear();
lcd.print("RMS:"); //wehter to stop timer
//delay(500);
RMS_vol = RMS_cal(vReal_scaled);
RMS_cur = RMS_cal(cReal_scaled);
//lcd.clear();
//lcd.home(); //start form top left
lcd.setCursor(5,0);
lcd.print("V(mV)=");
lcd.print(RMS_vol*1000);
lcd.setCursor(0,1);
lcd.print("I(mA)=");
lcd.print(RMS_cur*1000);
//delay(500);
finish_flag = 1;
break;
case 3: //pf
lcd.clear();
lcd.print("PF:");
RMS_vol = RMS_cal(vReal_scaled);
RMS_cur = RMS_cal(cReal_scaled);
P_pow = P_cal(vReal_scaled,cReal_scaled);
//Serial.println("P=");
//Serial.println(P_pow);
S_pow = S_cal(RMS_vol,RMS_cur);
if(reading>600&&reading<800){
     Q_pow = 0;
     S_pow = P_pow;
   }
//Serial.println("S=");
//Serial.println(S_pow);
PF = PF_cal(P_pow,S_pow);
lcd.setCursor(0,1);
lcd.print("pf=");
lcd.print(PF);
//delay(500);
finish_flag = 1;
break;
case 4: //dpf
lcd.clear();
lcd.print("DPF:");
//vReal_scaled = data_scale(vReal,samp_data_NO);
//cReal_scaled = data_scale(cReal,samp_data_NO);
cal_harmonics(cReal_scaled, Imag);
DFT_cal = DPF(thd());
lcd.setCursor(5,0);
lcd.print("dist=");
lcd.print(DFT_cal);
lcd.setCursor(5,1);
lcd.print("disp=");
lcd.print(PF/DFT_cal);

//delay(300);
finish_flag = 1;
break;
case 2: //powerlcd.clear();
lcd.clear();
lcd.print("POW:");
delay(2);
RMS_vol = RMS_cal(vReal_scaled);
RMS_cur = RMS_cal(cReal_scaled);
P_pow = P_cal(vReal_scaled,cReal_scaled);
S_pow = S_cal(RMS_vol,RMS_cur);
Q_pow = Q_cal(P_pow, S_pow);
if(reading>600&&reading<800){
     Q_pow = 0;
     S_pow = P_pow;
   }
lcd.setCursor(5,0);
lcd.print("S=");
lcd.print(S_pow*1000);
lcd.setCursor(0,1);
lcd.print("P=");
lcd.print(P_pow*1000);
lcd.setCursor(7,1);
lcd.print("Q=");
lcd.print(Q_pow*1000);
//delay(500);
finish_flag = 1;
break;
case 5: //THD
lcd.clear();
lcd.print("THD:");
cal_harmonics(vReal_scaled, Imag);
thdV = thd();
cal_harmonics(cReal_scaled, Imag);
thdI = thd();
lcd.setCursor(5,0);
lcd.print("thdV=");
lcd.print(thdV);
lcd.setCursor(5,1);
lcd.print("thdI=");
lcd.print(thdI);
//delay(500);
finish_flag = 1;
break;
case 6: //energy
time_begin = millis();
//Serial.println("tB=");
//Serial.println((time_begin/1000));
time_last = time_begin;
while(time_diff<time30s)
{
if(samp_flag)
{
lcd.clear();
lcd.print("please waite 30s");
lcd.setCursor(0,1);
lcd.print("time=");
lcd.print(time_diff/1000);
RMS_vol = RMS_cal(vReal_scaled);
RMS_cur = RMS_cal(cReal_scaled);
P_pow = P_cal(vReal_scaled,cReal_scaled);
S_pow = S_cal(RMS_vol,RMS_cur);
Q_pow = Q_cal(P_pow, S_pow);
if(reading>600&&reading<800){
     Q_pow = 0;
     S_pow = P_pow;
   }
time_now = millis();
P_eng += P_pow * (time_now-time_last)/1000;
Q_eng += Q_pow * (time_now-time_last)/1000;
time_last=time_now;
time_diff = time_now - time_begin;
//Serial.println(test_count++);
}
samp_flag=0;
MsTimer2::start();
delay(2);
}
tEnd = millis()/1000;
//Serial.println("tE=");
//Serial.println(tEnd);
lcd.clear();
lcd.print("E30:");
lcd.setCursor(5,0);
lcd.print("PE=");
lcd.print(P_eng);
lcd.setCursor(5,1);
lcd.print("QE=");
lcd.print(Q_eng);
//delay(1000);
finish_flag = 1;
break;
case 7:
time_begin = millis();
//Serial.println("tB=");
//Serial.println((time_begin/1000));
time_last = time_begin;
while(time_diff<time1min)
{
if(samp_flag)
{
lcd.clear();
lcd.print("please waite 1m");
lcd.setCursor(0,1);
lcd.print("time=");
lcd.print(time_diff/1000);
RMS_vol = RMS_cal(vReal_scaled);
RMS_cur = RMS_cal(cReal_scaled);
P_pow1m = P_cal(vReal_scaled,cReal_scaled);
S_pow1m = S_cal(RMS_vol,RMS_cur);
Q_pow1m = Q_cal(P_pow1m, S_pow1m);
time_now = millis();
P_eng1m += P_pow1m * (time_now-time_last)/1000;
Q_eng1m += Q_pow1m * (time_now-time_last)/1000;
time_last=time_now;
time_diff = time_now - time_begin;
//Serial.println(test_count++);
}
samp_flag=0;MsTimer2::start();
delay(2);
}
tEnd = millis()/1000;
//Serial.println("tE=");
//Serial.println(tEnd);
lcd.clear();
lcd.print("E1m:");
lcd.setCursor(5,0);
lcd.print("PE=");
lcd.print(P_eng1m);
lcd.setCursor(5,1);
lcd.print("QE=");
lcd.print(Q_eng1m);
//delay(1000);
finish_flag = 1;
break;
case 8:
time_begin = millis();
//lcd.clear();
//lcd.print("tB:");
//lcd.print(time_begin/1000);
//Serial.println("tB=");
//Serial.println((time_begin/1000));
time_last = time_begin;
while(time_diff<time3min)
{
if(samp_flag)
{
lcd.clear();
lcd.print("please waite 3m");
lcd.setCursor(0,1);
lcd.print("time=");
lcd.print(time_diff/1000);
RMS_vol = RMS_cal(vReal_scaled);
RMS_cur = RMS_cal(cReal_scaled);
P_pow3m = P_cal(vReal_scaled,cReal_scaled);
S_pow3m = S_cal(RMS_vol,RMS_cur);
Q_pow3m = Q_cal(P_pow3m, S_pow3m);
time_now = millis();
P_eng3m += P_pow3m * (time_now-time_last)/1000;
Q_eng3m += Q_pow3m * (time_now-time_last)/1000;
time_last=time_now;
time_diff = time_now - time_begin;
//Serial.println(test_count++);
}
samp_flag=0;
MsTimer2::start();
delay(2);
}
tEnd = millis()/1000;
//Serial.println("tE=");
//Serial.println(tEnd);
lcd.clear();
lcd.print("E3m:");
lcd.setCursor(5,0);
lcd.print("PE=");
lcd.print(P_eng3m);
lcd.setCursor(5,1);
lcd.print("QE=");
lcd.print(Q_eng3m);
//delay(1000);
finish_flag = 1;
break;
default:
lcd.clear();
lcd.print("pls select function");
//delay(500);
break;
}
}
if(finish_flag)
{
MsTimer2::start(); //restart sampling
samp_flag =0;finish_flag =0;
}
}
