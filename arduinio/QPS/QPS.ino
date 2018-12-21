#include <MsTimer2.h>
#include <LiquidCrystal.h>

#define offset 512 //assume there is a sensor, which scale and offset the signal
#define sig_freq 50
#define samp_freq 800
#define samp_data_NO 400
#define vol_port 3 //3 as input to sample voltage
#define cur_port 4 //4 input to sample current

LiquidCrystal lcd(8,9,4,5,6,7);
int i=0;
const PROGMEM int oneP_NO= samp_freq/sig_freq;
static unsigned char samp_flag = 0;
const PROGMEM double scale = 0.004883; //5 / 1024
const PROGMEM long samp_time = 1/samp_freq * 1000; //ms
double vReal[samp_data_NO];
double cReal[samp_data_NO];
double *vSamp_add = vReal;
double *cSamp_add = cReal;
static int tick = 0; // count how many samples being sampled
double vReal_scaled[samp_data_NO],cReal_scaled[samp_data_NO];
double S_pow, P_pow, Q_pow;
float RMS_vol = 0, RMS_cur = 0;

void sampling()
{
*vSamp_add = (analogRead(vol_port) - offset)*scale; // sample voltage
*cSamp_add = (analogRead(cur_port) - offset)*scale; //sample current

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

double RMS_cal(double *input)
{
double sum=0.0;
for(i=0;i<oneP_NO;i++)
{
sum += sq(input[i]);
}
sum = sqrt(sum/oneP_NO);
return sum;
}

double P_cal(double *vol, double *cur)
{
  double p_pow;
  double muti = double(sig_freq)/double(samp_freq);
  for(i=0;i<oneP_NO;i++)
  {
    p_pow+=vol[i]*cur[i];
  }
  p_pow=p_pow *muti;
  return p_pow;
}

double S_cal(double VRMS, double IRMS)
{
return VRMS * IRMS;
}

double Q_cal(double qpow, double spow)
{
return sqrt(sq(spow)-sq(qpow));
}

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
lcd.begin(16, 2);  
lcd.print("hello,Arduino!");
lcd.setCursor(0,1);
lcd.print("initialized");
delay(500);

MsTimer2::set(samp_time, sampling); //set interrupt, with samp_time as sampling period
MsTimer2::start();
}

void loop() {
  // put your main code here, to run repeatedly:
memcpy(vReal_scaled,vReal,sizeof(vReal));
memcpy(cReal_scaled,cReal,sizeof(cReal));
/*
lcd.clear();
lcd.print("RMS:"); 
delay(500);
RMS_vol = RMS_cal(vReal_scaled);
RMS_cur = RMS_cal(cReal_scaled);
Serial.println(RMS_vol);
Serial.println(RMS_cur);
lcd.setCursor(7,0);
lcd.print("V=");
lcd.print(RMS_vol);
lcd.setCursor(0,1);
lcd.print("I=");
lcd.print(RMS_cur);
delay(500);
*/
// power display
lcd.clear();
lcd.print("POW:");
delay(2);
RMS_vol = RMS_cal(vReal_scaled);
RMS_cur = RMS_cal(cReal_scaled);
P_pow = P_cal(vReal_scaled,cReal_scaled);
S_pow = S_cal(RMS_vol,RMS_cur);
Q_pow = Q_cal(P_pow, S_pow);
lcd.setCursor(7,0);
lcd.print("S=");
lcd.print(S_pow);
lcd.setCursor(0,1);
lcd.print("P=");
lcd.print(P_pow);
lcd.setCursor(7,1);
lcd.print("Q=");
lcd.print(Q_pow);
delay(500);
}
