#include <MsTimer2.h>

#define offset 0 //assume there is a sensor, which scale and offset the signal
#define samp_freq 800
#define samp_data_NO 400
#define vol_port 3 //3 as input to sample voltage
#define cur_port 4 //4 input to sample current

static unsigned char samp_flag = 0;
const PROGMEM double scale = 0.004883; //5 / 1024
const PROGMEM long samp_time = 1/samp_freq * 1000; //ms
int i=0;
double vReal[samp_data_NO];
double cReal[samp_data_NO];
double *vSamp_add = vReal;
double *cSamp_add = cReal;
static int tick = 0; // count how many samples being sampled
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

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);

MsTimer2::set(samp_time, sampling); //set interrupt, with samp_time as sampling period
MsTimer2::start();
}

void loop() {
  // put your main code here, to run repeatedly:
Serial.println("SD=");
for(i=0;i<samp_data_NO;i++)
{
Serial.println(vReal[i]);
}
Serial.println("endSP");
Serial.println("");
}
