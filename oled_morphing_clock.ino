/*
 * Morphing Clock Demo
 * (c)2018 Pawel A. Hernik
 * code for https://youtu.be/PxVekSvcsgQ
 */
 
//#include <OLEDSoftI2C.h>
#include <OLEDSoftI2C_SH1106.h>
// define USEHW in above header for hw I2C version

//OLEDSoftI2C oled(0x3c);
OLEDSoftI2C_SH1106 oled(0x3c);

#if USEHW==1
#include <Wire.h>
#endif

#include "modern_dig_15x32_font.h"
#include "seg7_dig_15x32_font.h"

const uint8_t *font = modern_dig_15x32;
//const uint8_t *font = seg7_dig_15x32;

// --------------------------------------------------------------------------
byte buf[15*4];
byte bufWd = 15;
byte bufHt = 4;

void clrBuf()
{
  int ii=0;
  for(int i=0;i<bufWd*bufHt/4;i++) { buf[ii++]=0; buf[ii++]=0; buf[ii++]=0; buf[ii++]=0; }
}
// --------------------------------------------------------------------------
void drawPixel(int16_t x, int16_t y, uint16_t color) 
{
  if(x<0 || x>=bufWd || y<0 || y>=bufHt*8) return;

  switch(color) {
    case 1: buf[x+(y/8)*bufWd] |=  (1 << (y&7)); break;
    case 0: buf[x+(y/8)*bufWd] &= ~(1 << (y&7)); break; 
    case 2: buf[x+(y/8)*bufWd] ^=  (1 << (y&7)); break; 
  }
}
// --------------------------------------------------------------------------

int genPoints(byte *tab, int max, const uint8_t* font, char c)
{
  uint8_t xSize    =-pgm_read_byte(font+0);
  uint8_t ySize    = pgm_read_byte(font+1);
  uint8_t offs     = pgm_read_byte(font+2);
  uint8_t numChars = pgm_read_byte(font+3);
  uint8_t ySize8 = (ySize+7)/8;
  uint8_t wd, ch = oled.convertPolish(c);
  if(!font || ch<offs || ch>=offs+numChars) return 0;
  int v;
  int num = 0;
  int i, idx = 4 + (ch - offs)*(xSize*ySize8+1)+1;
  //wd = pgm_read_byte(font + idx++);
  wd = xSize;
  for(int j=0; j<ySize8; j++) {
    for(i=0; i<wd; i++)  {
      v = pgm_read_byte(font+idx+i*ySize8+j);
      byte mask = 1;
      for(int b=0;b<8;b++) {
        if(v & mask) {
          tab[num*2+0] = i;
          tab[num*2+1] = b+j*8;
          if(++num>=max) { Serial.println("Size too big for: "+String(ch)); return num; }
        }
        mask<<=1;
      }
    }
  }
  return num;
}

// --------------------------------------------------------------------------

void drawPoints(int x, int y, byte *from, int num)
{
  for(int i=0;i<num;i++) drawPixel( x+from[i*2], y+from[i*2+1], 1 );
}

// --------------------------------------------------------------------------

#define TAB_MAX 100
uint8_t src[TAB_MAX*2];
uint8_t dst[TAB_MAX*2];
int numSrc=0;
int numDst=0;
int tt=0;

unsigned int h,m,s;
unsigned long startTime=16*3600L+58*60+50;
unsigned long startMillis;
char digits[7];
char digitsOld[7];
int digitX[8];

void tick()
{
  unsigned long tim=startTime+(millis()-startMillis)/1000;
  h=tim/3600;
  m=(tim-h*3600)/60;
  s=tim-h*3600-m*60;
  strcpy(digitsOld,digits);
  snprintf(digits,7,"%02u%02u%02u",h,m,s);
  //Serial.println(digits);
}

void morph(int digX, int mode)
{
  int i,i0,i1,x,y;
  int n = max(numSrc,numDst);
  clrBuf();
  for(int i=0;i<n;i++) {
    i0 = (numSrc<n) ? i*numSrc/n : i;
    i1 = (numDst<n) ? i*numDst/n : i;
    if(mode&1) i1 = numDst-1-i1;
    x = src[i0*2]+(dst[i1*2]-src[i0*2])*tt/32;
    y = src[i0*2+1]+(dst[i1*2+1]-src[i0*2+1])*tt/32;
    drawPixel(x,y,1);
  }
  oled.drawBuf(buf,digX,2,bufWd,bufHt);
}

void showClock()
{
  for(int i=0;i<8;i++) oled.printChar(digitX[i], 2, i<6 ? digits[i] : ':');
}

void setup()
{
  Serial.begin(9600);
  oled.init();
  oled.clrScr();
  oled.setFont(font);
  oled.setDigitMinWd(16);
  startMillis=millis();
  // 17+17+6+17+17+6+17+17
  int x = 6;
  digitX[0]=x;
  digitX[1]=17+digitX[0];
  digitX[6]=17+digitX[1];
  digitX[2]= 7+digitX[6];
  digitX[3]=17+digitX[2];
  digitX[7]=17+digitX[3];
  digitX[4]= 7+digitX[7];
  digitX[5]=17+digitX[4];
  showClock();
}


int mode = 0;
void loop()
{
  // morph demo
  tick();
  for(tt=0;tt<=32;tt++) {
    unsigned long morphStart = millis();    
    // one buffer for all digits - not optimal and slow but saves memory
    for(int i=0;i<6;i++) {
      if(digits[i]!=digitsOld[i]) {
        numSrc = genPoints(src,TAB_MAX,font,digitsOld[i]);
        numDst = genPoints(dst,TAB_MAX,font,digits[i]);
        morph(digitX[i],mode);
      }
    }
    while(millis()-morphStart<10) {}
  }
  if(m==00 && s>11) { // next mode at 17:00:11
    mode=(mode+1)&3;
    font = mode<2 ? modern_dig_15x32 : seg7_dig_15x32;
    startMillis=millis();
    startTime=16*3600L+59*60+35;
    tick();
    strcpy(digitsOld,digits);
    oled.setFont(font);
    showClock();
  }
}

