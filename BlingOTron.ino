#include <TimerOne.h>
#include <SPI.h>
#include "Adafruit_WS2801.h"



int dataPin  = 8;    // Yellow wire on Adafruit Pixels
int clockPin = 7;    // Green wire on Adafruit Pixels
#define stripLength 15

Adafruit_WS2801 zeeStrip(15, dataPin, clockPin, WS2801_GRB);
uint32_t lerpColor(uint32_t colorA, uint32_t colorB, int lerp, int lerpMax);


class BeatOverlay
{
public:
  BeatOverlay(Adafruit_WS2801 *pStrip)
  {
    m_pStrip = pStrip;
    Reset();
    m_pStrip->begin();
  }
  
  void begin(void)
  {
    m_pStrip->begin();
  }
  
  void Reset(void)
  {
    for (int i=0; i<stripLength; ++i)
    {
      m_aLerping[i] = false;
    }
  }
  
  void show(bool noBaseLineChange = false)
  {
    bool lerpApplied = false;
    
    // The actual colors are the ones set by the effects plus any beat overlay effect
    for (int i=0; i<stripLength; ++i)
    {
      if (m_aLerping[i])
      {
        lerpApplied = true;
        uint32_t elapsed = millis() - m_aLerpStart[i];
        if (elapsed <= m_aLerpIn[i])
        {
          // We're in the lerp-in phase
          uint32_t color = lerpColor(m_aBaseColors[i], m_aLerpColors[i], elapsed, m_aLerpIn[i]);
          m_pStrip->setPixelColor(i, color);
        }
        else if (elapsed < m_aLerpIn[i] + m_aLerpDwell[i])
        {
          // We're in the dwell phase
          m_pStrip->setPixelColor(i, m_aLerpColors[i]); 
        }
        else if (elapsed <= m_aLerpIn[i] + m_aLerpDwell[i] + m_aLerpOut[i])
        {
          // We're in the lerp-out phase
          uint32_t color = lerpColor(m_aLerpColors[i], m_aBaseColors[i], elapsed-m_aLerpIn[i]-m_aLerpDwell[i], m_aLerpOut[i]);
          m_pStrip->setPixelColor(i, color);
        }
        else
        {
          // Actually we're all done here. Go base to baseline color
          m_aLerping[i] = false;
          m_pStrip->setPixelColor(i, m_aBaseColors[i]);
        }
      }
      else
      {
        m_pStrip->setPixelColor(i, m_aBaseColors[i]);
      }
    }
    
    m_pStrip->show();
  }

  void SetLerpColor(int index, uint32_t color, int millisIn, int millisDwell, int millisOut)
  {
    if (index < 0 || index >= stripLength)
    {
      return;
    }
    
    m_aLerpColors[index] = color;
    m_aLerpStart[index] = millis();
    m_aLerpIn[index] = millisIn;
    m_aLerpDwell[index] = millisDwell;
    m_aLerpOut[index] = millisOut;
    m_aLerping[index] = true;
  }  

  void setPixelColor(uint16_t n, uint32_t c)
  {
    if (n < 0 || n >= stripLength)
    {
      return;
    }
    m_aBaseColors[n] = c;
    m_pStrip->setPixelColor(n, c);
  }

private:
  uint32_t         m_aBaseColors[stripLength];
  uint32_t         m_aLerpColors[stripLength];
  uint32_t         m_aLerpStart[stripLength];
  int              m_aLerpIn[stripLength];
  int              m_aLerpOut[stripLength];
  int              m_aLerpDwell[stripLength];
  bool             m_aLerping[stripLength];
  Adafruit_WS2801 *m_pStrip;
};

BeatOverlay strip(&zeeStrip);


uint32_t hsv2rgb(float hue, float sat, float val)
{
    double      hh, p, q, t, ff;
    long        i;
    int         red, green, blue;
    
    hh = hue;

    if(sat <= 0.0) 
    {       
        red   = val;
        green = val;
        blue  = val;
    }
    else
    {
      if(hh >= 360.0) hh = hh-360.0;
      hh /= 60.0;
      i = (long)hh;
      ff = hh - i;
      p = val * (1.0 - sat);
      q = val * (1.0 - (sat * ff));
      t = val * (1.0 - (sat * (1.0 - ff)));
  
      switch(i) {
      case 0:
          red = val * 255.0;
          green = t * 255.0;
          blue = p * 255.0;
          break;
      case 1:
          red = q * 255.0;
          green = val * 255.0;
          blue = p * 255.0;
          break;
      case 2:
          red = p * 255.0;
          green = val * 255.0;
          blue = t * 255.0;
          break;
  
      case 3:
          red = p * 255.0;
          green = q * 255.0;
          blue = val * 255.0;
          break;
      case 4:
          red = t * 255.0;
          green = p * 255.0;
          blue = val * 255.0;
          break;
      case 5:
      default:
          red = val  * 255.0;
          green = p * 255.0;
          blue = q * 255.0;
          break;
      }
    }
   uint32_t result = (((uint32_t)red) << 16) | 
                     (((uint32_t)green) << 8) |
                     (((uint32_t)blue));
                     
   if (0)
   {
     Serial.print("HSV Color = ");
     Serial.print(hue);
     Serial.print(",");
     Serial.print(sat);
     Serial.print(",");
     Serial.print(val);
     Serial.println(")");
     
     Serial.print("RGB Color = ");
     Serial.print(red);
     Serial.print(",");
     Serial.print(green);
     Serial.print(",");
     Serial.print(blue);
     Serial.println(")");
     
     Serial.print("0x00");
     Serial.println(result, HEX);
     Serial.println("");
   }
   return result;
}
          
          
uint32_t lerpColor(uint32_t colorA, uint32_t colorB, int lerp, int lerpMax)
{
//  Serial.print("LERP "); Serial.print(colorA, HEX); Serial.print(" -> "); Serial.print(colorB, HEX); Serial.print("  ("); Serial.print(lerp); Serial.print("/"); Serial.println(lerpMax);
  // Extract colors to ints
  uint32_t color1[3];
  uint32_t color2[3];
  
  color1[0] = (colorA>>16) & 255;
  color1[1] = (colorA>>8) & 255;
  color1[2] = colorA & 255;
  
  color2[0] = (colorB>>16) & 255;
  color2[1] = (colorB>>8) & 255;
  color2[2] = colorB & 255;
  
   uint32_t r = ((lerpMax-lerp)*color1[0] + lerp*color2[0]) / lerpMax;
   uint32_t g = ((lerpMax-lerp)*color1[1] + lerp*color2[1]) / lerpMax;
   uint32_t b = ((lerpMax-lerp)*color1[2] + lerp*color2[2]) / lerpMax;
   
   uint32_t result = (((uint32_t)r) << 16) | 
                     (((uint32_t)g) << 8) |
                     (((uint32_t)b));
   return result;
}
 
void renderSlab(float left, float right, float hue, float sat, float val)
{
  float alpha = 0.0;
  
  for (int index=floor(left); index<=floor(right); ++index)
  {
    if (index < floor(left))
    {
      // do nothing. shouldn't happen
      alpha = 0.0;
    }
    else if (index == floor(left) && index == floor(right))
    {
      // entire width is a semi-pixel
      alpha = right-left;
    }
    else if (index == floor(left))
    {
      // left semipxel portion
      alpha = 1.0-(left-(float)floor(left));
    }
    else if (index < floor(right))
    {
       // full pixel
       alpha = 1.0;
    }
    else if (index == floor(right))
    {
        // right semipxel portion
        alpha = right-(float)floor(right);
    }
    else
    {
      alpha = 0.0;
    }
    
    strip.setPixelColor(index, hsv2rgb(hue, sat, alpha*val));
  }
//  strip.show();
}

   
// ISR State
volatile bool beatLatch = false;
volatile bool beatAck = false;
volatile int audioMax = 0;
volatile int average = 0;
volatile int audioLevel = 0;
int localAudioMax = 0;
uint16_t eightSampleAverage = 0;
uint16_t oneSecondAverage = 0;
int sampleCount = 0;
bool localBeatTriggered = false;
volatile int beatTriggerLevel = 255;
volatile int beatFinishedLevel = 0;
volatile int lastAverage = 0;
volatile int lastMax = 0;
volatile bool showStats = false;


void xISR()
{
  int val = analogRead(A3);
  if (val > localAudioMax)
  {
    localAudioMax = val;
  }
  eightSampleAverage += val;
  ++sampleCount;
  
  // Are we looking for a beat?
  if (!localBeatTriggered)
  {
    // Yes -- does this qualify?
    if (val > beatTriggerLevel)
    {
      // Yes!
      localBeatTriggered = true;
      //Serial.println("X");
      
      // But does anyone care?
      if (beatAck)
      {
        // Yes. Main program is ready to listen to beats
        beatLatch = true;
        beatAck = false;
      }
    }
  }
  else
  {
   // We are in a beat. See if we should go out of it
   if (val < beatFinishedLevel)
   {
     localBeatTriggered = false; 
   } 
  }

  // Are we on an 8-sample boundary?
  if (0 == (sampleCount & 7))
  {
    // Yes. Divide by 8 and accumulate into one second average
    oneSecondAverage += eightSampleAverage >> 3;
    
    // And update the 8ms audio level for the main loop
    audioLevel = eightSampleAverage >> 3;
    audioMax = localAudioMax;
    
    // Clear for next accumulation
    eightSampleAverage = 0;
  }
  
  // Are we on a 1-second boundary?
  if (0 == (sampleCount & 1023))
  {
    // Yes. Store new average value
    lastAverage = average;
    lastMax = audioMax;
    average = oneSecondAverage >> 7;
    audioMax = localAudioMax;
    localAudioMax = 0;
    oneSecondAverage = 0;
    showStats = true;
    
    // Tune these params
    int delta = audioMax - average;
    int newBeatTriggerLevel = audioMax - (delta>>1);
    int newBeatFinishedLevel = average + (delta>>1) - (delta>>2);
    
       beatTriggerLevel = newBeatTriggerLevel;
      beatFinishedLevel = newBeatFinishedLevel;
   if (0)
    {
    if (newBeatTriggerLevel > beatTriggerLevel)
    {
      beatTriggerLevel = newBeatTriggerLevel;
      beatFinishedLevel = newBeatFinishedLevel;
    }
    else
    {
      beatTriggerLevel = newBeatTriggerLevel + ((beatTriggerLevel-newBeatTriggerLevel)>>1);
      beatFinishedLevel = newBeatFinishedLevel;
    }
    }
  }
}

void setup()
{
  Serial.begin(57600);
  Serial.println("Hello Cleveland...78rpm");
  int index;
  strip.begin();  
  strip.show();
  double hueScale = 360.0 / (stripLength+1);
  for (index=0; index<stripLength; ++index)
  {
    uint32_t color = hsv2rgb(hueScale * index, 1.0, 0.25);
    strip.setPixelColor(index, color); 
  }
  Serial.println("Showing LEDs");
  strip.show();
  
  #if 0
  Serial.println("Calibrating timing");
  uint32_t interval = millis() + 500;
  uint32_t iterations = 0;
  while (millis() < interval)
  {
    int val = analogRead(A3);
    ++iterations;
  }
  Serial.print("    reads: "); Serial.println(iterations);
  Serial.print(" reads/ms: "); Serial.println(iterations/500);
  Serial.println("\nMeasuing ISR speed");
  interval = millis() + 2500;
  iterations = 0;
  while (millis() < interval)
  {
    xISR();
    ++iterations;
  }
  Serial.print("    ISRs: "); Serial.println(iterations);
  Serial.print(" ISRs/ms: "); Serial.println(iterations/2500);
#endif

  randomSeed(analogRead(A3) + analogRead(A3) + analogRead(0));
  Timer1.initialize(1000); // 1ms timer
  Timer1.attachInterrupt(xISR);
}

// shared state
int frameIndex = 0;
int effectID = 0;
int framesRemaining = 0;


// hue cycling state
float startAngle = 0.0;
float tubeAngle = 100.0;
float oneLightAngle = tubeAngle / stripLength;
float maxAngle = 358.0-tubeAngle;
float frameAngleAdvance = oneLightAngle/3;

void HueCycle_Initialize()
{
  startAngle = 0.0;
}

int HueCycle_RenderFrame(int iFrameIndex)
{
  double hueScale = 360.0 / (stripLength+1);
  for (int index=0; index<stripLength; ++index)
  {
    uint32_t color = hsv2rgb(startAngle + oneLightAngle*index, 1.0, 0.20);
    strip.setPixelColor(index, color); 
  }
  strip.show();
  startAngle += frameAngleAdvance;
  if (startAngle >= 360.0) startAngle -= 360.0;
  
  return 2;
}


// raindrop state
float dropCenter;   // 2.0 - 15.0
float dropWidth;    // 1.5 -  6.5
int dropFrame;      // 0 - 25
int dropDelay;      // 10ms - 175ms
float dropHue;

void Raindrop_Initialize()
{
  dropCenter = (random(130)+20) * 0.10;
  dropWidth  = (random(100)+15)  * 0.10;
  dropFrame = 0;
  dropDelay = 7 ;
  dropHue = random(36)*10.0;
}


int Raindrop_RenderFrame(int iFrameIndex)
{
  int frame = dropFrame;
  int delayz = dropDelay;
  
  if (dropFrame > 10)
  {
    frame = 20-dropFrame;
  }
  
  float radius = dropWidth / 2.0 * ((float)frame / 10.0);
  float left = dropCenter - radius;
  float right = dropCenter + radius;
  //Serial.print("LEFT="); Serial.print(left); Serial.print("  RIGHT="); Serial.println(right);
  
  
  for (int index=0; index<stripLength; ++index)
  {
    if (index < floor(left))
    {
      strip.setPixelColor(index, 0x0);
    }
    else if (index == floor(left) && index == floor(right))
    {
      // entire width is a semi-pixel
      strip.setPixelColor(index, hsv2rgb(dropHue, 1.0, (right-left)*0.20));      
    }
    else if (index == floor(left))
    {
      // left semipxel portion
      strip.setPixelColor(index, hsv2rgb(dropHue, 1.0, (1.0-(left-(float)floor(left))) * 0.20));
    }
    else if (index < floor(right))
    {
       // full pixel
       strip.setPixelColor(index, hsv2rgb(dropHue, 1.0, 2.0));
    }
    else if (index == floor(right))
    {
        // left semipxel portion
        strip.setPixelColor(index, hsv2rgb(dropHue, 1.0, (right-(float)floor(right)) * 0.20));    
    }
    else
    {
      strip.setPixelColor(index, 0);
    }
  }
  strip.show();
  ++dropFrame;
  
  if (dropFrame == 21)
  {
    delayz = 25;
    Raindrop_Initialize();
  }
  
  return delayz;
}

#define CURTAIN_MAX 10
typedef struct
{
  // HSV color of curtain
  float  hue;
  float  sat;
  float  val;
  
  // Curtain width
  float  width;
  
  // Location of "gravity" for this curtain
  float  gravity;
  
  // ---- above are constant. below change per frame ---
  // Current location and velocity
  float  x;
  int    homeStay;
} TCurtainState;
TCurtainState aCurtains[CURTAIN_MAX];
int iCurtainCount = 0;
float fCenter = ((float)stripLength) / 2.0;

void Curtains_Initialize(void)
{
  // First pick how many curtains to create in the range of 2..10
  iCurtainCount = 5 + random(5);
  
  // Now initialize that many curtains
  for (int iIndex=0; iIndex<iCurtainCount; ++iIndex)
  {
   aCurtains[iIndex].hue = 10.0 * random(36);
   aCurtains[iIndex].sat = 1.0; //((float)random(800)) / 1000.0 + 0.19;
   aCurtains[iIndex].val = 0.2;
   
   // width in range of 1.0 - 7.0
   aCurtains[iIndex].width = ((float)random(60)) / 10.0 + 1.0;
   
   // random gravity location
   aCurtains[iIndex].gravity = ((float)random(stripLength * 100)) / 100.0 ;
   
   // random starting position.
   aCurtains[iIndex].x = ((float)random(stripLength * 100)) / 100.0 ;
  }
}

int Curtains_RenderFrame(int iFrameIndex)
{
  int iIndex;
  for (iIndex=0; iIndex<stripLength; ++iIndex)
  {

    strip.setPixelColor(iIndex, 0x0);  
  }

  for (iIndex=0; iIndex<iCurtainCount; ++iIndex)
  {
   // Render at current position first, then update
   float radius = aCurtains[iIndex].width / 2.0;
   float left = aCurtains[iIndex].x - radius;
   float right = aCurtains[iIndex].x + radius;
   
   renderSlab(left, right, aCurtains[iIndex].hue, aCurtains[iIndex].sat, aCurtains[iIndex].val);
     
   // This totally ignores velocity & acceleration.
   // Simply treats center as gravity and the further away, the stronger the pull
   float distance = aCurtains[iIndex].gravity - aCurtains[iIndex].x;
   float fudge = ((float)random(50)) / 50.0 + 0.75;

   // Velocity is enough to move to center in 5 frames
   // plus some fudge
   float velocity = (distance / 12.0) * fudge;
   aCurtains[iIndex].x += velocity;
   
   if (0)
   {
     Serial.print("Curtain #"); Serial.print(iIndex);
     Serial.print("   gravity = "); Serial.print(aCurtains[iIndex].gravity);
     Serial.print("   position = "); Serial.print(aCurtains[iIndex].x);
     Serial.print("   distance = "); Serial.print(distance);
     Serial.print("   velocity = "); Serial.print(velocity);
     Serial.print("   fudge = "); Serial.println(fudge);
   }
   
   // If the curtain has found home, then randomly position it again
   if (fabs(distance) < 0.5)
   {
     ++aCurtains[iIndex].homeStay;
     if (aCurtains[iIndex].homeStay >= 10)
     {
       aCurtains[iIndex].x = ((float)random(stripLength * 100)) / 100.0 ;
     }
   }
   else
   {
     aCurtains[iIndex].homeStay = 0; 
   }
  }
  
  strip.show();
  return 50;
}

float cylonHue;
float cylonWidth;
float cylonPosition;
float cylonVelocity;
float cylonSpeed;
int cylonEndDwell;

void Cylon_Initialize()
{
   cylonHue = 10.0 * random(36);
   cylonWidth = 1.0 + ((float)random(500) / 100.0);
   cylonPosition = 1.0;
   cylonSpeed = ((float)random(15)+1.0)/20.0;
   cylonVelocity = cylonSpeed;
   cylonEndDwell = random(2000);
}

int Cylon_RenderFrame(int)
{
  int iIndex;
  int delayz = 10;
  for (iIndex=0; iIndex<stripLength; ++iIndex)
  {
    strip.setPixelColor(iIndex, 0x0);  
  }
   // Render at current position first, then update
   float radius = cylonWidth / 2.0;
   float left = cylonPosition - radius;
   float right = cylonPosition + radius;
   
   renderSlab(left, right, cylonHue, 1.0, 0.2);
   
   if (cylonVelocity > 0.0)
   {
     // Moving right. Check if at end
     if (cylonPosition >= (float)stripLength)
     {
       // Yes. Dwell and then reverse
       delayz = cylonEndDwell;
       cylonVelocity = -cylonSpeed;
     }
   }
   else
   {
     // Moving left. Check if at end
     if (cylonPosition <= 0.0)
     {
       // Yes. Dwell and then reverse
       delayz = cylonEndDwell;
       cylonVelocity = cylonSpeed;
     }
   }

  cylonPosition += cylonVelocity;
  strip.show();
  
  return delayz;
}

void Cylon_Beat(int)
{
  int width = cylonWidth;
  
  if (width < 2) width = 2;
  if (width > 5) width = 5;
  
  int left = (int)cylonPosition - (width>>1);
  for (int i=left; i<left+width; ++i)
  {
     strip.SetLerpColor(i, 0x480000, 15, 250, 750); 
  }
}

#define VU_NODE_MAX 5
int vuNodeCount = 0;
float vuNodeCenter[VU_NODE_MAX];
float vuNodeRadius[VU_NODE_MAX];
float vuNodeHue[VU_NODE_MAX];
bool  vuBeatGroup[VU_NODE_MAX];

void VU_Init()
{
  vuNodeCount = 1 + random(VU_NODE_MAX);
  
  float widthRemaining = stripLength / 2.0;
  int nodesRemaining = vuNodeCount;
  int nodeIndex = 0;
  bool beatGroup = false;
  
  // Two ways of laying out nodes, based on odd or even
  if (vuNodeCount & 1)
  {
    // Odd number of nodes...
    // Lay down the center node first
    vuNodeCenter[0] = (float)stripLength / 2.0;
    vuNodeRadius[0] = 1.0 + ((float)random(500))/100.0;
    vuNodeHue[0] = 75.0 + ((float)random(215));
    vuBeatGroup[0] = beatGroup;
    
    // The symmetric layout below has this much left to work with
    widthRemaining = stripLength/2.0 - vuNodeRadius[0] - 1.0;
    
    --nodesRemaining;
    ++nodeIndex;
    beatGroup = !beatGroup;
  }
  
  widthRemaining -= 1.5;
  while (nodesRemaining >= 2 && widthRemaining >= 1.0)
  {
    float radius = random((int)(100.0 * widthRemaining))/200.0;
    float center = widthRemaining-radius;
    float hue = 75.0 + ((float)random(215));
    
    vuNodeCenter[nodeIndex] = center;
    vuNodeRadius[nodeIndex] = radius;
    vuNodeHue[nodeIndex] = hue;
    vuBeatGroup[nodeIndex] = beatGroup; 
    ++nodeIndex;
    
    vuNodeCenter[nodeIndex] = (float)stripLength - center;
    vuNodeRadius[nodeIndex] = radius;
    vuNodeHue[nodeIndex] = hue;
    vuBeatGroup[nodeIndex] = beatGroup;
    ++nodeIndex;
    
    nodesRemaining -= 2;
    beatGroup = !beatGroup;
    widthRemaining -= radius*2.0 + 0.5;
    
    widthRemaining -= ((float)random(100))/150.0 * widthRemaining;
  }
  
  while (nodesRemaining > 0)
  {
    vuNodeCenter[nodeIndex] = 0.0;
    vuNodeRadius[nodeIndex] = 0.0;
    vuNodeHue[nodeIndex] = 0.0;
    vuBeatGroup[nodeIndex] = false; 
    ++nodeIndex;
    --nodesRemaining;
  }
}

int VU_RenderFrame(int)
{
  for (int i=0; i<stripLength; ++i)
  {
    strip.setPixelColor(i, 0);
  }
  for (int i=0; i<vuNodeCount; ++i)
  {
    if (vuNodeRadius[i] > 0)
    {
      float radius = ((float)audioLevel / (float)audioMax) * vuNodeRadius[i];
      renderSlab(vuNodeCenter[i]-radius, vuNodeCenter[i]+radius, vuNodeHue[i], 1.0, 0.2);
    }
  }
  strip.show();
  
  return 10;
}

bool vuBeatFlipFlop = true;
void VU_Beat(int)
{
  for (int i=0; i<vuNodeCount; ++i)
  {
    if (vuBeatGroup[i] == vuBeatFlipFlop)
    {
       float radius = ((float)audioLevel / (float)audioMax) * vuNodeRadius[i];
       int left  = vuNodeCenter[i]-radius;
       int right = vuNodeCenter[i]+radius;
       
       for (int pixel=left; pixel<=right; ++pixel)
       {
          strip.SetLerpColor(pixel, 0x400000, 25, 150, 125);         
       }
    }
  }
  vuBeatFlipFlop = !vuBeatFlipFlop;
}

int skankinBeatOffsets[5] = {1, 5, 8, 10, 14};
void Skankin_Beat(int)
{
  int offset = skankinBeatOffsets[random(5)];
  strip.SetLerpColor(offset, 0x480000, 15, 250, 15);
  strip.SetLerpColor(offset+1, 0x480000, 15, 250, 15);
  strip.SetLerpColor(offset+2, 0x480000, 15, 250, 15);
}

void DefaultBeat(int val)
{
  for (int iIndex=0; iIndex<stripLength; ++iIndex)
  {
    int centerDistance = abs(stripLength/2 - iIndex);
    strip.SetLerpColor(iIndex, 0x480000, centerDistance*5, 200, 500+centerDistance*20);  
  }
}


typedef struct 
{
  void (*pInitialize)();
  int (*pRenderFrame)(int);
  void (*pBeat)(int);
} TEffect;
TEffect *pEffect = (TEffect*)0;
#define EFFECT_COUNT 5
TEffect effects[EFFECT_COUNT] = 
    {  {HueCycle_Initialize, HueCycle_RenderFrame, DefaultBeat}, 
       {Raindrop_Initialize, Raindrop_RenderFrame, Skankin_Beat},
       {Curtains_Initialize, Curtains_RenderFrame, DefaultBeat},
       {Cylon_Initialize, Cylon_RenderFrame, Cylon_Beat},
       {VU_Init, VU_RenderFrame, VU_Beat}
    };


unsigned long endMillis = 0;
void loop()
{
  
#if 0
  VU_Init();
  for (int i=0; i<vuNodeCount; ++i)
  {
    int left = vuNodeCenter[i]-vuNodeRadius[i];
    int right = vuNodeCenter[i]+vuNodeRadius[i];
    for (int j=0; j<left; ++j) Serial.print(" ");
    for (int j=left; j<=right; ++j) Serial.print(i);
    Serial.println();
  }
  for (int i=0; i<vuNodeCount; ++i)
  {
    Serial.print(i); Serial.print(") Center="); Serial.print(vuNodeCenter[i]); Serial.print(", Radius="); Serial.print(vuNodeRadius[i]);
    Serial.println();
  }

  Serial.println();
#endif

  beatLatch = false;
  beatAck = true;
  
  if (millis() >= endMillis)
  {
    frameIndex = 0;
    effectID = random(EFFECT_COUNT);


    pEffect = &effects[effectID];
    framesRemaining = 300;
    pEffect->pInitialize();
    endMillis = millis() + 15000;
    
    Serial.print("Selecting new effect: "); Serial.println(effectID);
  }
  
  unsigned long delayz;
  delayz = pEffect->pRenderFrame(frameIndex);
  ++frameIndex;
  --framesRemaining;
  
  unsigned long delayEnd = millis() + delayz;
  boolean beating = false;
  while (millis() < delayEnd)
  {
    if (showStats)
    {
      showStats = false;
 //     Serial.print("NEW average: "); Serial.print(average); Serial.print(" / MAX: "); Serial.print(audioMax); 
 //     Serial.print("   Trigger/Release: "); Serial.print(beatTriggerLevel); Serial.print(", "); Serial.println(beatFinishedLevel);    
     }
    
    if (beatLatch)
    {
      pEffect->pBeat(100);
      beatLatch = false;
      beatAck = true;
    }

    strip.show(true);
  }
//  hsv2rgb(300.0, 1.0, 1.0);

}

