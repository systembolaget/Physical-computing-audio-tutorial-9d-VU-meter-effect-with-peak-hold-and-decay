// Tutorial 9d. VU meter effect with peak hold and decay

// Main parts: See tutorials 9a or 9b; here we use set-up 9a, in the
// stereo configuration, and two potentiometers, see schematic

// Library required to drive RGB LEDs; use the latest version
#include "FastLED.h"

// Variables that remain constant
const byte pinAudioInL = 0; const byte pinAudioInR = 1; // Analog input pin from L/R channel audio
const byte pinPotentiometer1 = 2; // Analog input pin from linear potentiometer
const byte pinPotentiometer2 = 3; // Analog input pin from logarithmic potentiometer (brightness perception is logarithmic)

const byte timeIntervalSampling = 30; // Sampling window width time in milliseconds
const int levelShift = 512; // Level shifting and re-fitting
const int intervalPeakHold = 500; // Peak hold interval in milliseconds
const byte intervalPeakDecay = 20; // Peak decay interval in milliseconds

const byte pinDataL = 4; const byte pinDataR = 5; // Digital output pin to the LED ring
const byte numLeds = 24; // Number of LEDs per ring

// Variables that can change
int audioInRawL; int audioInRawR; // Raw audio input
int audioInRectifiedL; int audioInRectifiedR; // Rectified audio input
float audioFilteredL; float audioFilteredR; // Filtered audio input

byte ledBrightness = 128; // Adjust from 0 - 229 with potentiometer; see why in checkPotentiometers() at the end of code

int newPeakL; int newPeakR; // New peak value (= LED-number from 0 to 23)
int previousPeakL; int previousPeakR; // Previous peak value
unsigned long timeOfPeakL; unsigned long timeOfPeakR; // Timestamp that updates with each new peak
bool peakDecayL = false; bool peakDecayR = false; // Toggles if the peak LED is allowed to decay or not
int peakLevelLow; // Set depending on the dynamics of audio input; see why in checkPotentiometers() at the end of code
int peakLevelHigh;

CRGB ledRingL[numLeds]; CRGB ledRingR[numLeds]; // Declare an array to store the LED ring's data
CRGB ledGradient[numLeds]; // Declare an array to store the LED colour gradient
CHSV gradientHueStart = CHSV(96, 255, 255); // Colour gradient start colour (= green); any FastLED gradient setting method can be used alternatively
CHSV grandientHueEnd = CHSV(0, 255, 255); // Colour gradient end colour (= red)

void setup()
{
  // Serial printing only necessary when observing values in the serial plotter
  // Serial.begin(115200);

  // Use 2,5V AREF as reference voltage, bias = 1,65V (microphone amplifier board)
  // analogReference(EXTERNAL);
  // Use 1,1V AREF as reference voltage, bias = 0,55V (headphone jack)
  analogReference(INTERNAL);

  ADCSRA &= ~(bit (ADPS0) | bit (ADPS1) | bit (ADPS2)); // Clear ADC prescaler bits
  ADCSRA |= bit (ADPS0) | bit (ADPS2); // Set ADC prescaler to 32

  // Initialise the FastLED library with the type of programmable RGB LEDs
  // used, the digital output pin the LED ring is wired to, the array that
  // stores each ring's LEDs data, and the number of LEDs per ring
  FastLED.addLeds<NEOPIXEL, pinDataL>(ledRingL, numLeds); FastLED.addLeds<NEOPIXEL, pinDataR>(ledRingR, numLeds);

  // Fill LED colour gradient array with colour values (= from green to red)
  // Any FastLED gradient setting method can be used alternatively
  fill_gradient(ledGradient, 0, gradientHueStart, numLeds, grandientHueEnd, SHORTEST_HUES);
}

void loop()
{
  unsigned long timeSamplingStart = millis(); // Timestamp

  uint16_t audioMinL = 1023; uint16_t audioMinR = 1023;
  uint16_t audioMaxL = 0; uint16_t audioMaxR = 0;
  uint16_t audioPeakToPeakL = 0; uint16_t audioPeakToPeakR = 0;

  while (millis() - timeSamplingStart < timeIntervalSampling)
  {
    audioInRawL = analogRead(pinAudioInL) + 0; audioInRawR = analogRead(pinAudioInR) + 0; // ADC correction factor +- # at 5V, +- # at 3.3V, +- # at 2.5V, +- # at 1.1V
    audioInRectifiedL = abs(audioInRawL - levelShift); audioInRectifiedR = abs(audioInRawR - levelShift); // Rectify and re-fit to 0

    audioMinL = min(audioInRectifiedL, audioMinL); audioMinR = min(audioInRectifiedR, audioMinR); // Find the minimum input value
    audioMaxL = max(audioInRectifiedL, audioMaxL); audioMaxR = max(audioInRectifiedR, audioMaxR); // Find the maximum input value
  }

  audioPeakToPeakL = audioMaxL - audioMinL; audioPeakToPeakR = audioMaxR - audioMinR; // Calculate peak-to-peak value
  //Serial.print("audioPeakToPeakL:"); Serial.println(audioPeakToPeakL);

  audioFilteredL += (audioPeakToPeakL - audioFilteredL) * 0.25; audioFilteredR += (audioPeakToPeakR - audioFilteredR) * 0.25; // Leaky integrator, fast attack and slow decay (k < 1)
  //Serial.print("audioFilteredL:"); Serial.println(audioFilteredL);

  newPeakL = constrain(map(audioFilteredL, peakLevelLow, peakLevelHigh, 0, numLeds), 0, numLeds) - 1; newPeakR = constrain(map(audioFilteredR, peakLevelLow, peakLevelHigh, 0, numLeds), 0, numLeds) - 1; // Set new peak value
  //Serial.print("newPeakL:"); Serial.println(newPeakL);

  peakHoldAndDecayL(); peakHoldAndDecayR(); // Update peak hold and decay

  lightLEDs(); // Light the LEDs

  checkPotentiometers(); // Check if input dynamics or brightness were adjusted
}

void peakHoldAndDecayL() // Peak hold and decay algorithm
{
  // Only if the new peak value is larger than the previous peak value,
  // it is set to the new peak value. Then we take the time of when that
  // happened. The peak decay indicator is set to false, so that the
  // peak LED won't start to decay until the peak hold time interval
  // has expired, tested in the next if statement
  if (newPeakL >= previousPeakL)
  {
    previousPeakL = newPeakL;
    timeOfPeakL = millis();
    peakDecayL = false;
  }

  // If the peak decay indicator was set to false, and the peak hold
  // time interval is over, we set it to true, to allow the peak LED to
  // decay. We add the peak hold time interval to the time of when the
  // new peak occurred and subtract the first peak decay time interval.
  // Now that the peak decay indicator is set to true, this if statement
  // will not be revisited while the LED decays
  else if (!peakDecayL && (millis() - timeOfPeakL > intervalPeakHold))
  {
    peakDecayL = true;
    timeOfPeakL += intervalPeakHold - intervalPeakDecay;
  }

  // While the peak decay indicator is set to true, and if another
  // peak decay time interval expired, we continue decaying the peak
  // LED while it is above 0
  else if (peakDecayL && (millis() - timeOfPeakL > intervalPeakDecay))
  {
    if (previousPeakL >= 0)
    {
      previousPeakL--;
      timeOfPeakL += intervalPeakDecay;
    }
  }
}

void peakHoldAndDecayR()
{
  if (newPeakR >= previousPeakR)
  {
    previousPeakR = newPeakR;
    timeOfPeakR = millis();
    peakDecayR = false;
  }

  else if (!peakDecayR && (millis() - timeOfPeakR > intervalPeakHold))
  {
    peakDecayR = true;
    timeOfPeakR += intervalPeakHold - intervalPeakDecay;
  }

  else if (peakDecayR && (millis() - timeOfPeakR > intervalPeakDecay))
  {
    if (previousPeakR >= 0)
    {
      previousPeakR--;
      timeOfPeakR += intervalPeakDecay;
    }
  }
}

void lightLEDs()
{
  // Clear the LED arrays that hold all LED's colour and brightnesss data
  FastLED.clear();

  FastLED.setBrightness(ledBrightness);

  // Depending on the newe peak value (= number of LEDs to light)
  for ( byte i = 0; i <= newPeakL; i++)
  {
    // Copy the corresponding colour from the gradient array to the LED array L
    ledRingL[i] = ledGradient[i];
  }

  // And do the same thing for the LED array R
  for ( byte j = 0; j <= newPeakR; j++)
  {
    ledRingR[j] = ledGradient[j];
  }

  // Then set the peak LED to red
  ledRingL[previousPeakL] = CHSV(0, 255, 255); ledRingR[previousPeakR] = CHSV(0, 255, 255);

  // Finally, display all LED's data (= illuminate the LED ring)
  FastLED.show();
}

void checkPotentiometers()
{
  // The potentiometers are hardware smoothed with a 0.1uF capacitor; no
  // code is necessary. Read the voltage from the potentiometer pins
  peakLevelLow = map(analogRead(pinPotentiometer1), 0, 915, 0, 75); // 915 max. instead of 1023 because of the 39k/10k voltage divider (1,02V are not quite 1,1V AREF, a 35.4k resistor does not exist)
  peakLevelHigh = map(analogRead(pinPotentiometer1), 0, 915, 225, 475);

  // Bit-shift division by four yields 0 - 229 (not 0 - 255 because the range is 0 - 915 instead of 0 - 1023)
  ledBrightness = analogRead(pinPotentiometer2) >> 2;
}
