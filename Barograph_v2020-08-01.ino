#include <SPI.h> // Needed for the e-ink display and the BME280 sensor.
#include "epd2in9.h" // Needed for the e-ink display.
#include "epdpaint.h" // Needed for the e-ink display.
#include <BME280I2C.h> // Needed for the BME280 sensor.
#include <Wire.h> // Needed for the BME280 sensor.
#include <sSense-CCS811.h> // Needed for the CCS811 sensor.

#define COLORED     0 // Needed for the e-ink display.
#define UNCOLORED   1 // Needed for the e-ink display.

BME280I2C::Settings settings( // Needed for the BME280 sensor.
  BME280::OSR_X2, // Temperature oversampling.
  BME280::OSR_X2, // Humidity oversampling.
  BME280::OSR_X16, // Pressure oversampling.
  BME280::Mode_Normal,
  BME280::StandbyTime_500us,
  BME280::Filter_16,
  BME280::SpiEnable_False,
  0x76 // I2C address.
);
BME280I2C bme(settings); // Needed for the BME280 sensor.

unsigned char image[1024]; // Needed for the e-ink display.
Paint paint(image, 0, 0); // Needed for the e-ink display.
Epd epd; // Needed for the e-ink display.
CCS811 ssenseCCS811; // Needed for the CCS811 sensor.

const uint8_t PinPressureBlueLED = 2; // Pressure blue LED on pin 2.
const uint8_t PinPressureGreenLED = 3; // Pressure green LED on pin 3.
const uint8_t PinPressureRedLED = 4; // Pressure red LED on pin 4.
const uint8_t PinVariationBlueLED = 5; // Variation blue LED on pin 5.
const uint8_t PinVariationGreenLED = 6; // Variation green LED on pin 6.
const uint8_t PinVariationRedLED = 7; // Variation red LED on pin 7.
const uint8_t PinPredictionGreenLED = 8; // Prediction green LED on pin 8.
const uint8_t PinPredictionRedLED = 9; // Prediction red LED on pin 9.
// BME280 and CCS811 "SDA" on pin 20.
// BME280 and CCS811 "SCL" on pin 21.
const uint8_t PinFan = 44; // Fan on pin 44.
// E-ink "RST" on pin 47.
// E-ink "DC" on pin 48.
// E-ink "BUSY" on pin 49.
// E-ink "DIN" on pin 51.
// E-ink "CLK" on pin 52.
// E-ink "CS" on pin 53.
uint8_t MMeanCounter = 0; // Counter used to have a more precise minutely mean pressure.
uint8_t HMeanCounter = 0; // Counter used to have a more precise hourly mean pressure.
uint8_t OutPressureBlueLED = 0; // Pressure blue LED output.
uint8_t OutPressureGreenLED = 0; // Pressure green LED output.
uint8_t OutPressureRedLED = 0; // Pressure red LED output.
uint8_t OutVariationBlueLED = 0; // Variation blue LED output.
uint8_t OutVariationGreenLED = 0; // Variation green LED output.
uint8_t OutVariationRedLED = 0; // Variation red LED output.
uint8_t OutPredictionGreenLED = 0; // Prediction green LED output.
uint8_t OutPredictionRedLED = 0; // Prediction red LED output.
uint8_t OutFan = 0; // Fan output.
bool LastSignVariation = 1; // Used to calculate how much time elapsed since the last sign change of the pressure variation.
bool SignChangeCounterArray[720]; // Used to calculate how many time a change of variation sign happened in the last 12 hours.
int8_t x = 0; // Variable used to do the x axis cycling for the display in the FOR loops.
char CharPressureReading[8]; // Used to display the current pressure.
char CharMAXPressure[8]; // Used to display the maximum pressure of the array.
char CharMINPressure[8]; // Used to display the minimum pressure of the array.
char CharSignVariation[2];  // Used to display the sign of the pressure variation per hour.
char CharVariation[6]; // Used to display the calculated pressure variation per hour.
char CharTSignVariation[6]; // Used to display how much time elapsed since the last sign change of the pressure variation.
char CharSignChangeCounter[4]; // Used to display how many time a change of variation sign happened in the last 12 hours.
char CharHumidityReading[6]; // Used to display the current humidity.
char CharTempReading[6]; // Used to display the current temperature.
char CharTVOCReading[8]; // Used to display the current TVOC.
char CharCO2Reading[8]; // Used to display the current TVOC.
int ConvertedPressure = 0; // Used to store the calculated relation Pa/Pixels on the y axis.
int SignChangeCounter = 0; // Used to calculate how many time a change of variation sign happened in the last 12 hours.
int TVOCReading = 0; // Used for the TVOC reading.
int CO2Reading = 0; // Used for the CO2 reading.
float pres(NAN); // Used by the BME280 sensor for the pressure reading.
float temp(NAN); // Used by the BME280 sensor for the pressure reading.
float hum(NAN); // Used by the BME280 sensor for the humidity reading.
float TempReading = 0; // Used for the temperature reading.
float HumidityReading = 0; // Humidity reading of the BME280 sensor. Units in %RH.
float Variation = 0; // Used to do calculations and comparisons with the variation float value.
float Calc1PredictionGreenLED = 0; // Used to do the addition for the prediction green LED output.
float Calc2PredictionGreenLED = 0; // Used to do the addition for the prediction green LED output.
float Calc1PredictionRedLED = 0; // Used to do the addition for the prediction red LED output.
float Calc2PredictionRedLED = 0; // Used to do the addition for the prediction red LED output.
long BiS[30]; // Array to store the values for the pressure of the last minute of pressure reading (done every 2 seconds).
long M[60]; // Array to store the values for the pressure of the last hour of pressure reading (done every minute).
long H[73]; // Array to store the values for the pressure of the last 72 to 0 hours.
long PressureReading = 0; // Pressure reading of the BME280 sensor. Units in Pa.
long MAXPressure = 0; // Used to calculate the maximum pressure of the array.
long MINPressure = 999999; // Used to calculate the minimum pressure of the array. It's set at 999999 to make sure that a value is lower than it.
long BufferMMeanPressure = 0; // Used to calculate the mean pressure of the last minute.
long MMeanPressure = 0; // Used to calculate the mean pressure of the last minute.
long BufferHMeanPressure = 0; // Used to calculate the mean pressure of the last hour.
long HMeanPressure = 0; // Used to calculate the mean pressure of the last hour.
unsigned long TSignVariation = 0; // Used to display how much time elapsed since the last sign change of the pressure variation.
unsigned long currentMillis; // Used to store the current ms count since the program started.
unsigned long ChronoSignChange; // Used to calculate how much time elapsed since the last sign change of the pressure variation.
unsigned long TimerDataSamplingStartMillis; // Used to calculate a delay of 2 seconds for the TimerDataSampling timer on.
const unsigned long TimerDataSamplingDelay = 2000; // Used to calculate a delay of 2 seconds for the TimerDataSampling timer on.
unsigned long TimerScreenRefreshStartMillis; // Used to calculate a delay of 1 minute for the TimerScreenRefresh timer on.
const unsigned long TimerScreenRefreshDelay = 60000; // Used to calculate a delay of 1 minute for the TimerScreenRefresh timer on.
unsigned long TimerDataMoveStartMillis; // Used to calculate a delay of 1 hour for the TimerDataMove timer on.
const unsigned long TimerDataMoveDelay = 3600000; // Used to calculate a delay of 1 hour for the TimerDataMove timer on.

void setup() {

  TCCR5B = TCCR5B & B11111000 | B00000001; // Change the timer 5 for a PWM frequency of 31372.55 Hz on pin 44.

  pinMode(PinPressureBlueLED, OUTPUT);
  pinMode(PinPressureGreenLED, OUTPUT);
  pinMode(PinPressureRedLED, OUTPUT);
  pinMode(PinVariationBlueLED, OUTPUT);
  pinMode(PinVariationGreenLED, OUTPUT);
  pinMode(PinVariationRedLED, OUTPUT);
  pinMode(PinPredictionGreenLED, OUTPUT);
  pinMode(PinPredictionRedLED, OUTPUT);
  pinMode(PinFan, OUTPUT);

  Serial.begin(9600);

  // E-ink display initialization.
  if (epd.Init(lut_full_update) != 0) {
    Serial.print("e-Paper init failed");
    return;
  }

  // BME280 sensor initialization.
  //while(!Serial) {} // Wait
  Wire.begin();
  while (!bme.begin())
  {
    Serial.println("Could not find BME280 sensor!");
    delay(1000);
  }
  switch (bme.chipModel())
  {
    case BME280::ChipModel_BME280:
      Serial.println("Found BME280 sensor! Success.");
      break;
    case BME280::ChipModel_BMP280:
      Serial.println("Found BMP280 sensor! No Humidity available.");
      break;
    default:
      Serial.println("Found UNKNOWN sensor! Error!");
  }

  // CCS811 sensor initialization.
  if (!ssenseCCS811.begin(uint8_t(I2C_CCS811_ADDRESS), driveMode_1sec)) {
    DebugPort.println("Initialization failed.");
  }

  // Reset the chrono and timers to 0.
  ChronoSignChange = millis(); // Set the ChronoSignChange chrono on to 0.
  TimerDataSamplingStartMillis = TimerDataSamplingDelay; // Set the TimerDataSampling timer on to 0.
  TimerScreenRefreshStartMillis = TimerScreenRefreshDelay; // Set the TimerScreenRefresh timer on to 0.
  TimerDataMoveStartMillis = millis(); // Set the TimerDataMove timer on to 0.
}

void loop() {

  // Store the current ms count since the program started.
  currentMillis = millis();

  // Logic for the TimerDataMove timer on set for 1 hour.
  if (currentMillis - TimerDataMoveStartMillis >= TimerDataMoveDelay) {

    // Move the hourly values except H[0] which is the actual pressure, not a mean pressure.
    for (int8_t y = 72; y >= 2; y--) {
      H[y] = H[y - 1];
    }

    // Put the new hourly pressure data in the hourly array.
    H[1] = HMeanPressure;

    // Reset the TimerDataMove timer on.
    TimerDataMoveStartMillis = currentMillis;
  }

  // Logic for the TimerDataSampling timer on set for 2 seconds.
  if (currentMillis - TimerDataSamplingStartMillis >= TimerDataSamplingDelay) {

    // Read the BME280 sensor.
    BME280::PresUnit presUnit(BME280::PresUnit_Pa);
    BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
    bme.read(pres, temp, hum, tempUnit, presUnit);
    PressureReading = long(pres);
    HumidityReading = (hum + 6);
    TempReading = (temp - 1);

    // Calculate the mean pressure for 1 minute and move every 2 seconds sampling.
    if (MMeanCounter < 30) {
      MMeanCounter = MMeanCounter + 1;
    }
    else {
      MMeanCounter = 30;
    }
    for (int8_t y = 29; y >= 1; y--) { // Samples moving.
      BiS[y] = BiS[y - 1];
    }
    BiS[0] = PressureReading;
    for (int8_t y = 29; y >= 0; y--) { // Minutely mean pressure calculation.
      BufferMMeanPressure = BufferMMeanPressure + BiS[y];
    }
    MMeanPressure = round(BufferMMeanPressure / MMeanCounter);
    BufferMMeanPressure = 0;

    // Reset the TimerDataSampling timer on.
    TimerDataSamplingStartMillis = currentMillis;
  }

  // TimerScreenRefresh timer on set for 1 minute.
  if (currentMillis - TimerScreenRefreshStartMillis >= TimerScreenRefreshDelay) {

    // Send temperature and humudity data to the CCS811 and read the CCS811 sensor.
    ssenseCCS811.setEnvironmentalData(HumidityReading, TempReading);
    if (ssenseCCS811.checkDataAndUpdate()) {
      TVOCReading = ssenseCCS811.gettVOC();
      CO2Reading = ssenseCCS811.getCO2();
    }
    else if (ssenseCCS811.checkForError())
    {
      TVOCReading = -1;
      CO2Reading = -1;
    }

    // Calculate the mean pressure for one hour and move every 1 minute sampling.
    if (HMeanCounter < 60) {
      HMeanCounter = HMeanCounter + 1;
    }
    else {
      HMeanCounter = 60;
    }
    for (int8_t y = 59; y >= 1; y--) { // Samples moving.
      M[y] = M[y - 1];
    }
    M[0] = MMeanPressure;
    for (int8_t y = 59; y >= 0; y--) { // Hourly mean pressure calculation.
      BufferHMeanPressure = BufferHMeanPressure + M[y];
    }
    HMeanPressure = round(BufferHMeanPressure / HMeanCounter);
    BufferHMeanPressure = 0;
    H[0] = MMeanPressure;

    // Clearing the screen.
    epd.ClearFrameMemory(0xFF);   // bit set = white, bit reset = black
    epd.DisplayFrame();

    // Set frame size and orientation.
    paint.SetWidth(128);
    paint.SetHeight(64);
    paint.SetRotate(ROTATE_90);

    // Draw frame #1 in memory.
    paint.Clear(UNCOLORED);
    paint.DrawStringAt(1, 1, "104", &Font8, COLORED);
    paint.DrawHorizontalLine(17, 3, 2, COLORED);
    paint.DrawStringAt(1, 20, "103", &Font8, COLORED);
    paint.DrawHorizontalLine(17, 22, 2, COLORED);
    paint.DrawStringAt(1, 39, "102", &Font8, COLORED);
    paint.DrawHorizontalLine(17, 41, 2, COLORED);
    paint.DrawStringAt(1, 58, "101", &Font8, COLORED);
    paint.DrawHorizontalLine(17, 60, 2, COLORED);
    paint.DrawStringAt(1, 77, "100", &Font8, COLORED);
    paint.DrawHorizontalLine(17, 79, 2, COLORED);
    paint.DrawStringAt(7, 96, "99", &Font8, COLORED);
    paint.DrawHorizontalLine(17, 98, 2, COLORED);
    paint.DrawStringAt(7, 115, "98", &Font8, COLORED);
    paint.DrawHorizontalLine(17, 117, 2, COLORED);
    paint.DrawStringAt(11, 122, "-72", &Font8, COLORED);
    paint.DrawVerticalLine(21, 119, 2, COLORED);
    paint.DrawStringAt(35, 122, "-60", &Font8, COLORED);
    paint.DrawVerticalLine(45, 119, 2, COLORED);
    paint.DrawStringAt(59, 122, "-", &Font8, COLORED);
    // For drawing hours 72 to 51.
    x = 20;
    for (int8_t y = 72; y >= 51; y--) {
      ConvertedPressure = round((116 - 0.019 * (H[y] - 98000))); // Pa to pixel convertion + rounding.
      paint.DrawRectangle(x, ConvertedPressure, (x + 1), (ConvertedPressure + 1), COLORED);
      x = x + 2;
      if (MAXPressure < H[y]) { // Store the highest pressure encountered.
        MAXPressure = H[y];
      }
      if ((MINPressure > H[y]) && (H[y] > 0)) { // Store the lowest pressure encountered.
        MINPressure = H[y];
      }
    }
    epd.SetFrameMemory(paint.GetImage(), 0, 0, paint.GetWidth(), paint.GetHeight()); // Location of frame #1.

    // Draw frame #2 in memory.
    paint.Clear(UNCOLORED);
    paint.DrawStringAt(0, 122, "48", &Font8, COLORED);
    paint.DrawVerticalLine(5, 119, 2, COLORED);
    paint.DrawStringAt(19, 122, "-36", &Font8, COLORED);
    paint.DrawVerticalLine(29, 119, 2, COLORED);
    paint.DrawStringAt(43, 122, "-24", &Font8, COLORED);
    paint.DrawVerticalLine(53, 119, 2, COLORED);
    // For drawing hours 50 to 19.
    x = 0;
    for (int8_t y = 50; y >= 19; y--) {
      ConvertedPressure = round((116 - 0.019 * (H[y] - 98000))); // Pa to pixel convertion + rounding.
      paint.DrawRectangle(x, ConvertedPressure, (x + 1), (ConvertedPressure + 1), COLORED);
      x = x + 2;
      if (MAXPressure < H[y]) { // Store the highest pressure encountered.
        MAXPressure = H[y];
      }
      if ((MINPressure > H[y]) && (H[y] > 0)) { // Store the lowest pressure encountered.
        MINPressure = H[y];
      }
    }
    epd.SetFrameMemory(paint.GetImage(), 0, 64, paint.GetWidth(), paint.GetHeight()); // Location of frame #2.

    // New frame size for frame #3.
    paint.SetWidth(128);
    paint.SetHeight(40);
    paint.SetRotate(ROTATE_90);

    // Draw frame #3 in memory (partial : 40 x 128).
    paint.Clear(UNCOLORED);
    paint.DrawStringAt(3, 122, "-12", &Font8, COLORED);
    paint.DrawVerticalLine(13, 119, 2, COLORED);
    paint.DrawStringAt(35, 122, "0", &Font8, COLORED);
    paint.DrawVerticalLine(37, 119, 2, COLORED);
    // For drawing hours 18 to 0.
    x = 0;
    for (int8_t y = 18; y >= 0; y--) {
      ConvertedPressure = round((116 - 0.019 * (H[y] - 98000))); // Pa to pixel convertion + rounding.
      paint.DrawRectangle(x, ConvertedPressure, (x + 1), (ConvertedPressure + 1), COLORED);
      x = x + 2;
      if (MAXPressure < H[y]) { // Store the highest pressure encountered.
        MAXPressure = H[y];
      }
      if ((MINPressure > H[y]) && (H[y] > 0)) { // Store the lowest pressure encountered.
        MINPressure = H[y];
      }
    }
    epd.SetFrameMemory(paint.GetImage(), 0, 128, paint.GetWidth(), paint.GetHeight()); // Location of frame #3.

    // Display formatting for the actual pressure.
    ltoa(MMeanPressure, CharPressureReading, 10); // Long to ASCII convertion to a character array.
    if (MMeanPressure >= 100000) {
      for (int8_t y = 7; y >= 4; y--) {
        CharPressureReading[y] = CharPressureReading[y - 1];
      }
      CharPressureReading[3] = '.';
    }
    else {
      for (int8_t y = 7; y >= 3; y--) {
        CharPressureReading[y] = CharPressureReading[y - 1];
      }
      CharPressureReading[2] = '.';
    }

    // Display formatting for the maximum pressure.
    ltoa(MAXPressure, CharMAXPressure, 10); // Long to ASCII convertion to a character array.
    if (MAXPressure >= 100000) {
      for (int8_t y = 7; y >= 4; y--) {
        CharMAXPressure[y] = CharMAXPressure[y - 1];
      }
      CharMAXPressure[3] = '.';
    }
    else {
      for (int8_t y = 7; y >= 3; y--) {
        CharMAXPressure[y] = CharMAXPressure[y - 1];
      }
      CharMAXPressure[2] = '.';
    }

    // Display formatting for the minimum pressure.
    ltoa(MINPressure, CharMINPressure, 10); // Long to ASCII convertion to a character array.
    if (MINPressure >= 100000) {
      for (int8_t y = 7; y >= 4; y--) {
        CharMINPressure[y] = CharMINPressure[y - 1];
      }
      CharMINPressure[3] = '.';
    }
    else {
      for (int8_t y = 7; y >= 3; y--) {
        CharMINPressure[y] = CharMINPressure[y - 1];
      }
      CharMINPressure[2] = '.';
    }

    // Reset the MAXPressure and MINPressure variables for the next comparison cycle.
    MAXPressure = 0;
    MINPressure = 999999;

    // Calculate and display formatting for the variation sign change counter. Also calculate and display formatting for the mean pressure variation per hour for an interval of 30 minutes
    // and for the hours elapsed since the last sign change of mean pressure variation.
    for (int y = 719; y >= 1; y--) { // Samples moving.
      SignChangeCounterArray[y] = SignChangeCounterArray[y - 1];
    }
    SignChangeCounterArray[0] = 0;
    Variation = (M[0] - M[59]);
    if ((Variation > 0) && (LastSignVariation == 0)) {
      ChronoSignChange = currentMillis;
      SignChangeCounterArray[0] = 1;
    }
    if ((Variation < 0) && (LastSignVariation == 1)) {
      ChronoSignChange = currentMillis;
      SignChangeCounterArray[0] = 1;
    }
    TSignVariation = ((currentMillis - ChronoSignChange) / 1000);
    CharTSignVariation[0] = TSignVariation / 36000 + '0';
    CharTSignVariation[1] = TSignVariation % 36000 / 3600 + '0';
    CharTSignVariation[2] = ':';
    CharTSignVariation[3] = TSignVariation % 3600 / 600 + '0';
    CharTSignVariation[4] = TSignVariation % 3600 % 600 / 60 + '0';
    for (int y = 719; y >= 0; y--) { // Summing.
      SignChangeCounter = SignChangeCounter + SignChangeCounterArray[y];
    }
    itoa(SignChangeCounter, CharSignChangeCounter, 10); // Integer to ASCII convertion to a character array.
    SignChangeCounter = 0;
    if (Variation > 0) {
      CharSignVariation[0] = '+';
      LastSignVariation = 1;
    }
    if (Variation < 0) {
      CharSignVariation[0] = '-';
      LastSignVariation = 0;
    }
    if (Variation == 0) {
      CharSignVariation[0] = ' ';
    }
    itoa((round(abs(Variation))), CharVariation, 10); // Integer to ASCII convertion to a character array.
    if ((abs(Variation)) >= 1000) {
      for (int8_t y = 5; y >= 2; y--) {
        CharVariation[y] = CharVariation[y - 1];
      }
      CharVariation[1] = '.';
    }
    if (((abs(Variation)) < 1000) && ((abs(Variation)) >= 100)) {
      for (int8_t y = 5; y >= 2; y--) {
        CharVariation[y] = CharVariation[y - 2];
      }
      CharVariation[0] = '0';
      CharVariation[1] = '.';
    }
    if (((abs(Variation)) < 100) && ((abs(Variation)) >= 10)) {
      for (int8_t y = 5; y >= 3; y--) {
        CharVariation[y] = CharVariation[y - 3];
      }
      CharVariation[0] = '0';
      CharVariation[1] = '.';
      CharVariation[2] = '0';
    }
    if (((abs(Variation)) < 10) && ((abs(Variation)) > 0)) {
      for (int8_t y = 5; y >= 4; y--) {
        CharVariation[y] = CharVariation[y - 4];
      }
      CharVariation[0] = '0';
      CharVariation[1] = '.';
      CharVariation[2] = '0';
      CharVariation[3] = '0';
    }

    // For the humidity display.
    dtostrf(HumidityReading, 5, 2, CharHumidityReading); // Float to ASCII convertion to a character array.

    // For the temperature display.
    dtostrf(TempReading, 5, 2, CharTempReading); // Float to ASCII convertion to a character array.

    // For the TVOC display.
    itoa(TVOCReading, CharTVOCReading, 10); // Integer to ASCII convertion to a character array.

    // For the CO2 display.
    itoa(CO2Reading, CharCO2Reading, 10); // Integer to ASCII convertion to a character array.

    // New size for frame #4 and #5.
    paint.SetWidth(64);
    paint.SetHeight(128);
    paint.SetRotate(ROTATE_90);

    // Draw frame #4 in memory.
    paint.Clear(UNCOLORED);
    paint.DrawStringAt(0, 4, "P(kPa)   :", &Font12, COLORED);
    paint.DrawStringAt(70, 4, CharPressureReading, &Font12, COLORED);
    paint.DrawStringAt(0, 14, "PMAX(kPa):", &Font12, COLORED);
    paint.DrawStringAt(70, 14, CharMAXPressure, &Font12, COLORED);
    paint.DrawStringAt(0, 24, "PMIN(kPa):", &Font12, COLORED);
    paint.DrawStringAt(70, 24, CharMINPressure, &Font12, COLORED);
    paint.DrawStringAt(0, 34, "V(kPa/h) :", &Font12, COLORED);
    paint.DrawStringAt(70, 34, CharSignVariation, &Font12, COLORED);
    paint.DrawStringAt(77, 34, CharVariation, &Font12, COLORED);
    paint.DrawStringAt(0, 44, "T+-V(h:m):", &Font12, COLORED);
    paint.DrawStringAt(70, 44, CharTSignVariation, &Font12, COLORED);
    paint.DrawStringAt(0, 54, "#+-V -12h:", &Font12, COLORED);
    paint.DrawStringAt(70, 54, CharSignChangeCounter, &Font12, COLORED);
    epd.SetFrameMemory(paint.GetImage(), 64, 168, paint.GetWidth(), paint.GetHeight()); // Location of frame #4.

    // Draw frame #5 in memory.
    paint.Clear(UNCOLORED);
    paint.DrawStringAt(0, 0, "H(%)     :", &Font12, COLORED);
    paint.DrawStringAt(70, 0, CharHumidityReading, &Font12, COLORED);
    paint.DrawStringAt(0, 10, "Temp(C)  :", &Font12, COLORED);
    paint.DrawStringAt(70, 10, CharTempReading, &Font12, COLORED);
    paint.DrawStringAt(0, 20, "COVT(ppb):", &Font12, COLORED);
    paint.DrawStringAt(70, 20, CharTVOCReading, &Font12, COLORED);
    paint.DrawStringAt(0, 30, "CO2(ppm) :", &Font12, COLORED);
    paint.DrawStringAt(70, 30, CharCO2Reading, &Font12, COLORED);
    epd.SetFrameMemory(paint.GetImage(), 0, 168, paint.GetWidth(), paint.GetHeight()); // Location of frame #5.

    // Draw on the display everything that was put in the display memory.
    epd.DisplayFrame();

    // Pressure LEDs calculations.
    if (MMeanPressure < 98000) {
      OutPressureBlueLED = 255;
    }
    if ((MMeanPressure >= 98000) && (MMeanPressure <= 101000)) {
      OutPressureBlueLED = round(255 - 0.085 * (MMeanPressure - 98000));
    }
    if (MMeanPressure > 101000) {
      OutPressureBlueLED = 0;
    }
    if ((MMeanPressure < 98000) || (MMeanPressure > 104000)) {
      OutPressureGreenLED = 0;
    }
    if ((MMeanPressure >= 98000) && (MMeanPressure <= 101000)) {
      OutPressureGreenLED = round(0.085 * (MMeanPressure - 98000));
    }
    if ((MMeanPressure > 101000) && (MMeanPressure <= 104000)) {
      OutPressureGreenLED = round(255 - (0.085 * (MMeanPressure - 101000)));
    }
    if (MMeanPressure < 101000) {
      OutPressureRedLED = 0;
    }
    if ((MMeanPressure >= 101000) && (MMeanPressure <= 104000)) {
      OutPressureRedLED = round(0.085 * (MMeanPressure - 101000));
    }
    if (MMeanPressure > 104000) {
      OutPressureRedLED = 255;
    }

    // Variation LEDs calculations.
    if (Variation < -100) {
      OutVariationBlueLED = 255;
    }
    if ((Variation >= -100) && (Variation < 0)) {
      OutVariationBlueLED = round(abs(Variation * 2.55));
    }
    if (Variation >= 0) {
      OutVariationBlueLED = 0;
    }
    if ((abs(Variation)) > 100) {
      OutVariationGreenLED = 0;
    }
    if ((abs(Variation)) <= 100) {
      OutVariationGreenLED = round(255 - abs(Variation * 2.55));
    }
    if (Variation > 100) {
      OutVariationRedLED = 255;
    }
    if ((Variation <= 100) && (Variation > 0)) {
      OutVariationRedLED = round(Variation * 2.55);
    }
    if (Variation <= 0) {
      OutVariationRedLED = 0;
    }

    // Prediction LEDs calculations.
    if (((MMeanPressure >= 102000) && (Variation >= 50)) || ((MMeanPressure <= 100000) && (Variation <= -50)) || (((TSignVariation / 3600) >= 36) && ((abs(Variation)) >= 100)) || ((abs(Variation)) >= 150) || (MMeanPressure >= 104000) || (MMeanPressure <= 98000)) {
      OutPredictionGreenLED = 0;
      OutPredictionRedLED = 255;
    }
    // Prediction 1 : If P >= 102000 AND V >= 50 (maximum values) / P >= 101666 AND V >= 33 (minimum values). Minimum = 2/3 maximum.
    else if ((MMeanPressure >= 101666) && (Variation >= 33)) {
      if ((MMeanPressure >= 101666) && (MMeanPressure < 102000)) {
        Calc1PredictionGreenLED = (127.5 - (0.381736 * (MMeanPressure - 101666)));
        Calc1PredictionRedLED = (0.381736 * (MMeanPressure - 101666));
      }
      if (MMeanPressure >= 102000) {
        Calc1PredictionGreenLED = 0;
        Calc1PredictionRedLED = 127.5;
      }
      if ((Variation >= 33) && (Variation < 50)) {
        Calc2PredictionGreenLED = (127.5 - (7.5 * (Variation - 33)));
        Calc2PredictionRedLED = (7.5 * (Variation - 33));
      }
      if (Variation >= 50) {
        Calc2PredictionGreenLED = 0;
        Calc2PredictionRedLED = 127.5;
      }
      OutPredictionGreenLED = round(Calc1PredictionGreenLED + Calc2PredictionGreenLED);
      OutPredictionRedLED = round(Calc1PredictionRedLED + Calc2PredictionRedLED);
    }
    // Prediction 2 : If P <= 100000 AND V <= -50 (maximum values) / P <= 101666 AND V <= -33 (minimum values). Minimum = 2/3 maximum.
    else if ((MMeanPressure <= 100333) && (Variation <= -33)) {
      if ((MMeanPressure <= 100333) && (MMeanPressure > 100000)) {
        Calc1PredictionGreenLED = (0.382882 * (MMeanPressure - 100000));
        Calc1PredictionRedLED = (127.5 - (0.382882 * (MMeanPressure - 100000)));
      }
      if (MMeanPressure <= 100000) {
        Calc1PredictionGreenLED = 0;
        Calc1PredictionRedLED = 127.5;
      }
      if ((Variation <= -33) && (Variation > -50)) {
        Calc2PredictionGreenLED = (127.5 - (7.5 * ((abs(Variation)) + 33)));
        Calc2PredictionRedLED = (7.5 * ((abs(Variation)) + 33));
      }
      if (Variation <= -50) {
        Calc2PredictionGreenLED = 0;
        Calc2PredictionRedLED = 127.5;
      }
      OutPredictionGreenLED = round(Calc1PredictionGreenLED + Calc2PredictionGreenLED);
      OutPredictionRedLED = round(Calc1PredictionRedLED + Calc2PredictionRedLED);
    }
    // Prediction 3 : If T+/-V >= 36 AND abs(V) >= 100 (maximum values) / T+/-V >= 24 AND abs(V) >= 66 (minimum values). Minimum = 2/3 maximum.
    else if (((TSignVariation / 3600) >= 24) && ((abs(Variation)) >= 66)) {
      if (((TSignVariation / 3600) >= 24) && ((TSignVariation / 3600) < 36)) {
        Calc1PredictionGreenLED = (127.5 - (10.625 * ((TSignVariation / 3600) - 24)));
        Calc1PredictionRedLED = (10.625 * ((TSignVariation / 3600) - 24));
      }
      if ((TSignVariation / 3600) >= 36) {
        Calc1PredictionGreenLED = 0;
        Calc1PredictionRedLED = 127.5;
      }
      if (((abs(Variation)) >= 66) && ((abs(Variation)) < 100)) {
        Calc2PredictionGreenLED = (127.5 - (3.75 * ((abs(Variation)) - 66)));
        Calc2PredictionRedLED = (3.75 * ((abs(Variation)) - 66));
      }
      if ((abs(Variation)) >= 100) {
        Calc2PredictionGreenLED = 0;
        Calc2PredictionRedLED = 127.5;
      }
      OutPredictionGreenLED = round(Calc1PredictionGreenLED + Calc2PredictionGreenLED);
      OutPredictionRedLED = round(Calc1PredictionRedLED + Calc2PredictionRedLED);
    }
    // Prediction 4 : If abs(V) >= 150 (maximum value) / abs(V) >= 100 (minimum value). Minimum = 2/3 maximum.
    else if ((abs(Variation)) >= 100) {
      if (((abs(Variation)) >= 100) && ((abs(Variation)) < 150)) {
        OutPredictionGreenLED = round(255 - (5.1 * ((abs(Variation)) - 100)));
        OutPredictionRedLED = round(5.1 * ((abs(Variation)) - 100));
      }
      if ((abs(Variation)) >= 150) {
        OutPredictionGreenLED = 0;
        OutPredictionRedLED = 255;
      }
    }
    // Prediction 5 : If P >= 104000 (maximum value) / P >= 103000 (minimum value). Minimum = 2/3 maximum.
    else if (MMeanPressure >= 103000) {
      if ((MMeanPressure >= 103000) && (MMeanPressure < 104000)) {
        OutPredictionGreenLED = round(255 - (0.255 * (MMeanPressure - 103000)));
        OutPredictionRedLED = round(0.255 * (MMeanPressure - 103000));
      }
      if (MMeanPressure >= 104000) {
        OutPredictionGreenLED = 0;
        OutPredictionRedLED = 255;
      }
    }
    // Prediction 6 : If P <= 98000 (maximum value) / P <= 99000 (minimum value). Minimum = 2/3 maximum.
    else if (MMeanPressure <= 99000) {
      if ((MMeanPressure <= 99000) && (MMeanPressure > 98000)) {
        OutPredictionGreenLED = round(0.255 * (MMeanPressure - 98000));
        OutPredictionRedLED = round(255 - (0.255 * (MMeanPressure - 98000)));
      }
      if (MMeanPressure <= 98000) {
        OutPredictionGreenLED = 0;
        OutPredictionRedLED = 255;
      }
    }
    else {
      OutPredictionGreenLED = 255;
      OutPredictionRedLED = 0;
    }

    // Write the calculated values to the output.
    analogWrite(PinPressureBlueLED, OutPressureBlueLED);
    analogWrite(PinPressureGreenLED, OutPressureGreenLED);
    analogWrite(PinPressureRedLED, OutPressureRedLED);
    analogWrite(PinVariationBlueLED, OutVariationBlueLED);
    analogWrite(PinVariationGreenLED, OutVariationGreenLED);
    analogWrite(PinVariationRedLED, OutVariationRedLED);
    analogWrite(PinPredictionGreenLED, OutPredictionGreenLED);
    analogWrite(PinPredictionRedLED, OutPredictionRedLED);

    // Reset the TimerScreenRefresh timer on.
    TimerScreenRefreshStartMillis = currentMillis;
  }

  // Fan speed calculation.
  if (TempReading < 21) {
    OutFan = 0;
  }
  if (TempReading >= 21 && TempReading < 31) {
    OutFan = (155 + ((TempReading - 21) * 2.5));
  }
  if (TempReading >= 31) {
    OutFan = 180;
  }
  analogWrite(PinFan, OutFan);
}
