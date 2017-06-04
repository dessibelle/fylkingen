#include <SFE_BMP180.h>
#include <Wire.h>

// #define DEBUG // This enables debug logging, uncomment to enable debug loggin

#ifdef DEBUG
  #define SAMPLING_DELAY 500 // Must be greater than 60ms

  #define DEBUG_PRINT(x)       Serial.print (x)
  #define DEBUG_PRINT_DEC(x)   Serial.print (x, DEC)
  #define DEBUG_PRINT_LN(x)    Serial.println (x)
#else
  #define SAMPLING_DELAY 80 // Must be greater than 60ms

  #define DEBUG_PRINT(x)
  #define DEBUG_PRINT_DEC(x)
  #define DEBUG_PRINT_LN(x)
#endif

#define SERIAL_PRINT_COMPOUND_VALUE 1

/***********************************
 * Internal communication protocol *
 ***********************************/

struct dataRecord
{
  float temperature;
  float pressure;
  float sealevelPressure;
  float altitude;
  int distance;
  int heartrate;
};

typedef struct dataRecord Record;

struct mappedRecord
{
  int temperature;
  int pressure;
  int sealevelPressure;
  int altitude;
  int distance;
  int heartrate;
};

typedef struct mappedRecord MappedRecord;

/****************
 * Global state *
 ****************/

double sessionPressureBaseline;
double sessionTemperatureBaseline;
int currentHeartrate = 60; // Default heart rate to 60, until first measurement completes

/**************************************
 * BMP180 Barometic sensor setup      *
 *                                    *
 * CL connects to Arduino Analog IN 5 *
 * DA connects to Arduino Analog IN 4 *
 *                                    *
 **************************************/

SFE_BMP180 barometer; // BPM180 pressure struct
#define BAROMETER_ALTITUDE 52.1 // Altitude of Stockholm, Sweden in meters
#define BAROMETER_BASELINE_PRESSURE 1013.25 // "Standard" pressure, the baseline used universally, is 1013.25 hPa
#define BAROMETER_OVERSAMPLING_MODE 3 // 0 to 3, higher numbers are slower, higher-res outputs

/***********************************
 * HC-SR04 Ultrasound sensor setup *
 ***********************************/

/*
 * HC-SR04 Ultrasound / Ping distance sensor
 * https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf
 *
 * Ultrasonic ranging module HC - SR04 provides 2cm - 400cm non-contact measurement function,
 * the ranging accuracy can reach to 3mm. The modules includes ultrasonic transmitters,
 * receiver and control circuit. The basic principle of work:
 *
 * 1. Using IO trigger for at least 10µs high level signal,
 * 2. The Module automatically sends eight 40 kHz and detect whether there is a pulse signal back.
 * 3. IF the signal back, through high level, time of high output IO duration is the time from
 *    sending ultrasonic to returning. Test distance = (high level time×velocity of sound (340M/S) / 2,
 */
#define ULTRASOUND_TRIG_PIN 13
#define ULRTASOUND_ECHO_PIN 12
#define ULTRASOUND_UPPER_BOUNDS 2000 // Theoretically 4 metres, but 2 works better
#define ULTRASOUND_LOWER_BOUNDS 20
#define ULTRASOUND_PULSE_TIMEOUT 11500 // Almost 2 m at 340 m/s

/**************************************
 * Grove - Ear-clip Heart Rate sensor *
 **************************************/

#define HEARTRATE_INTERRUPT_PIN 2          // Pin used for heartreate sensor
#define HEARTRATE_MAX_PULSE_INTERVAL 2000  // No heartbeat for this many ms - reset counter
#define HEARTRATE_NUM_SAMPLES 20           // Number of samples needed for calculation

unsigned char heartrateCounter;
unsigned long heartrateSamples[HEARTRATE_NUM_SAMPLES];

/*************************************
 * BMP180 Barometic sensor functions *
 *************************************/

void setupBarometricSensor()
{
  if (barometer.begin())
  {
    DEBUG_PRINT(F("BMP180 init success, altitude: "));
    DEBUG_PRINT(BAROMETER_ALTITUDE);
    DEBUG_PRINT(F(" m, baseline (sealevel) pressure: "));
    DEBUG_PRINT(BAROMETER_BASELINE_PRESSURE);
    DEBUG_PRINT(F(" mbar, oversampling mode: "));
    DEBUG_PRINT_LN(BAROMETER_OVERSAMPLING_MODE);
  } else
  {
    // Oops, something went wrong, this is usually a connection problem
    DEBUG_PRINT_LN(F("BMP180 init fail\n\n"));
    while(1); // Pause forever.
  }
}


/*
 * Sets supplied argument to the BMP180 temperature, in degrees Celcius
 * Returns 1 on success, and 0 on failure
 */
short getTemeprature(double &temperature)
{
  char status = 0;
  char measurementDelay = barometer.startTemperature();
  if (measurementDelay > 0) {
    delay(measurementDelay); // Wait for the measurement to complete
    status = barometer.getTemperature(temperature);
    if (status != 1)
    {
      DEBUG_PRINT_LN(F("Error retrieving temperature measurement\n"));
    }
  } else
  {
    DEBUG_PRINT_LN(F("Error starting temperature measurement\n"));
  }
  return status;
}

/*
 * Sets supplied argument to the BMP180 absolute pressure, in milli bar
 * Returns 1 on success, and 0 on failure
 */
short getPressure(double &pressure, double temperature, char oversamplingMode)
{
  char status = 0;
  // Start a pressure measurement using the specified oversamplingMode, from 0 to 3 (highest res, longest wait).
  // If request is successful, the number of ms to wait is returned. Otherwise  0 is returned.
  char measurementDelay = barometer.startPressure(oversamplingMode);
  if (measurementDelay > 0)
  {
    delay(measurementDelay); // Wait for the measurement to complete
    status = barometer.getPressure(pressure, temperature);
    if (status != 1)
    {
      DEBUG_PRINT_LN(F("Error retrieving pressure measurement\n"));
    }
  } else
  {
    DEBUG_PRINT_LN(F("Error starting pressure measurement\n"));
  }
  return status;
}

/*
 * The pressure sensor returns abolute pressure, which varies with altitude.
 * To remove the effects of altitude, use the sealevel function and your current altitude.
 * This number is commonly used in weather reports.
 *
 * Returns the sealevel compensated pressure in milli bar
 */
double getSealevelPressure(double pressure, double altitude)
{
  return barometer.sealevel(pressure, altitude);
}

/*
 * Returns the altitude in metres above sea level
 */
double getAltitude(double absolutePressure, double pressureBaseline)
{
  return barometer.altitude(absolutePressure, pressureBaseline);
}

/********************************
 * HC-SR04 Ultrasound functions *
 ********************************/

void setupUltrasound(short trigPin, short echoPin) {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  DEBUG_PRINT(F("HC-SR04 ultrasound sensor using trig pin: "));
  DEBUG_PRINT(trigPin);
  DEBUG_PRINT(F(", echo pin: "));
  DEBUG_PRINT_LN(echoPin);
}

void HCSR04_do_trig(short trigPin) {
  // Start a reading: Using IO trigger for at least 10µs high level signal
  digitalWrite(trigPin, LOW); // Reset trig pin
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
}

/*
 * Returns the ping duration of the HC-SR04 ultrasound sensor
 *
 *  Formula: uS / 58 = centimeters or uS / 148 =inch; or: the
 *  range = high level time * velocity (340M/S) / 2; we suggest to use over 60ms
 *  measurement cycle, in order to prevent trigger signal to the echo signal
 */
long getUltrasoundPingDuration(short trigPin, short echoPin)
{
  HCSR04_do_trig(trigPin);
  return pulseIn(echoPin, HIGH, ULTRASOUND_PULSE_TIMEOUT);
}

/*
 * Returns the speed of sound (m/s) for a given air temperature (C)
 *
 * See https://en.wikipedia.org/wiki/Speed_of_sound#Practical_formula_for_dry_air
 */
float getSpeedOfSoundForTemperature(float temperature)
{
  return 331.3 + 0.606 * temperature;
}

/*
 * Returns the ping distance of the HC-SR04 ultrasound sensor
 */
long getUltrasoundPingDistance(short trigPin, short echoPin, float speedOfSound)
{
  long duration = getUltrasoundPingDuration(trigPin, echoPin);
  return (duration / 2) / (1000 / speedOfSound);
}

/************************************************
 * Grove - Ear-clip Heart Rate sensor functions *
 ************************************************/

void resetHeartrateSamplesArray()
{
  for (unsigned char i = 0; i < HEARTRATE_NUM_SAMPLES; i++)
  {
    heartrateSamples[i] = 0;
  }
  heartrateCounter = 0;
}

void setupHeartrate(byte interruptPin)
{
  resetHeartrateSamplesArray();

  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), heartrateInterrupt, RISING);
}

void heartrateInterrupt()
{
  long delta = logHeartbeat(heartrateCounter);

  if (delta >= HEARTRATE_MAX_PULSE_INTERVAL)
  {
    /* Sample took to long, something is wrong. Start over. */
    return resetHeartrateSamplesArray();
  }

  heartrateCounter++;

  if (heartrateCounter >= HEARTRATE_NUM_SAMPLES)
  {
    // Set global state
    currentHeartrate = (HEARTRATE_NUM_SAMPLES * 60000) / (heartrateSamples[HEARTRATE_NUM_SAMPLES - 1] - heartrateSamples[0]);
    heartrateCounter = 0;
  }
}

long logHeartbeat(unsigned char idx)
{
  heartrateSamples[idx] = millis();

  short lastIdx = !!idx ? idx - 1 : HEARTRATE_NUM_SAMPLES - 1;
  unsigned long thisValue = heartrateSamples[idx];
  unsigned long lastValue = heartrateSamples[idx - 1];

  return max(lastValue - thisValue, 0);
}

/*********************
 * Utility functions *
 *********************/

void CreateRecord(Record &record, float temperature, float pressure, float sealevelPressure, float altitude, int distance, int heartrate)
{
  record.temperature = temperature;
  record.pressure = pressure;
  record.sealevelPressure = sealevelPressure;
  record.altitude = altitude;
  record.distance = distance;
  record.heartrate = heartrate;
}

void CreateMappedRecord(MappedRecord &mappedRecord, Record record)
{
  mappedRecord.temperature = map(record.temperature, sessionTemperatureBaseline - 2, sessionTemperatureBaseline + 2, 0, 255);
  mappedRecord.pressure = map(record.pressure, BAROMETER_BASELINE_PRESSURE - 3, BAROMETER_BASELINE_PRESSURE + 3, 0, 255);
  mappedRecord.sealevelPressure = map(record.sealevelPressure, sessionPressureBaseline - 3, sessionPressureBaseline + 3, 0, 255);
  mappedRecord.altitude = map(record.altitude, 0, BAROMETER_ALTITUDE * 2, 0, 255);;
  mappedRecord.distance = map(record.distance, 3, 2000, 0, 255);
  mappedRecord.heartrate = map(record.heartrate, 30, 200, 0, 255);
}

/****************
 * Main program *
 ****************/

void setup()
{
  Serial.begin(9600);

  DEBUG_PRINT(F("REBOOT, samling delay: "));
  DEBUG_PRINT(SAMPLING_DELAY);
  DEBUG_PRINT_LN(F(" ms"));

  setupBarometricSensor();
  setupUltrasound(ULTRASOUND_TRIG_PIN, ULRTASOUND_ECHO_PIN);
  setupHeartrate(HEARTRATE_INTERRUPT_PIN);

  // setupFirmata();

  DEBUG_PRINT_LN(F("\n--------\n"));
}

void serialSendRecord(MappedRecord record)
{
  if (SERIAL_PRINT_COMPOUND_VALUE)
  {
    int compoundOutput = round((record.temperature + record.sealevelPressure + record.distance + record.heartrate) / 4);
    Serial.print(compoundOutput);
  } else {
    Serial.println(record.temperature);
    // Serial.println(record.pressure);
    Serial.println(record.sealevelPressure);
    // Serial.println(record.altitude);
    Serial.println(record.distance);
    Serial.println(record.heartrate);
  }
}

void loop() {
  double temperature, pressure, sealevelPressure, altitude;
  getTemeprature(temperature);
  getPressure(pressure, temperature, BAROMETER_OVERSAMPLING_MODE);
  sealevelPressure = getSealevelPressure(pressure, BAROMETER_ALTITUDE);
  altitude = getAltitude(pressure, BAROMETER_BASELINE_PRESSURE);


  if (!sessionTemperatureBaseline)
  {
    sessionTemperatureBaseline = temperature;
    DEBUG_PRINT(F("Session pressure baseline: "));
    DEBUG_PRINT(sessionTemperatureBaseline);
    DEBUG_PRINT_LN(F("°C"));
  }
  if (!sessionPressureBaseline)
  {
    sessionPressureBaseline = sealevelPressure;
    DEBUG_PRINT(F("Session pressure baseline: "));
    DEBUG_PRINT(sessionPressureBaseline);
    DEBUG_PRINT_LN(F("mbar"));
  }

  DEBUG_PRINT(F("Temperature: "));
  DEBUG_PRINT(temperature);
  DEBUG_PRINT_LN(F("°C"));
  DEBUG_PRINT(F("Pressure (absolute): "));
  DEBUG_PRINT(pressure);
  DEBUG_PRINT_LN(F(" mbar"));
  DEBUG_PRINT(F("Pressure (sealevel): "));
  DEBUG_PRINT(sealevelPressure);
  DEBUG_PRINT_LN(F(" mbar"));
  DEBUG_PRINT(F("Altitude: "));
  DEBUG_PRINT(altitude);
  DEBUG_PRINT_LN(F(" m"));

  float speedOfSound = getSpeedOfSoundForTemperature(temperature);
  int distance = getUltrasoundPingDistance(ULTRASOUND_TRIG_PIN, ULRTASOUND_ECHO_PIN, speedOfSound);
  if (distance >= ULTRASOUND_UPPER_BOUNDS || distance <= ULTRASOUND_LOWER_BOUNDS)
  {
    DEBUG_PRINT_LN(F("Out of range"));
  } else
  {
    DEBUG_PRINT(F("Distance: "));
    DEBUG_PRINT(distance);
    DEBUG_PRINT_LN(F(" mm"));
  }

  DEBUG_PRINT(F("Heartrate: "));
  DEBUG_PRINT(currentHeartrate);
  DEBUG_PRINT_LN(F(" BPM"));

  Record record;
  CreateRecord(record, temperature, pressure, sealevelPressure, altitude, distance, currentHeartrate);

  MappedRecord mappedRecord;
  CreateMappedRecord(mappedRecord, record);

  DEBUG_PRINT_LN(F("\nOUTPUT:"));
  DEBUG_PRINT_LN(F("\n--------\n"));

  serialSendRecord(mappedRecord);

  DEBUG_PRINT_LN(F("\n--------\n"));
  delay(SAMPLING_DELAY);
}
