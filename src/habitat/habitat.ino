#define USE_ARDUINO_INTERRUPTS true    // Set-up low-level interrupts for most acurate BPM math.
#include <PulseSensorPlayground.h>     // Includes the PulseSensorPlayground Library.

// #define DEBUG // This enables debug logging, uncomment to enable debug loggin
#define SERIAL_PLOT_VALUES 1
#define SERIAL_OUTPUT_COMPOUND_VALUE 0

#define SERIAL_PLOT_SAMPLE_SEPARATOR ' '
#define SERIAL_PLOT_RECORD_SEPARATOR '\n'


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


#define TEMPERATURE_DEG_C     22

#define MAP_DISTANCE_MM_MIN       30
#define MAP_DISTANCE_MM_MAX       800
#define MAP_HEARTRATE_BPM_MIN     30
#define MAP_HEARTRATE_BPM_MAX     200
#define MAP_AMPLITUDE_MIN         250
#define MAP_AMPLITUDE_MAX         1000

/***********************************
 * Internal communication protocol *
 ***********************************/

struct dataRecord
{
  int distance;
  int heartrate;
  int amplitude;
};

typedef struct dataRecord Record;

struct mappedRecord
{
  byte distance;
  byte heartrate;
  byte amplitude;
};

typedef struct mappedRecord MappedRecord;


/****************
 * Global state *
 ****************/

float gSpeedOfSound = 331;

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
 * PulseSensor.com heart rate monitor *
 **************************************/

PulseSensorPlayground pulseSensor;

/*
 * Determine which Signal to "count as a beat" and which to ignore.
 * Use the "Gettting Started Project" to fine-tune Threshold Value beyond default setting.
 * Otherwise leave the default "550" value.
 */

#define HEARTRATE_ANALOG_INPUT_PIN  0       // Pin used for heartreate sensor, 0 means A0 on the Arduino
#define HEARTRATE_NOISE_TRESHOLD    550


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
long getUltrasoundPingDistance(short trigPin, short echoPin)
{
  long duration = getUltrasoundPingDuration(trigPin, echoPin);
  return (duration / 2) / (1000 / gSpeedOfSound);
}

/************************************************
 * PulseSensor.com heart rate monitor functions *
 ************************************************/

void setupHeartrate(byte inputPin, short treshold)
{
  // resetHeartrateSamplesArray();

  pulseSensor.analogInput(inputPin);
  pulseSensor.setThreshold((int)treshold);

  if (!pulseSensor.begin()) {
    Serial.println("Error initializing pulse sensor!");  //This prints one time at Arduino power-up,  or on Arduino reset.
  }
}

/*********************
 * Utility functions *
 *********************/

void CreateRecord(Record &record, int distance, int heartrate, int amplitude)
{
  record.distance = distance;
  record.heartrate = heartrate;
  record.amplitude = amplitude;
}

void CreateMappedRecord(MappedRecord &mappedRecord, Record record)
{
  mappedRecord.distance = (byte)map(record.distance, MAP_DISTANCE_MM_MIN, MAP_DISTANCE_MM_MAX, 0, 255);
  mappedRecord.heartrate = (byte)map(record.heartrate, MAP_HEARTRATE_BPM_MIN, MAP_HEARTRATE_BPM_MAX, 0, 255);
  mappedRecord.amplitude = (byte)map(record.amplitude, MAP_AMPLITUDE_MIN, MAP_AMPLITUDE_MAX, 0, 255);
}

/****************
 * Main program *
 ****************/

void setup()
{
  Serial.begin(57600);

  DEBUG_PRINT(F("REBOOT, samling delay: "));
  DEBUG_PRINT(SAMPLING_DELAY);
  DEBUG_PRINT_LN(F(" ms"));

  gSpeedOfSound = getSpeedOfSoundForTemperature(TEMPERATURE_DEG_C);

  setupUltrasound(ULTRASOUND_TRIG_PIN, ULRTASOUND_ECHO_PIN);
  setupHeartrate(HEARTRATE_ANALOG_INPUT_PIN, HEARTRATE_NOISE_TRESHOLD);

  DEBUG_PRINT_LN(F("\n--------\n"));
}

byte getCompoundValue(MappedRecord record)
{
  return (byte)round((record.distance + record.heartrate + record.amplitude) / 3);
}

void serialSendRecord(MappedRecord record)
{
  if (SERIAL_OUTPUT_COMPOUND_VALUE)
  {
    byte compoundOutput = getCompoundValue(record);
    Serial.write(compoundOutput);
  } else {
    Serial.write(record.distance);
    Serial.write(record.heartrate);
    Serial.write(record.amplitude);
  }
}

void plotRecord(MappedRecord record)
{
  byte compoundOutput = getCompoundValue(record);

  if (SERIAL_OUTPUT_COMPOUND_VALUE)
  {
    byte compoundOutput = getCompoundValue(record);
    Serial.println(compoundOutput);
  } else {
    Serial.print(record.distance);
    Serial.print(SERIAL_PLOT_SAMPLE_SEPARATOR);
    Serial.print(record.heartrate);
    Serial.print(SERIAL_PLOT_SAMPLE_SEPARATOR);
    Serial.print(record.amplitude);
    Serial.print(SERIAL_PLOT_SAMPLE_SEPARATOR);
    Serial.print(compoundOutput);
    Serial.print(SERIAL_PLOT_RECORD_SEPARATOR);
  }
}

void loop() {
  int distance = getUltrasoundPingDistance(ULTRASOUND_TRIG_PIN, ULRTASOUND_ECHO_PIN);
  if (distance >= ULTRASOUND_UPPER_BOUNDS || distance <= ULTRASOUND_LOWER_BOUNDS)
  {
    DEBUG_PRINT(F("Out of range:"));
    DEBUG_PRINT_LN(distance);
  } else
  {
    DEBUG_PRINT(F("Distance: "));
    DEBUG_PRINT(distance);
    DEBUG_PRINT_LN(F(" mm"));
  }

  // A beat happened since "last sample"?
  // if (pulseSensor.sawStartOfBeat()) {}

  pulseSensor.sawStartOfBeat();

  int currentBpm = pulseSensor.getBeatsPerMinute();
  int currentAmplitude = pulseSensor.getPulseAmplitude();
  int latestAmplitudeSample = pulseSensor.getLatestSample();

  DEBUG_PRINT(F("Current heartrate:"));
  DEBUG_PRINT(currentBpm);
  DEBUG_PRINT_LN(F(" bpm"));

  DEBUG_PRINT(F("Current amplitude:"));
  DEBUG_PRINT(currentAmplitude);
  DEBUG_PRINT_LN(F(""));

  DEBUG_PRINT(F("Latest amplitude sample:"));
  DEBUG_PRINT(latestAmplitudeSample);
  DEBUG_PRINT_LN(F(""));

  Record record;
  CreateRecord(record, distance, currentBpm, latestAmplitudeSample);

  MappedRecord mappedRecord;
  CreateMappedRecord(mappedRecord, record);

  DEBUG_PRINT_LN(F("\nOUTPUT:"));
  DEBUG_PRINT_LN(F("\n--------\n"));

  if (SERIAL_PLOT_VALUES) {
    plotRecord(mappedRecord);
  } else {
    serialSendRecord(mappedRecord);
  }

  DEBUG_PRINT_LN(F("\n--------\n"));

  delay(SAMPLING_DELAY);
}
