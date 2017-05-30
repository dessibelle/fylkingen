#include <SFE_BMP180.h>
#include <Wire.h>
// #include <Firmata.h>

/*
 * Function that printf and related will use to print
 */
int serial_putchar(char c, FILE* f)
{
    if (c == '\n')
    {
      serial_putchar('\r', f);
    }
    return Serial.write(c) == 1? 0 : 1;
}

/****************
 * Global state *
 ****************/

FILE serial_stdout;
double sessionPressureBaseline;
#define SAMPLING_DELAY 80 // Must be greater than 60ms

/*********************************
 * BMP180 Barometic sensor setup *
 *********************************/

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


/*************************************
 * BMP180 Barometic sensor functions *
 *************************************/

void setupBarometricSensor()
{
  if (barometer.begin())
  {
    Serial.print(F("BMP180 init success, altitude: "));
    Serial.print(BAROMETER_ALTITUDE);
    Serial.print(F(", oversampling mode: "));
    Serial.println(BAROMETER_OVERSAMPLING_MODE);
  } else
  {
    // Oops, something went wrong, this is usually a connection problem
    Serial.println(F("BMP180 init fail\n\n"));
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
      Serial.println(F("Error retrieving temperature measurement\n"));
    }
  } else
  {
    Serial.println(F("Error starting temperature measurement\n"));
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
      Serial.println(F("Error retrieving pressure measurement\n"));
    }
  } else
  {
    Serial.println(F("Error starting pressure measurement\n"));
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
  return pulseIn(echoPin, HIGH);
}

/*
 * Returns the ping distance of the HC-SR04 ultrasound sensor
 */
long getUltrasoundPingDistance(short trigPin, short echoPin)
{
  long duration = getUltrasoundPingDuration(trigPin, echoPin);
  return (duration/2) / 2.91;
}

/****************
 * Main program *
 ****************/

void setup()
{
  Serial.begin(9600);
  Serial.println(F("REBOOT"));

  // Set up stdout
  fdev_setup_stream(&serial_stdout, serial_putchar, NULL, _FDEV_SETUP_WRITE);
  stdout = &serial_stdout;

  printf("My favorite number is %6d!\n", 12);

  setupBarometricSensor();
  setupUltrasound(ULTRASOUND_TRIG_PIN, ULRTASOUND_ECHO_PIN);

  Serial.println(F("\n--------\n"));
}

void loop() {
  double temperature, pressure, sealevelPressure, altitude, pressureDifferential;
  getTemeprature(temperature);
  getPressure(pressure, temperature, BAROMETER_OVERSAMPLING_MODE);
  sealevelPressure = getSealevelPressure(pressure, BAROMETER_ALTITUDE);
  altitude = getAltitude(pressure, BAROMETER_BASELINE_PRESSURE);

  if (!sessionPressureBaseline)
  {
    sessionPressureBaseline = sealevelPressure;
  }
  pressureDifferential = sealevelPressure - sessionPressureBaseline;

  Serial.print(F("Temperature: "));
  Serial.print(temperature, 2);
  Serial.println(F("°C"));
  Serial.print(F("Pressure (absolute): "));
  Serial.print(pressure, 2);
  Serial.println(F(" mbar"));
  Serial.print(F("Pressure (sealevel): "));
  Serial.print(sealevelPressure, 2);
  Serial.println(F(" mbar"));
  Serial.print(F("Altitude: "));
  Serial.print(altitude, 2);
  Serial.println(F(" m"));
  Serial.print(F("Pressure differential: "));
  Serial.print(pressureDifferential, 2);
  Serial.println(F(" mbar"));

  long distance = getUltrasoundPingDistance(ULTRASOUND_TRIG_PIN, ULRTASOUND_ECHO_PIN);
  if (distance >= ULTRASOUND_UPPER_BOUNDS || distance <= ULTRASOUND_LOWER_BOUNDS)
  {
    Serial.println(F("Out of range"));
  } else
  {
    Serial.print(F("Distance: "));
    Serial.print(distance);
    Serial.println(F(" mm"));
  }

  Serial.println(F("\n--------\n"));
  delay(SAMPLING_DELAY);
}
