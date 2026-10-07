// for xiao esp32-c3  (no FPU so thats why there is "f" on most values)
// Icarus rocket
// mpc controller

//code breakdown
//sensor logger for icm-20948 and bmp390 sensors which writes data onto an sd card in CSV format so easy to graph.
//active aero sim to slow down the rocket to reach target apogee. active aero sim controls servo motor for the airbrakes.

//libs
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <ESP32Servo.h>


#define SEALEVELPRESSURE_PA (101325.0) //sea level in pascals
#define BMP_PRESSURE_IS_PA 1 // 1 means pa // 0 means hpa

// SPI linking 
const int chipSelect = D3;
const int SD_SCK  = D8;
const int SD_MISO = D2;  // not the normal MISO pin, the default one messes with boot on the xiao and drops it into sleep states
const int SD_MOSI = D10;

//sensor objects
Adafruit_BMP3XX bmp;
Adafruit_ICM20948 icm;
Servo myservo;

//  data logging object
File dataFile;

// creating a ram buffer to compile before writing // 256bytes x 100 depth = 25.6KB of SRAM out of 400 total
QueueHandle_t sdQueue;
TaskHandle_t sdTaskHandle;
#define LOG_BUF_LEN   256     //ram buffer 
#define LOG_QUEUE_LEN 100     //queue depth
char logBuffer[LOG_BUF_LEN];
bool loggingEnabled = true;

// better than delays logs at 50hz (20ms)
unsigned long previousMicros = 0;
const unsigned long INTERVAL_US = 20000UL; //20ms
unsigned long loopDtUs = 0;

//rocket airbreak master controller
enum FlightState
{
  PAD_IDLE,
  ASCENT,
  DESCENT,
  LANDED
};
FlightState currentState = PAD_IDLE;

//altitude variables
float groundAltitude = 0.0;
float currentAltitude = 0.0;
float previousAltitude = 0.0;
unsigned long previousTime = 0;

// distance and velocity tracking for code starter. code is gonna kick in a 1m as the rocket will reach that after 140ms with roughly 101m/s (acceleration) off launch. i think anyway, based on math from 20.1m/s at 2m
//starting code
float currentvelocity = 0.0; // used only on pad not in air (drifts values alot)
float barovelocity = 0.0;
float distancetravelled = 0.0;
float predictedVel = 0.0;
float baroAccel = 0.0;
float alpha = 0.4; // controls how strongly the barometer measurements corrects velocity estimates // higher the number higher the noise but faster tracking
float beta = 0.05; // similar to above but extremely noise so set very low 
float fusedVelocity = 0.0; // the velocity the mpc uses as the baro is filtered at 10hz in code even though the sensor runs at 50hz, im making it purposely lower
const float BARO_TRUST = 0.35;  // how hard each baro tick corrects the fusion // 0.35 means 35% of the gap on every tick basically enough to stop accelerometer drift

//sanity clamps for BMP
const float RHO_MIN = 0.6;
const float RHO_MAX = 1.4;
const float VEL_MIN = -150.0;
const float VEL_MAX = 200.0; // real max vel is ~116m/s so 85m/s of headroom 

//backup apogee detection
int fallingCount = 0; //back up incase velocity estimate is wack
float peakAltitude = 0.0; // logs highest altitude reach at all times

//landing detection and health check
unsigned long stillSince =0;
volatile bool closeRequested = false;
unsigned int droppedLogs = 0;
unsigned int missedBaro = 0;

//referance altitude and brands for landing detector
float landingRefAlt = 0.0;
const float LANDING_ALT_MAX  = 25.0;   // max altitude from pad altitude which landing could happen at. pad is roughly 103m from sealevel so max would be +128m and -78m more than big enough headroom
const float LANDING_ALT_BAND = 3.0;    // altitude must not move more than this
const float LANDING_ACC_BAND = 1.5;    // |a| must be within this of 1g

// REAL TIME MASS CALC overkill but fun
unsigned long launchTime = 0; // total launch time from burn start
float dryMass = 0.7293; // DRY MASS WEIGHT
float padMass = 0.789; // WET MASS 
float currentMass = padMass;
float burnTime = 1.4;  //1.4s rocket burn // G78G-7 aerotech rocket motor 

//servo controller
int currentServoAngle = 126;    // tracks servo degrees
const int SERVO_MIN = 0;        // hard lower limit // open = max drag
const int SERVO_MAX = 126;      // hard upper limit // closed = least drag
const int SERVO_STEP = 3;       // maximum degrees per step
const int SERVO_DEADBAND = 5;   // ignores any movements under 5 degrees 

//mpc varibles 
float area = 0.0019635;   // cfd referenace area 
float rho = 1.225;        // density of air (1.225kg/m^3)
float CDClean = 0.4724;   // airbrakes retracted //validated by my CFD
float CDMax   = 0.8776;   // airbrakes deployed  //validated by my CFD
float CD = CDClean;       // mpc starts clean each run
int apogeeTgt = 560;      //apoge target in metres (560m = 1837 feet) (580m = 1902 feet) (600m = 1968 feet)
const float overshootPenalty = 1.25; // tells the mpc to aim for slight undershoot rather than extreme overshoot
const float MPC_ARM_MARGIN_S = 0.1; // 0.1s delay before servo is aloud to move after burn phase so it doesnt strain itself, might change to 0.3seconds

//apogee airbrake controller
const unsigned long AIRBRAKE_DESCENT_DELAY_MS = 3000UL; // 3 second delay after apogee to max airbrakes
unsigned long apogeeTime = 0;
bool brakesDeployed = false; // check airbrake state

// MPC stuff
#define MPC_CANDIDATES 41  // amount of sims
float bestApogee = 0.0;          //what the winning candidate predicted 

// accelerometer scale correction 
float accelScale = 1.0f;

//tells compiler to run the SD background tasks
void logToSDTask(void *pvParameters);

// makes sure everything is done in pascals 
float pressurePa()
{
#if BMP_PRESSURE_IS_PA
  return bmp.pressure;
#else
  return bmp.pressure * 100.0;
#endif
}

//altitude checker
float altitudeFromPa()
{
  return 44330.0f * (1.0f - powf(pressurePa() / (float)SEALEVELPRESSURE_PA, 0.1903f)); //barometric formula but inverted for height
}

// density checker
float airDensity()
{
  return pressurePa() / (287.05f * (bmp.temperature + 273.15f));  // ideal gas law rearranged 
}

//                                       apogee predictor                                            // 

// sim predicts what do with the brakes based on the logic of if i slammed the brakes now and coast what would my apogee be
// sim comes from:        m·dv/dt = -mg - ½·ρ·Cd·A·v²       which goes to       v·dv/dh = -g - k·v²        where k = ρ·Cd·A/(2m) where time is swapped for height using   v·dv/dh = dv/dt
float predictApogee(float cdSim, float alt0, float v0, float mass)
{
  if (v0 <= 0.0f) return alt0; // checks how much the rocket will climb at current speed, (also techincally kills the sim at apogee where velocity becomes negative along Y axis)
  float k = (rho * cdSim * area) / (2.0f * mass); // bundles the whole drag setup into one number
  if (k < 1e-9f) return alt0; // divide-by-zero guard (would of put a NaN but it woulda killed it)
  float h = (1.0f / (2.0f * k)) * logf(1.0f + (k * v0 * v0) / 9.81f); // sim output (how much higher the rocket will climb until apogee (velcoity =0))
  float rhoMid = rho * (1.0f - (0.5f * h) / 8500.0f); // redo sim but with decaying density (air thinner not by much but mayaswell)
  if (rhoMid < RHO_MIN) rhoMid = RHO_MIN; // stops a insane H value given (basically if h was too big it would turn rhoMid negative basically saying neagtive density which means negative drag so bad)
  // redoes the sim part with the new density (more accurate density)
  k = (rhoMid * cdSim * area) / (2.0f * mass); 
  if (k < 1e-9f) return alt0;
  h = (1.0f / (2.0f * k)) * logf(1.0f + (k * v0 * v0) / 9.81f);
  return alt0 + h; // coast height (h) added to curent altitude (predicted apogee)

  //lots of math, lokey been cooking this up for the past few months since i made the first iteration, i wanna say its perfect now.
}

//                                      main setup                                  //
void setup()
{
  // setting up SD card
  Serial.begin(115200); //115200 baud rate
  Serial.setTxTimeoutMs(0); // doesnt block the loop if not device is attached
  Serial.println("initialising");
  delay (2000); // safety net

  // Creates the queue in RAM and launches the background SD logger.
  //also kills logging if queue crashes so it doesnt kill the entire code
  sdQueue = xQueueCreate(LOG_QUEUE_LEN, sizeof(logBuffer));
  if (sdQueue == NULL)
  {
    Serial.println("QUEUE ALLOCATION FAILED - SD logging disabled");
    loggingEnabled = false;
  }
  xTaskCreate(
    logToSDTask,   //task function
    "SD_Task",     //name
    4096,          //stack size
    NULL,          //parameters
    1,             //priority
    &sdTaskHandle  //handle
  );

  //the xiao needs its own timers allocated before it'll write servos
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(50);
  myservo.attach(D1, 500, 2400); // servo pin is now D1


  //                    SERVO WIGGLE TEST                                         //
  Serial.println("performing servo wiggle test");
  myservo.write(SERVO_MIN); // 0 Degrees (Fully Open)
  delay(1000);
  myservo.write(SERVO_MAX); // 126 Degrees (Fully Closed)
  delay(1000);
  currentServoAngle = SERVO_MAX; // Ensure logic matches physical state
  Serial.println("Servo test complete now ready to launch");


  //cranking up the speed on i2c bus to 400kHz
  Wire.begin();
  Wire.setClock(400000);

  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, chipSelect); //forcing spi pins

  if (!SD.begin(chipSelect, SPI, 4000000)) // checks SD card is working and kills logging if its fails so doesnt kill code
  {
    Serial.println("SD Card Failed - CONTINUING WITHOUT LOGGING");
    loggingEnabled = false;
  }

  Serial.println("Initializing BMP390 Sensor");
  if (!bmp.begin_I2C())
  {
    Serial.println("BMP390 Failed to initialize");
    while(1) { vTaskDelay(pdMS_TO_TICKS(100)); } //properly block BMP390 if it fail fails (better than yield)
  }


  // built in filtering and oversampling + preflight checker 
  if (!bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X)) Serial.println("WARNING: temp OSR rejected");
  if (!bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X))    Serial.println("WARNING: press OSR rejected");
  if (!bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3))       Serial.println("WARNING: IIR coeff rejected");
  if (!bmp.setOutputDataRate(BMP3_ODR_50_HZ))                Serial.println("WARNING: 50Hz ODR REJECTED - oversampling too high");


  // checking ICM-20948 sensor
  Serial.println("initializing TDK INVENSENSE ICM-20948");
  if (!icm.begin_I2C())
  {
    Serial.println("ICM-20948 Failed to initialize");
    while(1) { vTaskDelay(pdMS_TO_TICKS(100)); } // same reason as BMP 390
  }

  icm.setAccelRange(ICM20948_ACCEL_RANGE_16_G); // sets IMU to +or- 16G / peak G should be 11G
  icm.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS); //sets IMU to max 2000 degrees of roll per a second

  // read ICM data 10 times to clear the buffer and also help it settle down to a stable state
  for (int i = 0; i < 10; i++)
  {
    sensors_event_t accel, gyro, temp, mag;
    icm.getEvent(&accel, &gyro, &temp, &mag);
    delay(50);
  }

  //BMP reads where it is compared to sea level to get ground level. im a genius BTW
  Serial.println("Calibrating Ground Altitude");
  float altSum = 0.0f;
  int   altN = 0;
  for (int i = 0; i < 20; i++)
  {
    if (bmp.performReading()) { altSum += altitudeFromPa(); altN++; }
    delay(50);
  }
  if (altN > 0) groundAltitude = altSum / (float)altN; // pad altitude (sets the baseline altitude as a referance for all new altitude readings)
  else
  {
    groundAltitude = altitudeFromPa();
    Serial.println("WARNING: no good baro reads during ground calibration - using last value");
  }
  Serial.print("ground alt samples used: "); Serial.println(altN);

  previousAltitude = groundAltitude; // checks if pad alt is correct and not nonsense 
  previousTime = millis();

  // sanity check before flight
  Serial.print("BMP pressure (Pa): "); Serial.println(pressurePa());
  Serial.print("pad air density: "); Serial.println(airDensity()); //check if it is 1.225 or not
  Serial.print("apogee target: "); Serial.println(apogeeTgt);

  // creating data file and writing
  dataFile = SD.open("/flight_data.csv", FILE_APPEND);
  if (dataFile)
  {
    dataFile.println("Time(ms),State,Alt,Vel,ServoAngle,Temp_C,Press_Pa,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,MagX,MagY,MagZ,CD,PredApogee"); // logging column headers
    dataFile.flush(); // flushes data to the other void code 
    Serial.println("sensors are logging, READY FOR LAUNCH GO GO GOOOOOOO");
  }
  else
  {
    Serial.println("Error starting file, check for corruption");
    loggingEnabled = false; // just incase it dies during flight or when the rocket is on the rail
  }

  delay(5000); //safety net
  previousMicros = micros();
  previousTime   = millis();
}

//                            main code                               //
void loop()
{
  // timing stuff for sim loops so if sd or mpc messes up this fixes it or prevents it
  unsigned long currentMicros = micros();
  if (currentMicros - previousMicros >= INTERVAL_US)
  {
    loopDtUs = currentMicros - previousMicros;
    float dt = (float)loopDtUs * 1e-6f; // delta T in seconds
    previousMicros = currentMicros;
    unsigned long currentMillis = millis();

    bool baroOk = true;
    if (!bmp.performReading()) // checks for missed bmp readings and replaces them with last good altitude reading
      {
        baroOk = false;
        missedBaro++;
      }
    else
    {
      float altNew = altitudeFromPa();
      if (altNew > -500.0f && altNew < 10000.0f) { currentAltitude = altNew; } //rejects non finite altitude so the code doesnt use it
      else { baroOk = false; missedBaro++; }
    }
    sensors_event_t accel, gyro, mag, temp;
    icm.getEvent(&accel, &gyro, &temp, &mag);

    //pad calibration
    float ax = accel.acceleration.x * accelScale; //accelx
    float ay = accel.acceleration.y * accelScale; //accely
    float az = accel.acceleration.z * accelScale; //accelz


    float agl = currentAltitude - groundAltitude; // height from the pad baseline reading

    // fused the imu filter with the built in nano filter
    if (currentState == PAD_IDLE)
    {
      //abs acceleration in all directions
       float accelMag = sqrtf(sq(ax) + sq(ay) + sq(az));
      // subtract 1G cus gravity
      float netAccel = accelMag - 9.81f;

      //filter to ignore anything less than (2m/s)^2 or 0.204G
      if (netAccel > 2.0f)
      {
        currentvelocity += netAccel * dt;
      }
      else
      {
        currentvelocity = 0.0f;
        distancetravelled = 0.0f;
      }

      // Always add velocity to distance. (If we are on the pad, velocity is 0, so distance stays 0).
      distancetravelled += currentvelocity * dt;


      //checks for ascent 
      if (distancetravelled >= 1.0f || agl >= 50.0f) // checks till 50m above pad
      {
        currentState = ASCENT;
        launchTime = (millis() > 150UL) ? (millis() - 150UL) : millis(); // estimatees when igntion happened
        fusedVelocity = currentvelocity;
        barovelocity = 0.0f;   //starts clean so it ignores noise from being on pad
        baroAccel = 0.0f;      //integrates with no decay so it drifts worst
        peakAltitude = agl;
        fallingCount = 0;
        Serial.println(" Nominal Launch , Sensors are Logging ");
      }
    }


    if (currentState != PAD_IDLE)
    {
      float aVert = az - 9.81f; // vertical velocity 
      fusedVelocity += aVert * dt; //integrates it forward 
      fusedVelocity = constrain(fusedVelocity, VEL_MIN, VEL_MAX); //backup incase ICM glitches or saturates
    }

    //                                        ALPHA-BETA BARO VELOCITY FILTER                                     //

    float timeSinceLastVel = (currentMillis - previousTime) / 1000.0f;
    if (baroOk && timeSinceLastVel >= 0.1f) // makes the filter run at 10hz (slower than the 20ms(50hz) loop so it doesnt get buried in noise)
    {
      float rawVel = (currentAltitude - previousAltitude) / timeSinceLastVel; // raw velocity is simple and very noise so needs filtering

      // Predict next state
      predictedVel = barovelocity + (baroAccel * timeSinceLastVel);

      // Calculate difference between measured and predicted
      float residual = rawVel - predictedVel;

      // Update estimates using Alpha and Beta weights
      barovelocity = predictedVel + (alpha * residual);
      baroAccel = baroAccel + (beta * residual / timeSinceLastVel); // correct step for accel at only 5% so the output isnt oscillating wildly
      
      float trust = BARO_TRUST;
      if (currentState == ASCENT && (currentMillis - launchTime) < (unsigned long)(burnTime * 1000.0f)) trust = 0.0f;
      fusedVelocity += trust * (barovelocity - fusedVelocity); // fusion step // Pulls the accelerometer-propagated estimate 35% of the way toward the barometer estimate.

      previousAltitude = currentAltitude;
      previousTime = currentMillis;
    }

    // Only runs MPC and mass calcs if the rocket is actively flying upwards.
    if (currentState == ASCENT)
    {
      //                                                   REAL TIME MASS CALC                        //

      float elapsed = (currentMillis - launchTime) / 1000.0f;
      if (elapsed < burnTime) // linear from wet mass to dry mass (obv not linear in real life but its only 1.4s burn time)
      {
        currentMass = padMass - ((padMass - dryMass) * (elapsed / burnTime));
      }
      else
      {
        currentMass = dryMass;
      }

      // velocity test, resets falling until it is actually falling
      if (agl > peakAltitude)
      {
        peakAltitude = agl;
        fallingCount = 0;
      }
      else if (agl < peakAltitude - 1.0f)
      {
        fallingCount++;
      }

      // Check for apogee for descent
      if (peakAltitude > 50.0f && (fusedVelocity < -2.0f || fallingCount >= 5))
      {
        currentState = DESCENT;
        myservo.write(SERVO_MAX); // stowed till parachute deployment
        currentServoAngle = SERVO_MAX;
        apogeeTime = currentMillis;
        brakesDeployed = false;

        landingRefAlt = agl;
        stillSince = 0;
        Serial.println("APOGEE REACHED - DESCENT STATE ENGAGED");
      }
      else
      {
        // new burnphase check, keep airbreaks stored till burn is over
        if (elapsed >= burnTime + MPC_ARM_MARGIN_S)
        {
//                          MPC CONTROLLER                                         //
          // NEW MPC REFINEMENT AND PHYSYICS SIM (lemme cook gang)

          float rhoNow = airDensity(); // call
          rho = constrain(rhoNow, RHO_MIN, RHO_MAX); // variable based on call also validates it 3 times from the 3 density checks at the start of code 

          //ranks 41 candidates based on cost, and punishes the MPC for overshooting too much and tells it to undershoot if its going to overshoot (slam brakes)
          float bestCost = 1000000.0f; // score tracker for the 11 sims
          float bestCD = CDClean; // safety net if all 41 sims fail 
          bestApogee = agl;


          // Sims 41 different airbrake states
          for (int c = 0; c < MPC_CANDIDATES; c++)
          {
            float CDSim = CDClean + ((float)c * ((CDMax - CDClean) / (float)(MPC_CANDIDATES - 1)));
            float altSim = predictApogee(CDSim, agl, fusedVelocity, currentMass);

            // Check how close this simulation got to the target apogee
            float err = altSim - (float)apogeeTgt;
            float cost = (err > 0.0f) ? (err * overshootPenalty) : (-err);

            if (cost < bestCost)
            {
              bestCost = cost;
              bestCD = CDSim; // Saves the best CD to be used by servo
              bestApogee = altSim;
             
            }
          }
          CD = bestCD;

          //servo smoothing and limits so the servo doesnt strain itself
          int targetAngle = map((long)(CD * 10000), (long)(CDClean * 10000), (long)(CDMax * 10000), SERVO_MAX, SERVO_MIN);
          targetAngle = constrain(targetAngle, SERVO_MIN, SERVO_MAX);

          //deadband stops the servo from attempting a 5 degree or less movement
          if (abs(targetAngle - currentServoAngle) >= SERVO_DEADBAND)
          {
            if (targetAngle > currentServoAngle)
            {
              currentServoAngle += SERVO_STEP;
              if (currentServoAngle > targetAngle) currentServoAngle = targetAngle;
            }
            else if (targetAngle < currentServoAngle)
            {
              currentServoAngle -= SERVO_STEP;
              if (currentServoAngle < targetAngle) currentServoAngle = targetAngle;
            }
            myservo.write(currentServoAngle);
          }
        }
        else
        {
          // Keep airbrakes strictly closed/stowed during the motor burn phase
          currentServoAngle = SERVO_MAX;
          myservo.write(currentServoAngle);
        }
      }
    }

    if (currentState == DESCENT) // landing detection and missed logs/reads
    {
      if (!brakesDeployed && (currentMillis - apogeeTime) >= AIRBRAKE_DESCENT_DELAY_MS)
      {
        brakesDeployed = true;
        currentServoAngle = SERVO_MIN;   // 0 degrees, fully open, max drag
        myservo.write(currentServoAngle);
        Serial.println("parachute done - airbrakes to full");
      }

      float accelMagNow = sqrtf(sq(ax) + sq(ay) + sq(az));
      bool lowEnough  = (agl < LANDING_ALT_MAX);
      bool altStable  = (fabsf(agl - landingRefAlt) < LANDING_ALT_BAND);
      bool accelQuiet = (fabsf(accelMagNow - 9.81f) < LANDING_ACC_BAND);


      if (lowEnough && altStable && accelQuiet)
      {
        if (stillSince == 0) stillSince = currentMillis;
         if (currentMillis - stillSince >= 5000)
         {
          currentState = LANDED;
          closeRequested = true;
          myservo.detach();
          Serial.print("dropped log lines: "); Serial.println(droppedLogs);
          Serial.print("missed baro reads: "); Serial.println(missedBaro);
          Serial.println("LANDED - closing file");
         }
      }
      else 
      {
        landingRefAlt =agl;
        stillSince =0;
      }
    }

    if (loggingEnabled && (currentState == ASCENT || currentState == DESCENT)) //stop logging once landed
    {
      // due to the xiao having a 32bit RISC-V chip i can log much faster than a normal chip as this bit underneath basically combines all data into a single text array in RAM. (basically this thing is seriously fast
      // thought we needed faster as the rocket is going mach 0.4)
      float pressureLog = pressurePa();
      snprintf(logBuffer, sizeof(logBuffer), "%lu,%d,%.2f,%.2f,%d,%.2f,%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.4f,%.1f\n",
        currentMillis, currentState, agl, fusedVelocity, currentServoAngle,
        bmp.temperature, pressureLog,
        accel.acceleration.x, accel.acceleration.y, accel.acceleration.z,
        gyro.gyro.x, gyro.gyro.y, gyro.gyro.z,
        mag.magnetic.x, mag.magnetic.y, mag.magnetic.z,
        CD, bestApogee);
      //sends the data to the queue without waiting so 0 tick delay basically
      if (xQueueSend(sdQueue, &logBuffer, 0) != pdPASS) droppedLogs++; //count what gets thrown away
    }
  }
}

// background SD tasks
// This infinite loop runs completely separate from the rest of the code, not in parellel but instead in short bursts where it can
// waits for data to arrive in the queue, writes it, and flushes it.
// hopefully will remove all lag spikes from the code so physics sim and servo moves freely
void logToSDTask(void *pvParameters)
{
  char taskBuffer[LOG_BUF_LEN];
  unsigned long lastFlush = 0;
  if (sdQueue == NULL) { while (true) { vTaskDelay(pdMS_TO_TICKS(1000)); } } //handles crashes so it doesnt kill the code
  while (true)
  {
    // Wait for data to appear in the Queue
    if (xQueueReceive(sdQueue, &taskBuffer, pdMS_TO_TICKS(100)) == pdPASS)
    {
      if (dataFile)
      {
        dataFile.print(taskBuffer);
        // Flush every 2 seconds and only once nothing is left queued. also every 10s regardless cus overdue tasks bad
        bool quiet = (uxQueueMessagesWaiting(sdQueue) == 0 && millis() - lastFlush >= 2000);
        bool overdue = (millis() - lastFlush >= 10000); 
        if (quiet || overdue)
        {
          dataFile.flush();
          lastFlush = millis();
        }
      }
    }
    else if (closeRequested && dataFile)
    {
      dataFile.flush();
      dataFile.close();
      closeRequested = false;
      Serial.println("log file closed");
    }
    // Briefly yield to prevent Watchdog Timer crashes
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// end of code
// god speed ICARUS, Go Touch The Sun.






