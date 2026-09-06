#include <SPI.h>
#include <SD.h>

#define SEALEVELPRESSURE_PA (101325.0)

const int chipSelect = D3;
const int SD_SCK  = D8;
const int SD_MISO = D2;
const int SD_MOSI = D10;

File dataFile;

QueueHandle_t sdQueue;
TaskHandle_t sdTaskHandle;
#define LOG_BUF_LEN   256
#define LOG_QUEUE_LEN 100
char logBuffer[LOG_BUF_LEN];
bool loggingEnabled = true;
bool sdCardOk = false;
bool fileOpened = false;
const char *LOG_PATH = "/sim_flight.csv";

unsigned long previousMicros = 0;
const unsigned long INTERVAL_US = 20000UL;
unsigned long loopDtUs = 0;

enum FlightState
{
  PAD_IDLE,
  ASCENT,
  DESCENT,
  LANDED
};
FlightState currentState = PAD_IDLE;

float groundAltitude = 0.0;
float currentAltitude = 0.0;
float previousAltitude = 0.0;
unsigned long previousTime = 0;

float currentvelocity = 0.0;
float barovelocity = 0.0;
float distancetravelled = 0.0;
float predictedVel = 0.0;
float baroAccel = 0.0;
float alpha = 0.4;
float beta = 0.05;
float fusedVelocity = 0.0;
const float BARO_TRUST = 0.35;

const float RHO_MIN = 0.6;
const float RHO_MAX = 1.4;
const float VEL_MIN = -150.0;
const float VEL_MAX = 200.0;

int fallingCount = 0;
float peakAltitude = 0.0;

unsigned long stillSince = 0;
volatile bool closeRequested = false;
unsigned int droppedLogs = 0;
unsigned int missedBaro = 0;

float landingRefAlt = 0.0;
const float LANDING_ALT_MAX  = 25.0;
const float LANDING_ALT_BAND = 3.0;
const float LANDING_ACC_BAND = 1.5;

unsigned long launchTime = 0;
float padMass = 0.7293;
float currentMass = 0.789;
float dryMass = 0.7293;
float burnTime = 1.4;

int currentServoAngle = 126;
const int SERVO_MIN = 0;
const int SERVO_MAX = 126;
const int SERVO_STEP = 3;
const int SERVO_DEADBAND = 5;

float area = 0.0019635;
float rho = 1.225;
float CDClean = 0.4724;
float CDMax   = 0.8776;
float CD = CDClean;
int apogeeTgt = 530;
const float overshootPenalty = 1.25;
const float MPC_ARM_MARGIN_S = 0.1;

const unsigned long AIRBRAKE_DESCENT_DELAY_MS = 3000UL;
unsigned long apogeeTime = 0;
bool brakesDeployed = false;

#define MPC_CANDIDATES 41
float bestApogee = 0.0;

float accelScale = 1.0f;

const float SIM_PAD_ALT   = 103.0f;
const float SIM_PAD_HOLD  = 3.0f;
const float SIM_BURN      = 1.4f;
const float SIM_WET       = 0.789f;
const float SIM_DRY       = 0.7293f;
const float SIM_AREA      = 0.0019635f;
const float SIM_CHUTE_CDA = 0.060f;
const float SIM_CHUTE_DELAY = 1.0f;
const float SIM_BARO_NOISE = 3.0f;
const float SIM_ACC_NOISE  = 0.12f;
const float SIM_TIMEOUT_S  = 240.0f;
const int   SIM_BARO_MISS_ODDS = 500;

float simTime = 0.0f;
float simAgl = 0.0f;
float simVel = 0.0f;
float simAccelVert = 0.0f;
float simPeakAgl = 0.0f;
float simApogeeTime = 0.0f;
bool  simLiftoff = false;
bool  simApogee = false;
bool  simChuteOut = false;
bool  simTouchdown = false;
bool  simFinished = false;

float simPressure = 101325.0f;
float simTemperature = 15.0f;
float simAx = 0.0f;
float simAy = 0.0f;
float simAz = 9.81f;
float simGx = 0.0f;
float simGy = 0.0f;
float simGz = 0.0f;
float simMx = 22.0f;
float simMy = -3.0f;
float simMz = 41.0f;
bool  simBaroOk = true;

float maxFusedVel = 0.0f;
int   minServoSeen = 126;
float simBurnoutVel = 0.0f;
float simBurnoutAlt = 0.0f;
unsigned long simStartMillis = 0;

void logToSDTask(void *pvParameters);

bool sdInit()
{
  pinMode(chipSelect, OUTPUT);
  digitalWrite(chipSelect, HIGH);
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, chipSelect);
  delay(50);

  const uint32_t speeds[3] = {400000UL, 1000000UL, 4000000UL};
  for (int attempt = 0; attempt < 9; attempt++)
  {
    uint32_t sp = speeds[attempt % 3];
    Serial.print("SD init attempt ");
    Serial.print(attempt + 1);
    Serial.print(" at ");
    Serial.print(sp);
    Serial.print(" Hz ... ");
    if (SD.begin(chipSelect, SPI, sp))
    {
      Serial.println("OK");
      uint8_t ct = SD.cardType();
      Serial.print("card type: ");
      if (ct == CARD_NONE) Serial.println("NONE");
      else if (ct == CARD_MMC) Serial.println("MMC");
      else if (ct == CARD_SD) Serial.println("SDSC");
      else if (ct == CARD_SDHC) Serial.println("SDHC");
      else Serial.println("UNKNOWN");
      Serial.print("card size (MB): ");
      Serial.println((unsigned long)(SD.cardSize() / (1024ULL * 1024ULL)));
      return (ct != CARD_NONE);
    }
    Serial.println("fail");
    SD.end();
    delay(300);
  }
  return false;
}

float pressurePa()
{
  return simPressure;
}

float altitudeFromPa()
{
  return 44330.0f * (1.0f - powf(pressurePa() / (float)SEALEVELPRESSURE_PA, 0.1903f));
}

float airDensity()
{
  return pressurePa() / (287.05f * (simTemperature + 273.15f));
}

float predictApogee(float cdSim, float alt0, float v0, float mass)
{
  if (v0 <= 0.0f) return alt0;
  float k = (rho * cdSim * area) / (2.0f * mass);
  if (k < 1e-9f) return alt0;
  float h = (1.0f / (2.0f * k)) * logf(1.0f + (k * v0 * v0) / 9.81f);
  float rhoMid = rho * (1.0f - (0.5f * h) / 8500.0f);
  if (rhoMid < RHO_MIN) rhoMid = RHO_MIN;
  k = (rhoMid * cdSim * area) / (2.0f * mass);
  if (k < 1e-9f) return alt0;
  h = (1.0f / (2.0f * k)) * logf(1.0f + (k * v0 * v0) / 9.81f);
  return alt0 + h;
}

float simNoise(float amp)
{
  return ((float)random(-1000, 1001) / 1000.0f) * amp;
}

float simThrust(float t)
{
  static const float tt[7] = {0.00f, 0.05f, 0.20f, 0.60f, 1.00f, 1.30f, 1.40f};
  static const float ff[7] = {0.00f, 95.0f, 86.0f, 80.0f, 74.0f, 60.0f, 0.00f};
  if (t <= 0.0f || t >= SIM_BURN) return 0.0f;
  for (int i = 0; i < 6; i++)
  {
    if (t >= tt[i] && t <= tt[i + 1])
    {
      float f = (t - tt[i]) / (tt[i + 1] - tt[i]);
      return ff[i] + f * (ff[i + 1] - ff[i]);
    }
  }
  return 0.0f;
}

float simCdFromServo()
{
  float f = (float)(SERVO_MAX - currentServoAngle) / (float)(SERVO_MAX - SERVO_MIN);
  if (f < 0.0f) f = 0.0f;
  if (f > 1.0f) f = 1.0f;
  return CDClean + f * (CDMax - CDClean);
}

float simPressureFromAlt(float altM)
{
  float ratio = 1.0f - (altM / 44330.0f);
  if (ratio < 0.0001f) ratio = 0.0001f;
  return (float)SEALEVELPRESSURE_PA * powf(ratio, 1.0f / 0.1903f);
}

void simSensors()
{
  float trueAlt = SIM_PAD_ALT + simAgl;
  simPressure = simPressureFromAlt(trueAlt) + simNoise(SIM_BARO_NOISE);
  simTemperature = 15.0f - 0.0065f * trueAlt + simNoise(0.05f);

  simAz = simAccelVert + 9.81f + simNoise(SIM_ACC_NOISE);
  simAx = simNoise(SIM_ACC_NOISE);
  simAy = simNoise(SIM_ACC_NOISE);

  if (simLiftoff && !simTouchdown)
  {
    simAx += simNoise(0.6f);
    simAy += simNoise(0.6f);
    simGx = simNoise(0.35f);
    simGy = simNoise(0.35f);
    simGz = simChuteOut ? simNoise(2.5f) : (2.2f + simNoise(0.4f));
  }
  else
  {
    simGx = simNoise(0.01f);
    simGy = simNoise(0.01f);
    simGz = simNoise(0.01f);
  }

  simMx = 22.0f + simNoise(0.8f);
  simMy = -3.0f + simNoise(0.8f);
  simMz = 41.0f + simNoise(0.8f);

  simBaroOk = (random(0, SIM_BARO_MISS_ODDS) != 0);
}

void simStep(float dt)
{
  if (simFinished) { simSensors(); return; }

  simTime += dt;
  float t = simTime - SIM_PAD_HOLD;

  float m;
  if (t <= 0.0f) m = SIM_WET;
  else if (t < SIM_BURN) m = SIM_WET - (SIM_WET - SIM_DRY) * (t / SIM_BURN);
  else m = SIM_DRY;

  float thrust = simThrust(t);

  if (!simLiftoff && thrust > (m * 9.81f)) simLiftoff = true;

  float cdaNow;
  if (simChuteOut) cdaNow = SIM_CHUTE_CDA + simCdFromServo() * SIM_AREA;
  else cdaNow = simCdFromServo() * SIM_AREA;

  float rhoSim = 1.225f * expf(-(SIM_PAD_ALT + simAgl) / 8500.0f);
  float drag = 0.5f * rhoSim * cdaNow * simVel * fabsf(simVel);

  float a = ((thrust - drag) / m) - 9.81f;

  if (!simLiftoff)
  {
    a = 0.0f;
    simVel = 0.0f;
    simAgl = 0.0f;
  }
  else if (simTouchdown)
  {
    a = 0.0f;
    simVel = 0.0f;
    simAgl = 0.0f;
  }
  else
  {
    simVel += a * dt;
    simAgl += simVel * dt;
  }

  if (simLiftoff && !simTouchdown)
  {
    if (simAgl > simPeakAgl) simPeakAgl = simAgl;

    if (!simApogee && simVel < 0.0f && t > SIM_BURN)
    {
      simApogee = true;
      simApogeeTime = simTime;
    }
    if (simApogee && !simChuteOut && (simTime - simApogeeTime) >= SIM_CHUTE_DELAY)
    {
      simChuteOut = true;
    }
    if (simAgl <= 0.0f && t > SIM_BURN)
    {
      simAgl = 0.0f;
      simVel = 0.0f;
      a = 0.0f;
      simTouchdown = true;
    }
  }

  if (simLiftoff && simBurnoutVel == 0.0f && t >= SIM_BURN)
  {
    simBurnoutVel = simVel;
    simBurnoutAlt = simAgl;
  }

  simAccelVert = a;
  simSensors();
}

void printSummary()
{
  Serial.println();
  Serial.println("========== SIM FLIGHT SUMMARY ==========");
  Serial.print("sim run time (s): ");        Serial.println(simTime, 2);
  Serial.print("burnout velocity (m/s): ");  Serial.println(simBurnoutVel, 2);
  Serial.print("burnout altitude (m): ");    Serial.println(simBurnoutAlt, 2);
  Serial.print("true apogee (m AGL): ");     Serial.println(simPeakAgl, 2);
  Serial.print("baro peak seen (m AGL): ");  Serial.println(peakAltitude, 2);
  Serial.print("apogee target (m): ");       Serial.println(apogeeTgt);
  Serial.print("apogee error (m): ");        Serial.println(simPeakAgl - (float)apogeeTgt, 2);
  Serial.print("max fused velocity (m/s): ");Serial.println(maxFusedVel, 2);
  Serial.print("max airbrake deploy (deg): ");Serial.println(SERVO_MAX - minServoSeen);
  Serial.print("final CD command: ");        Serial.println(CD, 4);
  Serial.print("last predicted apogee (m): ");Serial.println(bestApogee, 2);
  Serial.print("ground altitude (m ASL): "); Serial.println(groundAltitude, 2);
  Serial.print("dropped log lines: ");       Serial.println(droppedLogs);
  Serial.print("missed baro reads: ");       Serial.println(missedBaro);
  Serial.println("========================================");
  if (fileOpened)
  {
    Serial.print("data written to ");
    Serial.println(LOG_PATH);
    Serial.print("file size on card (bytes): ");
    File chk = SD.open(LOG_PATH, FILE_READ);
    if (chk) { Serial.println((unsigned long)chk.size()); chk.close(); }
    else Serial.println("could not reopen file");
  }
  else
  {
    Serial.println("NO FILE WRITTEN - SD was unavailable, CSV above is serial only");
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);
  delay(2500);
  Serial.println();
  Serial.println("XIAO SIM TEST - FAKE FLIGHT HARNESS");
  Serial.println("no sensors required, SD logging active");

  randomSeed(micros() ^ 0x5A17F00D);

  sdQueue = xQueueCreate(LOG_QUEUE_LEN, sizeof(logBuffer));
  if (sdQueue == NULL)
  {
    Serial.println("QUEUE ALLOCATION FAILED - SD logging disabled");
    loggingEnabled = false;
  }
  xTaskCreate(
    logToSDTask,
    "SD_Task",
    4096,
    NULL,
    1,
    &sdTaskHandle
  );

  Serial.print("SD pins  SCK:"); Serial.print(SD_SCK);
  Serial.print("  MISO:");          Serial.print(SD_MISO);
  Serial.print("  MOSI:");          Serial.print(SD_MOSI);
  Serial.print("  CS:");            Serial.println(chipSelect);
  delay(1500);

  sdCardOk = sdInit();
  if (!sdCardOk)
  {
    Serial.println("SD CARD NOT DETECTED - CSV WILL STREAM OVER SERIAL ONLY");
    Serial.println("check: card inserted, FAT32, wiring, 3V3 rail");
    loggingEnabled = false;
  }

  simSensors();

  Serial.println("Calibrating Ground Altitude");
  float altSum = 0.0f;
  int   altN = 0;
  for (int i = 0; i < 20; i++)
  {
    simSensors();
    if (simBaroOk) { altSum += altitudeFromPa(); altN++; }
    delay(50);
  }
  if (altN > 0) groundAltitude = altSum / (float)altN;
  else
  {
    groundAltitude = altitudeFromPa();
    Serial.println("WARNING: no good baro reads during ground calibration - using last value");
  }
  Serial.print("ground alt samples used: "); Serial.println(altN);

  previousAltitude = groundAltitude;
  previousTime = millis();

  Serial.print("BMP pressure (Pa): "); Serial.println(pressurePa());
  Serial.print("pad air density: ");   Serial.println(airDensity());
  Serial.print("apogee target: ");     Serial.println(apogeeTgt);

  if (sdCardOk)
  {
    SD.remove(LOG_PATH);
    dataFile = SD.open(LOG_PATH, FILE_WRITE);
    if (dataFile)
    {
      dataFile.println("Time(ms),State,Alt,Vel,ServoAngle,Temp_C,Press_Pa,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,MagX,MagY,MagZ,CD,PredApogee,TrueAlt,TrueVel");
      dataFile.flush();
      fileOpened = true;
      loggingEnabled = true;
      Serial.print("log file open: "); Serial.println(LOG_PATH);
    }
    else
    {
      Serial.println("SD PRESENT BUT FILE WOULD NOT OPEN - check format is FAT32");
      loggingEnabled = false;
    }
  }

  Serial.println("Time(ms),State,Alt,Vel,ServoAngle,Temp_C,Press_Pa,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,MagX,MagY,MagZ,CD,PredApogee,TrueAlt,TrueVel");
  Serial.println("SIM ARMED - IGNITION IN 3s OF SIM TIME");

  delay(1000);
  simStartMillis = millis();
  previousMicros = micros();
  previousTime   = millis();
}

void loop()
{
  unsigned long currentMicros = micros();
  if (currentMicros - previousMicros >= INTERVAL_US)
  {
    loopDtUs = currentMicros - previousMicros;
    float dt = (float)loopDtUs * 1e-6f;
    previousMicros = currentMicros;
    unsigned long currentMillis = millis();

    if (simFinished)
    {
      vTaskDelay(pdMS_TO_TICKS(100));
      return;
    }

    simStep(dt);

    bool baroOk = true;
    if (!simBaroOk)
    {
      baroOk = false;
      missedBaro++;
    }
    else
    {
      currentAltitude = altitudeFromPa();
    }

    float ax = simAx * accelScale;
    float ay = simAy * accelScale;
    float az = simAz * accelScale;

    float agl = currentAltitude - groundAltitude;

    if (currentState == PAD_IDLE)
    {
      float accelMag = sqrtf(sq(ax) + sq(ay) + sq(az));
      float netAccel = accelMag - 9.81f;

      if (netAccel > 2.0f)
      {
        currentvelocity += netAccel * dt;
      }
      else
      {
        if (currentvelocity < 1.0f)
        {
          currentvelocity = 0.0f;
          distancetravelled = 0.0f;
        }
      }

      distancetravelled += currentvelocity * dt;

      if (distancetravelled >= 1.0f || agl >= 50.0f)
      {
        currentState = ASCENT;
        launchTime = (millis() > 150UL) ? (millis() - 150UL) : millis();
        fusedVelocity = currentvelocity;
        barovelocity = 0.0f;
        baroAccel = 0.0f;
        peakAltitude = 0.0f;
        fallingCount = 0;
        Serial.println(" Nominal Launch , Sensors are Logging ");
      }
    }

    if (currentState != PAD_IDLE)
    {
      float aVert = az - 9.81f;
      fusedVelocity += aVert * dt;
      fusedVelocity = constrain(fusedVelocity, VEL_MIN, VEL_MAX);
    }

    float timeSinceLastVel = (currentMillis - previousTime) / 1000.0f;
    if (baroOk && timeSinceLastVel >= 0.1f)
    {
      float rawVel = (currentAltitude - previousAltitude) / timeSinceLastVel;

      predictedVel = barovelocity + (baroAccel * timeSinceLastVel);

      float residual = rawVel - predictedVel;

      barovelocity = predictedVel + (alpha * residual);
      baroAccel = baroAccel + (beta * residual / timeSinceLastVel);

      float trust = BARO_TRUST;
      if (currentState == ASCENT && (currentMillis - launchTime) < (unsigned long)(burnTime * 1000.0f)) trust = 0.0f;
      fusedVelocity += trust * (barovelocity - fusedVelocity);

      previousAltitude = currentAltitude;
      previousTime = currentMillis;
    }

    if (fusedVelocity > maxFusedVel) maxFusedVel = fusedVelocity;

    if (currentState == ASCENT)
    {
      float elapsed = (currentMillis - launchTime) / 1000.0f;
      if (elapsed < burnTime)
      {
        currentMass = padMass - ((padMass - dryMass) * (elapsed / burnTime));
      }
      else
      {
        currentMass = dryMass;
      }

      if (agl > peakAltitude)
      {
        peakAltitude = agl;
        fallingCount = 0;
      }
      else if (agl < peakAltitude - 1.0f)
      {
        fallingCount++;
      }

      if (fusedVelocity < -2.0f || fallingCount >= 5)
      {
        currentState = DESCENT;
        currentServoAngle = SERVO_MAX;
        apogeeTime = currentMillis;
        brakesDeployed = false;

        landingRefAlt = agl;
        stillSince = 0;
        Serial.println("APOGEE REACHED - DESCENT STATE ENGAGED");
      }
      else
      {
        if (elapsed >= burnTime + MPC_ARM_MARGIN_S)
        {
          float rhoNow = airDensity();
          rho = constrain(rhoNow, RHO_MIN, RHO_MAX);

          float bestCost = 1000000.0f;
          float bestCD = CDClean;
          bestApogee = agl;

          for (int c = 0; c < MPC_CANDIDATES; c++)
          {
            float CDSim = CDClean + ((float)c * ((CDMax - CDClean) / (float)(MPC_CANDIDATES - 1)));
            float altSim = predictApogee(CDSim, agl, fusedVelocity, currentMass);

            float err = altSim - (float)apogeeTgt;
            float cost = (err > 0.0f) ? (err * overshootPenalty) : (-err);

            if (cost < bestCost)
            {
              bestCost = cost;
              bestCD = CDSim;
              bestApogee = altSim;
            }
          }
          CD = bestCD;

          int targetAngle = map((long)(CD * 10000), (long)(CDClean * 10000), (long)(CDMax * 10000), SERVO_MAX, SERVO_MIN);
          targetAngle = constrain(targetAngle, SERVO_MIN, SERVO_MAX);

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
          }
        }
        else
        {
          currentServoAngle = SERVO_MAX;
        }
      }
    }

    if (currentServoAngle < minServoSeen) minServoSeen = currentServoAngle;

    if (currentState == DESCENT)
    {
      if (!brakesDeployed && (currentMillis - apogeeTime) >= AIRBRAKE_DESCENT_DELAY_MS)
      {
        brakesDeployed = true;
        currentServoAngle = SERVO_MIN;
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
          Serial.println("LANDED - closing file");
          simFinished = true;
          printSummary();
        }
      }
      else
      {
        landingRefAlt = agl;
        stillSince = 0;
      }
    }

    if (!simFinished && simTime > SIM_TIMEOUT_S)
    {
      currentState = LANDED;
      closeRequested = true;
      simFinished = true;
      Serial.println("SIM TIMEOUT - ENDING RUN");
      printSummary();
    }

    if (currentState == ASCENT || currentState == DESCENT)
    {
      float pressureLog = pressurePa();
      snprintf(logBuffer, sizeof(logBuffer), "%lu,%d,%.2f,%.2f,%d,%.2f,%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.4f,%.1f,%.2f,%.2f\n",
        currentMillis, currentState, agl, fusedVelocity, currentServoAngle,
        simTemperature, pressureLog,
        simAx, simAy, simAz,
        simGx, simGy, simGz,
        simMx, simMy, simMz,
        CD, bestApogee, simAgl, simVel);
      if (loggingEnabled && sdQueue != NULL)
      {
        if (xQueueSend(sdQueue, &logBuffer, 0) != pdPASS) droppedLogs++;
      }
      Serial.print(logBuffer);
    }
  }
}

void logToSDTask(void *pvParameters)
{
  char taskBuffer[LOG_BUF_LEN];
  unsigned long lastFlush = 0;
  if (sdQueue == NULL) { while (true) { vTaskDelay(pdMS_TO_TICKS(1000)); } }
  while (true)
  {
    if (xQueueReceive(sdQueue, &taskBuffer, pdMS_TO_TICKS(100)) == pdPASS)
    {
      if (dataFile)
      {
        dataFile.print(taskBuffer);
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
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}
