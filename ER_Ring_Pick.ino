/*
  rotation levels
                |
                |
                |
                |
                |
                |
                level 2 (rotateLs2)(resetLevel)
  ------- level 1 (rotateLs1)


  Platform Levels

  -------------- level 2 (platformLs2)

    ---------   (Sub level for each ring in total 10 rings)
    ---------
    ---------

  -------------- level 1 (platformLs1)(resetLevel)
*/
#include <EspNow.h>
#include <Motor.h>
#include <positionalnew.h>

Peer remote;
JSONVar myData;
Motor rotationMotor(22, 23);
Motor platformMotor(33, 32);
UniversalEncoder rotationEncoder(19, 18, 1);
UniversalEncoder platformEncoder(25, 26, -1);
positionalnew rMPID(&rotationMotor);
positionalnew pMPID(&platformMotor);

double AggKp1 = 1.0, AggKi1 = 0.0, Aggkd1 = 0;
double SoftKp1 = 0.3, SoftKi1 = 0, Softkd1 = 0;
double AggKp2 = 1.2, AggKi2 = 0.0, Aggkd2 = 0;
double SoftKp2 = 0.6, SoftKi2 = 0, Softkd2 = 0;

int rotateLs1 = 15, rotateLs2 = 16, platformLs1 = 14, platformLs2 = 17;

int rotateLevel = 0, platformLevel = 0, platformSubLevel = 0;
int rInternalLvl = -1, pInternalLvl = -1;
double rotationPulseOffset = 0.05, platformPulseOffset = 0.05, signOffsetRotation = 0, signOffsetPlatform = 0;

long rotatePulse = 0, platformPulse = 0, resetPulse = 50000;
long rLvl2Pulse = 0, pLvl1Pulse = 0, subLevel1 = 0, oneRingPulse = 0, rotationExtraPulse = 0, platformExtraPulse = 0;

bool init_ = false;
bool rLs1 = false, rLs2 = false, pLs1 = false, pLs2 = false;
bool allRings = true;
void setup()
{
  Serial.begin(115200);
  //  delay(5000);
  rotationMotor.setEncoder(&rotationEncoder);
  platformMotor.setEncoder(&platformEncoder);

  rMPID.setThreshold(100);
  rMPID.setOutputLimits(-30, 30);
  rMPID.setAggTunings(AggKp1, AggKi1, Aggkd1);
  rMPID.setSoftTunings(SoftKp1, SoftKi1, Softkd1);

  pMPID.setThreshold(50);
  pMPID.setOutputLimits(-40, 40);
  pMPID.setAggTunings(AggKp2, AggKi2, Aggkd2);
  pMPID.setSoftTunings(SoftKp2, SoftKi2, Softkd2);

  pinMode(rotateLs1, INPUT_PULLUP);
  pinMode(rotateLs2, INPUT_PULLUP);
  pinMode(platformLs1, INPUT_PULLUP);
  pinMode(platformLs2, INPUT_PULLUP);

  setId("PICKE");
  remote.init("TenZZ");
  remote.setOnRecieve(rotationLvl1, "rLvl1");
  remote.setOnRecieve(rotationLvl2, "rLvl2");
  remote.setOnRecieve(platformLvl1, "pLvl1");
  remote.setOnRecieve(platformSubLvl2, "pLvl2");
  remote.setOnRecieve(setRotateExtraPulse, "exPl");
  remote.setOnRecieve(setPlatformExtraPulse, "exRo");
  remote.setOnRecieve(resetAll, "Erst");
}
void loop()
{
  rotatePulse = rotationMotor.getReadings();
  platformPulse = platformMotor.getReadings();
  pLs1 = !(bool)digitalRead(platformLs1);
  pLs2 = !(bool)digitalRead(platformLs2);
  rLs1 = !(bool)digitalRead(rotateLs1);
  rLs2 = !(bool)digitalRead(rotateLs2);
  //  Serial.println(rotatePulse);
  if (Serial.available() > 0)
  {
    rotateLevel = Serial.readStringUntil(',').toInt();
    platformLevel = Serial.readStringUntil('\n').toInt();
  }

  if (!init_)
  {
    if ((rLs1 || rLs2) && (rInternalLvl == -1 || rInternalLvl == 1 || rInternalLvl == 2 || rInternalLvl == -2 || rInternalLvl == -3)) // give range afterwards for rInternal
    {

      if (rInternalLvl >= -1) // stops motor
      {
        //Serial.println("Stop Motor");
        // rotationMotor.stop();
        rMPID.setPulse(rotationMotor.getReadings()); // can use rotatePulse // 0 issue
      }
      else if (rInternalLvl == -2 && rLs2) // reached lvl2
      {
        //Serial.println("Stop at level 2");
        rMPID.setPulse(rotationMotor.getReadings());
        rInternalLvl = 1;
      }
      else if (rInternalLvl == -3 && rLs1) // reached lvl1
      {
        //Serial.println("Stop at level 1");
        rMPID.setPulse(rotationMotor.getReadings());
        rInternalLvl = 2;
      }

      if (rInternalLvl == -2 || rInternalLvl == -3) // to remove uneccesary comaprison
      {
        //        Serial.println("doNothing");
      }
      else if (rInternalLvl == -1) // detected that a limit switch is pressed
      {
        //Serial.println("Limit Switched Pressed");
        rInternalLvl = 0;
      }
      else if (rInternalLvl == 1 && rLs2) // meaning rotated to lvl2
      {
        //Serial.println("Both Switch Pressed and finally reached level 2");
        rLvl2Pulse = rotationMotor.getReadings();
        if (rLvl2Pulse < 0)
        {
          signOffsetRotation = -1;
        }
        else
        {
          signOffsetRotation = 1;
        }
        rotationExtraPulse = rLvl2Pulse * rotationPulseOffset;
        rInternalLvl = 3;
        Serial.println("rsignOff: " +String(signOffsetRotation)+" rLvl2: " +String(rLvl2Pulse)+" rExtra: " +String(rotationExtraPulse));
        rMPID.setPulse(rLvl2Pulse);
      }
      else if (rInternalLvl == 2 && rLs1) // meaning rotated to lvl1
      {
        //Serial.println("Both Switch Pressed and finally reached level 1");
        rLvl2Pulse = rotationMotor.getReadings();
        if (rLvl2Pulse < 0)
        {
          signOffsetRotation = -1;
        }
        else
        {
          signOffsetRotation = 1;
        }
        rotationExtraPulse = rLvl2Pulse * rotationPulseOffset;
        Serial.println("rsignOff: " +String(signOffsetRotation)+" rLvl2: " +String(rLvl2Pulse)+" rExtra: " +String(rotationExtraPulse));
        rotationMotor.reset();
        rInternalLvl = 3;
        rMPID.setPulse(-rLvl2Pulse);
        //Serial.println("Goto level 2 after both switch press");
      }
    }

    if (!rLs1 && !rLs2 && rInternalLvl == -1) // no ls is pressed
    {
      //Serial.println("noLimit pressed");
      rMPID.setPulse(resetPulse);
    }
    else if (rLs1 && rInternalLvl == 0) // if first ls1 is pressed
    {
      //Serial.println("First LS-1 pressed");
      rotationMotor.reset();
      rInternalLvl = 1;
    }
    else if (rLs2 && rInternalLvl == 0) // if first ls2 is pressed
    {
      //Serial.println("First LS-2 pressed");
      rotationMotor.reset();
      rInternalLvl = 2;
    }
    else if (!rLs2 && rInternalLvl == 1) // going to level2
    {
      //Serial.println("Goto level 2");
      rMPID.setPulse(-resetPulse);
      rInternalLvl = -2;
      // if(rotationMotor.getReadings()<0)
      // {
      //   signOffsetRotation=-1;
      // }
      // else
      // {
      //   signOffsetRotation=1;
      // }
    }
    else if (!rLs1 && rInternalLvl == 2) // going to level1
    {
      //Serial.println("Goto level 1");
      rMPID.setPulse(resetPulse);
      rInternalLvl = -3;
    }
    rMPID.compute();
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if (rInternalLvl == 3)
    {
      if ((pLs1 || pLs2) && (pInternalLvl == -1 || pInternalLvl == 1 || pInternalLvl == 2 || pInternalLvl == -2 || pInternalLvl == -3)) // give range afterwards for pInternal
      {

        if (pInternalLvl >= -1) // stops motor
        {
          //Serial.println("Stop Motor");
          // platformMotor.stop();
          pMPID.setPulse(platformMotor.getReadings()); // can use rotatePulse // 0 issue
        }
        else if (pInternalLvl == -2 && pLs2) // reached lvl2
        {
          //Serial.println("Stop at level 2");
          pMPID.setPulse(platformMotor.getReadings());
          pInternalLvl = 1;
        }
        else if (pInternalLvl == -3 && pLs1) // reached lvl1
        {
          //Serial.println("Stop at level 1");
          pMPID.setPulse(platformMotor.getReadings());
          pInternalLvl = 2;
        }

        if (pInternalLvl == -2 || pInternalLvl == -3) // to remove uneccesary comaparison
        {
          //        Serial.println("doNothing");
        }
        else if (pInternalLvl == -1) // detected that a limit switch is pressed
        {
          //Serial.println("Limit Switched Pressed");
          pInternalLvl = 0;
        }
        else if (pInternalLvl == 2 && pLs1) // meaning rotated to lvl1
        {
          //Serial.println("Both Switch Pressed and finally reached level 2");
          pLvl1Pulse = platformMotor.getReadings();
          subLevel1 = pLvl1Pulse * 0.2;
          platformExtraPulse = (pLvl1Pulse - subLevel1) * 0.05;
          pInternalLvl = 3;
          pMPID.setPulse(pLvl1Pulse);
          if (pLvl1Pulse < 0)
          {
            signOffsetPlatform = -1;
          }
          else
          {
            signOffsetPlatform = 1;
          }
          Serial.println("psignOff: " +String(signOffsetPlatform)+" pLvl1: " +String(pLvl1Pulse)+" Sublvl1: " +String(subLevel1)+" pExtra: " +String(platformExtraPulse)+" oneRing: " +String(oneRingPulse));
        }
        else if (pInternalLvl == 1 && pLs2) // meaning rotated to lvl2
        {
          //Serial.println("Both Switch Pressed and finally reached level 1");
          pLvl1Pulse = platformMotor.getReadings();
          if (pLvl1Pulse < 0)
          {
            signOffsetPlatform = -1;
          }
          else
          {
            signOffsetPlatform = 1;
          }
          subLevel1 = pLvl1Pulse * 0.2;
          platformExtraPulse = (pLvl1Pulse - subLevel1) * 0.05;
          oneRingPulse = (pLvl1Pulse - subLevel1) * 0.1;
          platformMotor.reset();
          pInternalLvl = 3;
          Serial.println("psignOff: " +String(signOffsetPlatform)+" pLvl1: " +String(pLvl1Pulse)+" Sublvl1: " +String(subLevel1)+" pExtra: " +String(platformExtraPulse)+" oneRing: " +String(oneRingPulse));
          pMPID.setPulse(-pLvl1Pulse);
          //Serial.println("Goto level 1 after both switch press");
        }
      }

      if (!pLs1 && !pLs2 && pInternalLvl == -1) // no ls is pressed
      {
        //Serial.println("Platform noLimit pressed");
        pMPID.setPulse(resetPulse);
      }
      else if (pLs1 && pInternalLvl == 0) // if first ls1 is pressed
      {
        //Serial.println("First LS-1 pressed");
        platformMotor.reset();
        pInternalLvl = 1;
      }
      else if (pLs2 && pInternalLvl == 0) // if first ls2 is pressed
      {
        //Serial.println("First LS-2 pressed");
        platformMotor.reset();
        pInternalLvl = 2;
      }
      else if (!pLs2 && pInternalLvl == 1) // going to level2
      {
        //Serial.println("Goto level 2");
        pMPID.setPulse(-resetPulse);
        pInternalLvl = -2;
      }
      else if (!pLs1 && pInternalLvl == 2) // going to level1
      {
        //Serial.println("Goto level 1");
        pMPID.setPulse(-resetPulse);
        pInternalLvl = -3;
        // if(platformMotor.getReadings()<0)
        // {
        //   signOffset=-1
        // }
        // else
        // {
        //   signOffset=1;
        // }
      }
      pMPID.compute();
    }
  }
  else if (init_)
  {
    if (rLs1)
    {
      rMPID.setPulse(rotationMotor.getReadings() + signOffsetRotation * rLvl2Pulse * 0.03);
    }
    else if (rLs2)
    {
      rMPID.setPulse(rotationMotor.getReadings() + signOffsetRotation * -1 * rLvl2Pulse * 0.03);
    }
    rMPID.compute();
    pMPID.compute();
  }
}

void rotationLvl1(JSONVar msg)
{
  Serial.println("rotationLvl1");
  rotateLevel = 1;
  init_ = true;
  rMPID.setPulse(0);
}

void rotationLvl2(JSONVar msg)
{
  Serial.println("rotationLvl2");
  rotateLevel = 2;
  init_ = true;
  rMPID.setPulse(rLvl2Pulse);
}

void platformLvl1(JSONVar msg)
{
  Serial.println("platformLvl1");
  platformLevel = 1;
  init_ = true;
  pMPID.setPulse(pLvl1Pulse);
}

void platformSubLvl2(JSONVar msg)
{
  platformLevel = 2;
  platformSubLevel++;
  platformSubLevel = platformSubLevel > 10 ? 10 : platformSubLevel;
  Serial.println("platformSubLvl2");
  init_ = true;
  if (allRings)
  {
    pMPID.setPulse(subLevel1);
    allRings = false;
  }
  else if (!allRings)
  {
    pMPID.setPulse(subLevel1 + platformSubLevel*oneRingPulse);
  }
}

void setRotateExtraPulse(JSONVar msg)
{
  int extraOffset = (int)msg["offset"];
  Serial.println("setRotateExtraPulse");
  rMPID.setPulse(rotationMotor.getReadings() + (signOffsetRotation * extraOffset * rotationExtraPulse));
}

void setPlatformExtraPulse(JSONVar msg)
{
  int extraOffset = (int)msg["offset"];
  Serial.println("setPlatformExtraPulse");
  rMPID.setPulse(platformMotor.getReadings() + (signOffsetPlatform * extraOffset * platformExtraPulse));
}

void resetAll(JSONVar msg)
{
  Serial.println("resetAll");
  init_ = false;
  allRings = true;

  rInternalLvl = -1;
  rotateLevel = 0;

  pInternalLvl = -1;
  platformLevel = 0;
  platformSubLevel = 0;
}

// add offset for what extra up & down detection
