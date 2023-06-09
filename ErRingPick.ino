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
Peer dataesp;
JSONVar feedback;
JSONVar datapick;
Motor rotationMotor(22, 23);
Motor platformMotor(33, 32);
UniversalEncoder rotationEncoder(19, 18, 1);
UniversalEncoder platformEncoder(25, 26, -1);
positionalnew rMPID(&rotationMotor);
positionalnew pMPID(&platformMotor);

double AggKpRotation = 1.0, AggKiRotation = 0.0, AggKdRotation = 0;
double SoftKpRotation = 0.25, SoftKiRotation = 0.0, SoftKdRotation = 0;
double AggKpPlatform = 1.2, AggKiPlatform = 0.0, AggKdPlatform = 0;
double SoftKpPlatform = 0.4, SoftKiPlatform = 0, SoftKdPlatform = 0;

int rotateLs1 = 15, rotateLs2 = 16, platformLs1 = 14, platformLs2 = 17;
//int pneumaticPin = 13;

int rotateLevel = 0, platformLevel = 0, platformSubLevel = 0;
int rInternalLvl = -1, pInternalLvl = -1;
double rotationPulseOffset = 0.05, platformPulseOffset = 0.05, signOffsetRotation = 0, signOffsetPlatform = 0;

long rotatePulse = 0, platformPulse = 0, resetPulse = 50000;
long rLvl2Pulse = 0, pLvl1Pulse = 0, subLevel1 = 0, oneRingPulse = 0, rotationExtraPulse = 0, platformExtraPulse = 0;

bool init_ = false;
bool rLs1 = false, rLs2 = false, pLs1 = false, pLs2 = false;
bool allRings = true;
bool pChange = false;
void setup()
{
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  //  delay(5000);
  rotationMotor.setEncoder(&rotationEncoder);
  platformMotor.setEncoder(&platformEncoder);

  rMPID.setThreshold(100);
  rMPID.setOutputLimits(-30, 30);
  rMPID.setAggTunings(AggKpRotation, AggKiRotation, AggKdRotation);
  rMPID.setSoftTunings(SoftKpRotation, SoftKiRotation, SoftKdRotation);

  pMPID.setThreshold(50);
  pMPID.setOutputLimits(-90, 90);
  pMPID.setAggTunings(AggKpPlatform, AggKiPlatform, AggKdPlatform);
  pMPID.setSoftTunings(SoftKpPlatform, SoftKiPlatform, SoftKdPlatform);

  pinMode(rotateLs1, INPUT_PULLUP);
  pinMode(rotateLs2, INPUT_PULLUP);
  pinMode(platformLs1, INPUT_PULLUP);
  pinMode(platformLs2, INPUT_PULLUP);

  setId("PICKE");
  remote.init("TenZZ");
  dataesp.init("DATAZ");
  remote.setOnRecieve(rotationLvl1, "rLvl1");
  remote.setOnRecieve(rotationLvl2, "rLvl2");
  remote.setOnRecieve(platformLvl1, "pLvl1");
  //remote.setOnRecieve(platformSubLvl2, "pLvl2");
  remote.setOnRecieve(setRotateExtraPulse, "exRo");
  remote.setOnRecieve(setPlatformExtraPulse, "exPl");
  remote.setOnRecieve(resetAll, "Erst");
  remote.setOnRecieve(pneumaticClose, "pClP");
  remote.setOnRecieve(pneumaticOpen, "pOpP");
}
void loop()
{

  rotatePulse = rotationMotor.getReadings();
  platformPulse = platformMotor.getReadings();
  //  Serial.println("ROTATION=" + String(rotatePulse) + ", PLATFORM= " + String(platformPulse));
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
        Serial.println("Stop Motor");
        // rotationMotor.stop();
        rMPID.setPulse(rotationMotor.getReadings()); // can use rotatePulse // 0 issue
      }
      else if (rInternalLvl == -2 && rLs2) // reached lvl2
      {
        // Serial.println("Stop at level 2");
        rMPID.setPulse(rotationMotor.getReadings());
        rInternalLvl = 1;
      }
      else if (rInternalLvl == -3 && rLs1) // reached lvl1
      {
        Serial.println("Stop at level 1");
        rMPID.setPulse(rotationMotor.getReadings());
        rInternalLvl = 2;
      }

      if (rInternalLvl == -2 || rInternalLvl == -3) // to remove uneccesary comaprison
      {
        //        Serial.println("doNothing");
      }
      else if (rInternalLvl == -1) // detected that a limit switch is pressed
      {
        Serial.println("Limit Switched Pressed");
        rInternalLvl = 0;
      }
      else if (rInternalLvl == 1 && rLs2) // meaning rotated to lvl2
      {
        Serial.println("Both Switch Pressed and finally reached level 2");
        rLvl2Pulse = rotationMotor.getReadings();
        if (rLvl2Pulse < 0)
        {
          signOffsetRotation = 1;//-1
        }
        else
        {
          signOffsetRotation = -1;//1
        }
        feedback["type"] = "setROffset";
        feedback["rOffset"] = signOffsetRotation;
        remote.send(feedback);
        rotationExtraPulse = rLvl2Pulse * rotationPulseOffset;
        rInternalLvl = 3;
        //Serial.println("rsignOff: " + String(signOffsetRotation) + " rLvl2: " + String(rLvl2Pulse) + " rExtra: " + String(rotationExtraPulse));
        rMPID.setPulse(rLvl2Pulse);
        datapick["rsignoff"] = signOffsetRotation;
        datapick["rLvl2Pulse"] = rLvl2Pulse;
        datapick["rExtra"] = rotationExtraPulse;
        datapick["type"] = "setupR";
        dataesp.send(datapick);
      }
      else if (rInternalLvl == 2 && rLs1) // meaning rotated to lvl1
      {
        Serial.println("Both Switch Pressed and finally reached level 1");
        rLvl2Pulse = rotationMotor.getReadings();
        rotationExtraPulse = rLvl2Pulse * rotationPulseOffset;
        if (rotationMotor.getReadings() < 0)
        {
          signOffsetRotation = 1;//-1
        }
        else
        {
          signOffsetRotation = -1;//1
        }
        feedback["type"] = "setROffset";
        feedback["rOffset"] = signOffsetRotation;
        remote.send(feedback);
        rotationMotor.reset();
        rInternalLvl = 3;
        rMPID.setPulse(signOffsetRotation * rLvl2Pulse);
        Serial.println("Goto level 2 after both switch press");
        //Serial.println("rsignOff: " + String(signOffsetRotation) + " rLvl2: " + String(rLvl2Pulse) + " rExtra: " + String(rotationExtraPulse));
        datapick["rsignoff"] = signOffsetRotation;
        datapick["rLvl2Pulse"] = rLvl2Pulse;
        datapick["rExtra"] = rotationExtraPulse;
        datapick["type"] = "setupR";
        dataesp.send(datapick);
      }
    }

    if (!rLs1 && !rLs2 && rInternalLvl == -1) // no ls is pressed
    {
      // Serial.println("noLimit pressed");
      rMPID.setPulse(resetPulse);
    }
    else if (rLs1 && rInternalLvl == 0) // if first ls1 is pressed
    {
      Serial.println("First LS-1 pressed");
      rotationMotor.reset();
      rInternalLvl = 1;
    }
    else if (rLs2 && rInternalLvl == 0) // if first ls2 is pressed
    {
      Serial.println("First LS-2 pressed");
      rotationMotor.reset();
      rInternalLvl = 2;
    }
    else if (!rLs2 && rInternalLvl == 1) // going to level2
    {
      Serial.println("Goto level 2");
      rMPID.setPulse(-resetPulse);
      rInternalLvl = -2;
    }
    else if (!rLs1 && rInternalLvl == 2) // going to level1
    {
      Serial.println("Goto level 1");
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
          // Serial.println("Stop Motor");
          pMPID.setPulse(platformMotor.getReadings()); // can use rotatePulse // 0 issue
        }
        else if (pInternalLvl == -2 && pLs2) // reached lvl2
        {
          // Serial.println("Stop at level 2");
          pMPID.setPulse(platformMotor.getReadings());
          pInternalLvl = 1;
        }
        else if (pInternalLvl == -3 && pLs1) // reached lvl1
        {
          // Serial.println("Stop at level 1");
          pMPID.setPulse(platformMotor.getReadings());
          pInternalLvl = 2;
        }

        if (pInternalLvl == -2 || pInternalLvl == -3) // to remove uneccesary comaparison
        {
          //        Serial.println("doNothing");
        }
        else if (pInternalLvl == -1) // detected that a limit switch is pressed
        {
          // Serial.println("Limit Switched Pressed");
          pInternalLvl = 0;
        }
        else if (pInternalLvl == 2 && pLs1) // meaning rotated to lvl1
        {
          // Serial.println("Both Switch Pressed and finally reached level 2");
          pLvl1Pulse = platformMotor.getReadings();
          subLevel1 = pLvl1Pulse * 0.8;
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
          feedback["type"] = "setPOffset";
          feedback["pOffset"] = signOffsetPlatform;
          remote.send(feedback);
          //Serial.println("psignOff: " + String(signOffsetPlatform) + " pLvl1: " + String(pLvl1Pulse) + " Sublvl1: " + String(subLevel1) + " pExtra: " + String(platformExtraPulse) + " oneRing: " + String(oneRingPulse));
          datapick["psignoff"] = signOffsetPlatform;
          datapick["pLvl1Pulse"] = pLvl1Pulse;
          datapick["pExtra"] = platformExtraPulse;
          datapick["sublvl1"] = subLevel1;
          datapick["onering"] = oneRingPulse;
          datapick["type"] = "setupP";
          dataesp.send(datapick);
        }
        else if (pInternalLvl == 1 && pLs2) // meaning rotated to lvl2
        {
          // Serial.println("Both Switch Pressed and finally reached level 1");
          pLvl1Pulse = platformMotor.getReadings();
          if (pLvl1Pulse < 0)
          {
            signOffsetPlatform = -1;
          }
          else
          {
            signOffsetPlatform = 1;
          }
          feedback["type"] = "setPOffset";
          feedback["pOffset"] = signOffsetPlatform;
          remote.send(feedback);
          subLevel1 = pLvl1Pulse * 0.8;
          platformExtraPulse = (pLvl1Pulse - subLevel1) * 0.1;
          subLevel1 = subLevel1 + platformExtraPulse;//check
          oneRingPulse = subLevel1/9.5;
          platformMotor.reset();
          pInternalLvl = 3;
          //Serial.println("psignOff: " + String(signOffsetPlatform) + " pLvl1: " + String(pLvl1Pulse) + " Sublvl1: " + String(subLevel1) + " pExtra: " + String(platformExtraPulse) + " oneRing: " + String(oneRingPulse));
          pMPID.setPulse(signOffsetPlatform * pLvl1Pulse);
          datapick["psignoff"] = signOffsetPlatform;
          datapick["pLvl1Pulse"] = pLvl1Pulse;
          datapick["pExtra"] = platformExtraPulse;
          datapick["sublvl1"] = subLevel1;
          datapick["onering"] = oneRingPulse;
          datapick["type"] = "setupP";
          dataesp.send(datapick);
          // Serial.println("Goto level 1 after both switch press");
        }
      }

      if (!pLs1 && !pLs2 && pInternalLvl == -1) // no ls is pressed
      {
        // Serial.println("Platform noLimit pressed");
        pMPID.setPulse(resetPulse);
      }
      else if (pLs1 && pInternalLvl == 0) // if first ls1 is pressed
      {
        // Serial.println("First LS-1 pressed");
        platformMotor.reset();
        pInternalLvl = 1;
      }
      else if (pLs2 && pInternalLvl == 0) // if first ls2 is pressed
      {
        // Serial.println("First LS-2 pressed");
        platformMotor.reset();
        pInternalLvl = 2;
      }
      else if (!pLs2 && pInternalLvl == 1) // going to level2
      {
        // Serial.println("Goto level 2");
        pMPID.setPulse(-resetPulse);
        pInternalLvl = -2;
      }
      else if (!pLs1 && pInternalLvl == 2) // going to level1
      {
        // Serial.println("Goto level 1");
        pMPID.setPulse(-resetPulse);
        pInternalLvl = -3;
      }
      pMPID.compute();
    }
  }
  else if (init_)
  {
    //    if (rLs1)
    //    {
    //      rMPID.setPulse(rotationMotor.getReadings() + signOffsetRotation * rLvl2Pulse * 0.03);
    //    }
    //    else if (rLs2)
    //    {
    //      rMPID.setPulse(rotationMotor.getReadings() + signOffsetRotation * -1 * rLvl2Pulse * 0.03);
    //    }
    rMPID.compute();
    pMPID.compute();
  }
}

void rotationLvl1(JSONVar msg)
{
  rMPID.setThreshold(200);
  rMPID.setOutputLimits(-30, 30);
  rMPID.setAggTunings(AggKpRotation, AggKiRotation, AggKdRotation);
  rMPID.setSoftTunings(SoftKpRotation, SoftKiRotation, SoftKdRotation);
  //Serial.println("rotationLvl1");
  rotateLevel = 1;
  init_ = true;
  datapick["rotate"] = "LEVEL 1";
  datapick["type"] = "rotation";
  dataesp.send(datapick);
  rMPID.setPulse(0);
}

void rotationLvl2(JSONVar msg)
{
  rMPID.setThreshold(200);
  rMPID.setOutputLimits(-50, 50);
  rMPID.setAggTunings(AggKpRotation, AggKiRotation, AggKdRotation);
  rMPID.setSoftTunings(SoftKpRotation, SoftKiRotation, SoftKdRotation);
  //Serial.println("rotationLvl2");
  datapick["rotate"] = "LEVEL 2";
  datapick["type"] = "rotation";
  dataesp.send(datapick);
  rotateLevel = 2;
  init_ = true;
  rMPID.setPulse(signOffsetRotation * rLvl2Pulse);
}

void platformLvl1(JSONVar msg)
{
  // pMPID.setThreshold(50);
  pMPID.setOutputLimits(-70, 70);
  // pMPID.setAggTunings(AggKpPlatform, AggKiPlatform, AggKdPlatform);
  // pMPID.setSoftTunings(SoftKpPlatform, SoftKiPlatform, SoftKdPlatform);
  //Serial.println("platformLvl1");
  datapick["platform"] = "LEVEL 1";
  datapick["type"] = "plat";
  dataesp.send(datapick);
  platformLevel = 1;
  allRings = true;
  platformSubLevel = 0;
  init_ = true;
  pMPID.setPulse(signOffsetPlatform * pLvl1Pulse);
}

void platformSubLvl2(JSONVar msg)
{
  //    // pMPID.setThreshold(50);
  //    pMPID.setOutputLimits(-100, 100);
  //    // pMPID.setAggTunings(AggKpPlatform, AggKiPlatform, AggKdPlatform);
  //    // pMPID.setSoftTunings(SoftKpPlatform, SoftKiPlatform, SoftKdPlatform);
  //    platformLevel = 2;
  //    platformSubLevel++;
  //    platformSubLevel = platformSubLevel > 10 ? 10 : platformSubLevel;
  //    Serial.println("platformSubLvl2");
  //    init_ = true;
  //    if (allRings)
  //    {
  //      pMPID.setPulse(signOffsetPlatform * subLevel1);
  //      allRings = false;
  //    }
  //    else if (!allRings)
  //    {
  //      pMPID.setPulse(signOffsetPlatform * (subLevel1 - platformSubLevel * oneRingPulse));
  //    }
  //    datapick["platform"] ="LEVEL 2";
  //    datapick["sublevel"]=platformSubLevel;
  //    datapick["type"] = "plat";
  //    dataesp.send(datapick);
}

void setRotateExtraPulse(JSONVar msg)
{
  rMPID.setThreshold(30);
  rMPID.setOutputLimits(-20, 20);
  rMPID.setAggTunings(AggKpRotation, AggKiRotation, AggKdRotation);
  rMPID.setSoftTunings(SoftKpRotation, SoftKiRotation, SoftKdRotation);
  int extraOffset = (int)msg["offset"];
  int setOffset = rotationMotor.getReadings() + (extraOffset * rotationExtraPulse);
  //Serial.println(JSON.stringify(msg));
  //Serial.println("setOffset: " + String(setOffset) + "RotaEP: " + String(rotationExtraPulse));
  rMPID.setPulse(setOffset);
  init_ = true;
  datapick["rExtra"] = rotationExtraPulse;
  datapick["setOffset"] = setOffset;
  datapick["type"] = "rotation";
  dataesp.send(datapick);
}

void setPlatformExtraPulse(JSONVar msg)
{
  int temp = msg["side"];
  int extraOffset = (int)msg["offset"];
  if (temp == 1)
  {
    // pMPID.setThreshold(50);
    pMPID.setOutputLimits(-100, 100);
    // pMPID.setAggTunings(AggKpPlatform, AggKiPlatform, AggKdPlatform);
    // pMPID.setSoftTunings(SoftKpPlatform, SoftKiPlatform, SoftKdPlatform);
    platformLevel = 2;
    
    Serial.println("platformSubLvl2");
    init_ = true;
    if (allRings)
    {
      //      pMPID.setPulse(signOffsetPlatform * subLevel1);
      pMPID.setPulse(-1 * extraOffset * subLevel1);
      Serial.println("lvl1: " + (String)pLvl1Pulse + " UpOffset1: " + (String)int(extraOffset * (int)subLevel1) + " Sublvl: " + (String)subLevel1);
      allRings = false;
    }
    else if (!allRings)
    {
      platformSubLevel++;
      platformSubLevel = platformSubLevel > 11 ? 11 : platformSubLevel;
      //      pMPID.setPulse(signOffsetPlatform * (subLevel1 - platformSubLevel * oneRingPulse));
      pMPID.setPulse(-1 * extraOffset * (subLevel1 - platformSubLevel * oneRingPulse));
      Serial.println("lvl1: " + (String)pLvl1Pulse + " UpOffset2: " + (String)int(extraOffset * (subLevel1 - platformSubLevel * (int) oneRingPulse)) + " Sublvl: " + (String)subLevel1);
    }
    datapick["platform"] = "LEVEL 2";
    datapick["sublevel"] = platformSubLevel;
    datapick["type"] = "plat";
    dataesp.send(datapick);
  }
  else if (temp == -1)
  {
    int extraOffset = (int)msg["offset"];
    platformSubLevel--;
    platformSubLevel = platformSubLevel < 1 ? 1 : platformSubLevel;
    allRings = platformSubLevel < 1 ? !allRings : allRings;
    //    int setOffset = platformMotor.getReadings() + (extraOffset * platformExtraPulse);
    int setOffset = extraOffset * (subLevel1 - platformSubLevel * oneRingPulse);
    //Serial.println(JSON.stringify(msg));
    Serial.println("lvl1: " + (String)pLvl1Pulse +"platformSubLevel" +String( platformSubLevel)+" Off: " + (String)setOffset + " platEP: " + String(platformExtraPulse));
    datapick["pExtra"] = platformExtraPulse;
    datapick["setOffset"] = setOffset;
    datapick["type"] = "plat";
    dataesp.send(datapick);
    pMPID.setPulse(setOffset);
    init_ = true;
    //Serial.println("platformSubLvl1 extra down");
  }
}

void resetAll(JSONVar msg)
{
  //Serial.println("resetAll");
  init_ = false;
  allRings = true;

  rInternalLvl = -1;
  rotateLevel = 0;

  pInternalLvl = -1;
  platformLevel = 0;
  platformSubLevel = 0;
  datapick["res"] = "reset";
  datapick["type"] = "reset";
  dataesp.send(datapick);
}

void pneumaticClose(JSONVar msg)
{
  Serial.println("Pneumatic Close");
  digitalWrite(13, HIGH);
}
void pneumaticOpen(JSONVar msg)
{
  Serial.println("Pneumatic Open");
  digitalWrite(13, LOW);
}
