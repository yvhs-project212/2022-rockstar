// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PWM;
import frc.robot.Constants.StorageConstants;

public class StorageSubsystem extends SubsystemBase {
  /** Creates a new StorageSubsystem. */
  private final WPI_TalonSRX indexer;
  private final WPI_TalonSRX feeder;

  public MotorSelection setMotorSelection;
  public MotorSelection runMotorSelection;
  public double indexerSpeed;
  public double feederSpeed;

  public DigitalInput bottomBeamBreak;

  public I2C.Port i2cPort_OnBoard;
  public ColorSensorV3 middleColorSensor;

  public I2C.Port i2cPort_External;
  public ColorSensorV3 topColorSensor;

  public boolean firstBall;
  public boolean secondBall;

  public boolean middleCache;
  public boolean topCache;

  public enum MotorSelection {
    NONE,
    ALL, 
    INDEXER, 
    FEEDER, 
    REVERSE_ALL,
    REVERSE_INDEXER,
    REVERSE_FEEDER;
  }


  public StorageSubsystem() {
    setMotorSelection = StorageSubsystem.MotorSelection.NONE;
    indexer = new WPI_TalonSRX(PWM.Storage.INDEXER);
    indexer.setInverted(true);

    feeder = new WPI_TalonSRX(PWM.Storage.FEEDER);
    feeder.setInverted(false);
    resetEncoder();

    bottomBeamBreak = new DigitalInput(PWM.Storage.BOTTOM_BEAM_BREAK);

    /*
    bottomTriggerRight = new AnalogTrigger(PWM.Storage.BOTTOM_TRIGGER_RIGHT);
    bottomTriggerRight.setLimitsVoltage(4.3, 4.3);
    bottomTriggerLeft.setAveraged(true);
    */

    i2cPort_External= I2C.Port.kMXP;
    middleColorSensor = new ColorSensorV3(i2cPort_External);

    i2cPort_OnBoard = I2C.Port.kOnboard;
    topColorSensor = new ColorSensorV3(i2cPort_OnBoard);

    setMotorSelection = MotorSelection.NONE;
    runMotorSelection = MotorSelection.NONE;
    indexerSpeed = 0;
    feederSpeed = 0;

    firstBall = false;
    secondBall = false;

    middleCache = false;
    topCache = false;

    SmartDashboard.putBoolean("Top Sensor", false);
    SmartDashboard.putBoolean("Middle Sensor", false);
    SmartDashboard.putBoolean("Bottom Sensor", false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putData(RobotContainer.storage);
    getBottomSensorBoolean();
    getMiddleColorSensorBoolean();
    getTopColorSensorBoolean();
    
    //SmartDashboard.putNumber("Right Drivetrain Encoder", feeder.getSelectedSensorPosition());
  }

  public void setMotors(final StorageSubsystem.MotorSelection setMotorSelection, double userSelectedIndexerSpeed, double userSelectedFeederSpeed) {
    runMotorSelection = setMotorSelection;
    indexerSpeed = userSelectedIndexerSpeed;
    feederSpeed = userSelectedFeederSpeed;
  }

  public void runMotors() {
    switch (runMotorSelection) {
      case NONE:
        stopMotors();
        break;
      case INDEXER:
        indexer(indexerSpeed);
        break;
      case FEEDER:
        feeder(feederSpeed);
        break;
      case ALL:
        all(indexerSpeed, feederSpeed);
        break;
      case REVERSE_ALL:
        reverseAll(indexerSpeed, feederSpeed);
        break;
      case REVERSE_FEEDER:
        reverseFeeder(indexerSpeed);
        break;
      case REVERSE_INDEXER:
        reverseIndexer(indexerSpeed);
        break;
    }
  }
  
  public void indexer(double indexerSpeed) {
    indexer.set(indexerSpeed);
    feeder.set(0.0);
  }

  public void feeder(double feederSpeed) {
    indexer.set(0.0);
    feeder.set(feederSpeed);
  }

  public void all(double indexerSpeed, double feederSpeed) {
    indexer.set(indexerSpeed);
    feeder.set(feederSpeed);
  }

  public void reverseAll(double indexerSpeed, double feederSpeed) {
    indexer.set(-indexerSpeed);
    feeder.set(-feederSpeed);
  }
  public void reverseIndexer(double indexerSpeed) {
    indexer.set(-indexerSpeed);
    feeder.set(0);
  }
  public void reverseFeeder(double feederSpeed) {
    indexer.set(0);
    feeder.set(-feederSpeed);
  }

  public boolean getBottomSensorBoolean() {
    boolean cache;

    if (!bottomBeamBreak.get()) {
      cache = true;
    } else {
      cache = false;
    }

    if (cache != SmartDashboard.getBoolean("Bottom Sensor", false)) {
      SmartDashboard.putBoolean("Bottom Sensor", cache);
    }

    return cache;
  }

  public boolean getMiddleColorSensorBoolean() {
    if (middleCache != SmartDashboard.getBoolean("Middle Sensor", false)) {
      SmartDashboard.putBoolean("Middle Sensor", middleCache);
    }

    //SmartDashboard.putNumber("Middle Sensor Proximity", middleColorSensor.getProximity());
    if (middleColorSensor.getProximity() > StorageConstants.MIDDLE_PROXIMITY) {
      //SmartDashboard.putBoolean("Middle Sensor", true);
      middleCache = true;
      return true;
    } else {
      //SmartDashboard.putBoolean("Middle Sensor", false);
      middleCache = false;
      return false;
    }


  }

  public boolean getTopColorSensorBoolean() {

    if (topCache != SmartDashboard.getBoolean("Top Sensor", false)) {
      SmartDashboard.putBoolean("Top Sensor", topCache);
    }

    //SmartDashboard.putNumber("Top Sensor Proximity", topColorSensor.getProximity());
    if (topColorSensor.getProximity() > StorageConstants.TOP_PROXIMITY) {
      //SmartDashboard.putBoolean("Top Sensor", true);
      topCache = true;
      return true;
    } else {
      //SmartDashboard.putBoolean("Top Sensor", false);
      topCache = false;
      return false;
    }
  }

  public void setFirstBall(boolean bool) {
    firstBall = bool;
  }

  public boolean getFirstBall() {
    return firstBall;
  }

  public void setSecondBall(boolean bool) {
    secondBall = bool;
  }

  public boolean getSecondBall() {
    return secondBall;
  }

  public boolean getOneBallPassedThrough() {

    if (getSecondBall()) {
      // if there is two balls
      if (getTopColorSensorBoolean() && (getMiddleColorSensorBoolean() == false) && (getTopColorSensorBoolean() == false)) {
        return true;
      } else {
        return false;
      }
    } else if (getFirstBall()) {
      // if there is one ball
      if ((getTopColorSensorBoolean() == false)&&(getMiddleColorSensorBoolean() == false)&&
      (getBottomSensorBoolean() == false)) {
        return true;
      } else {
        return false;
      }
    } else {
      return false;
    }
  }
  
  public void resetEncoder() {
    feeder.setSelectedSensorPosition(0);
  }

  public double getFeederSensorPosition() {
    return feeder.getSelectedSensorPosition();
  }
  


  public void stopMotors() {
    indexer.stopMotor();
    feeder.stopMotor();
  }
}
