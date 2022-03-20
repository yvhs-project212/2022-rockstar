// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PWM;

public class StorageSubsystem extends SubsystemBase {
  /** Creates a new StorageSubsystem. */
  private final WPI_TalonSRX indexer;
  private final WPI_TalonSRX feeder;

  public MotorSelection setMotorSelection;
  public MotorSelection runMotorSelection;
  public double indexerSpeed;
  public double feederSpeed;

  public AnalogTrigger bottomTrigger;

  public I2C.Port i2cPort_OnBoard;
  public ColorSensorV3 middleColorSensor;

  public I2C.Port i2cPort_External;
  public ColorSensorV3 topColorSensor;

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

    bottomTrigger = new AnalogTrigger(PWM.Storage.BOTTOM_TRIGGER);
    bottomTrigger.setLimitsVoltage(1.5, 4);

    i2cPort_OnBoard = I2C.Port.kOnboard;
    middleColorSensor = new ColorSensorV3(i2cPort_OnBoard);

    i2cPort_External= I2C.Port.kMXP;
    middleColorSensor = new ColorSensorV3(i2cPort_External);

    setMotorSelection = MotorSelection.NONE;
    runMotorSelection = MotorSelection.NONE;
    indexerSpeed = 0;
    feederSpeed = 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
    this.indexer.set(indexerSpeed);
    this.feeder.set(0.0);
  }

  public void feeder(double feederSpeed) {
    this.indexer.set(0.0);
    this.feeder.set(feederSpeed);
  }

  public void all(double indexerSpeed, double feederSpeed) {
    this.indexer.set(indexerSpeed);
    this.feeder.set(feederSpeed);
  }

  public void reverseAll(double indexerSpeed, double feederSpeed) {
    this.indexer.set(-indexerSpeed);
    this.feeder.set(-feederSpeed);
  }
  public void reverseIndexer(double indexerSpeed) {
    this.indexer.set(-indexerSpeed);
    this.feeder.set(0);
  }
  public void reverseFeeder(double feederSpeed) {
    this.indexer.set(0);
    this.feeder.set(-feederSpeed);
  }

  public void stopMotors() {
    indexer.stopMotor();
    feeder.stopMotor();
  }
}
