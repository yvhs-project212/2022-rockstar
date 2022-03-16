// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PWM;

public class StorageSubsystem extends SubsystemBase {
  /** Creates a new StorageSubsystem. */
  private final WPI_TalonSRX indexer;
  private final WPI_TalonSRX feeder;

  public MotorSelection setMotorSelection;
  public MotorSelection runMotorSelection;
  public double motorSpeed;

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
    indexer.setInverted(false);

    feeder = new WPI_TalonSRX(PWM.Storage.FEEDER);
    feeder.setInverted(true);

  setMotorSelection = MotorSelection.NONE;
  runMotorSelection = MotorSelection.NONE;
  motorSpeed = 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setMotors(final StorageSubsystem.MotorSelection setMotorSelection, double userSelectedMotorSpeed) {
    runMotorSelection = setMotorSelection;
    motorSpeed = userSelectedMotorSpeed;
  }

  public void runMotors() {
    switch (runMotorSelection) {
      case NONE:
        stopMotors();
        break;
      case INDEXER:
        indexer(motorSpeed);
        break;
      case FEEDER:
        feeder(motorSpeed);
        break;
      case ALL:
        all(motorSpeed);
        break;
      case REVERSE_ALL:
        reverseAll(motorSpeed);
        break;
      case REVERSE_FEEDER:
        reverseFeeder(motorSpeed);
        break;
      case REVERSE_INDEXER:
        reverseIndexer(motorSpeed);
        break;
    }
  }
  
  public void indexer(double speed) {
    this.indexer.set(speed);
    this.feeder.set(0.0);
  }

  public void feeder(double speed) {
    this.indexer.set(0.0);
    this.feeder.set(speed);
  }

  public void all(double speed) {
    this.indexer.set(speed);
    this.feeder.set(speed);
  }

  public void reverseAll(double speed) {
    this.indexer.set(-speed);
    this.feeder.set(-speed);
  }
  public void reverseIndexer(double speed) {
    this.indexer.set(-speed);
    this.feeder.set(0);
  }
  public void reverseFeeder(double speed) {
    this.indexer.set(0);
    this.feeder.set(-speed);
  }

  public void stopMotors() {
    indexer.stopMotor();
    feeder.stopMotor();
  }
}
