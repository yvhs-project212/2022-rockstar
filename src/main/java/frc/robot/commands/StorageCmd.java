// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.StorageConstants;
import frc.robot.subsystems.StorageSubsystem;

public class StorageCmd extends CommandBase {
  /** Creates a new StorageCmd. */
  StorageSubsystem storage;

  public StorageCmd(StorageSubsystem storage) {
    this.storage = storage;
    addRequirements(storage);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("StorageCmd started!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* Old

    if (RobotContainer.gunnerJoystick.getRightTriggerAxis() > 0.15) {
      storage.setMotors(StorageSubsystem.MotorSelection.REVERSE_ALL, StorageConstants.INDEXER_SPEED, StorageConstants.FEEDER_SPEED);
      storage.runMotors();
    } else if (RobotContainer.gunnerJoystick.getLeftTriggerAxis() > 0.15) {
      storage.setMotors(StorageSubsystem.MotorSelection.INDEXER, StorageConstants.INDEXER_SPEED, StorageConstants.FEEDER_SPEED);
      storage.runMotors();
    } else {
      storage.setMotors(StorageSubsystem.MotorSelection.NONE, 0, 0);
      storage.runMotors();
    }
    */

    // Autonomous storage
    
    if (storage.getTopColorSensorBoolean() && storage.getMiddleColorSensorBoolean()) {
      // Stop
      storage.setMotors(StorageSubsystem.MotorSelection.NONE, 0, 0);
    } else if (storage.getMiddleColorSensorBoolean()) {
      // Go
      storage.setMotors(StorageSubsystem.MotorSelection.INDEXER, StorageConstants.INDEXER_SPEED, StorageConstants.FEEDER_SPEED);
    } else if (storage.getBottomSensorBoolean()) {
      // Go
      storage.setMotors(StorageSubsystem.MotorSelection.INDEXER, StorageConstants.INDEXER_SPEED, StorageConstants.FEEDER_SPEED);
    } else {
      // Stop
      storage.setMotors(StorageSubsystem.MotorSelection.NONE, 0, 0);
    }
    storage.runMotors();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("StorageCmd ended!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
