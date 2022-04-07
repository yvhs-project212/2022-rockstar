// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.StorageConstants;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.StorageSubsystem.MotorSelection;

public class StorageCmd extends CommandBase {
  /** Creates a new StorageCmd. */
  StorageSubsystem storage;
  boolean flag = false;

  public StorageCmd(StorageSubsystem storage) {
    this.storage = storage;
    addRequirements(storage);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("StorageCmd started!");
    storage.setFirstBall(false);
    storage.setSecondBall(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Autonomous storage
    
    if (RobotContainer.driverJoystick.getLeftBumper()) {
      storage.setMotors(StorageSubsystem.MotorSelection.REVERSE_ALL, StorageConstants.INDEXER_SPEED, StorageConstants.FEEDER_SPEED);
      storage.setFirstBall(false);
      storage.setSecondBall(false);
    } else {

      // Finding out how many balls are in storage
      if (storage.getBottomSensorBoolean()) {
        storage.setFirstBall(true);
      } 

      if (storage.getTopColorSensorBoolean() && storage.getBottomSensorBoolean()) {
        storage.setSecondBall(true);
      }

      // Autonomous Storage 
      if ((storage.getTopColorSensorBoolean()) == false) {
        // if you DO NOT see ball at top
        if (storage.getBottomSensorBoolean() || 
        storage.getMiddleColorSensorBoolean() || storage.getFirstBall()) {
          storage.setMotors(MotorSelection.INDEXER, StorageConstants.INDEXER_SPEED_AUTO, StorageConstants.FEEDER_SPEED_AUTO);  
        } else {
          storage.setMotors(MotorSelection.NONE, 0, 0);
        }
      } else if ((storage.getMiddleColorSensorBoolean()) == false) {
        // if you DO NOT see ball in the middle
        if (storage.getBottomSensorBoolean() || storage.getSecondBall()) {
          storage.setMotors(MotorSelection.INDEXER, StorageConstants.INDEXER_SPEED_AUTO, StorageConstants.FEEDER_SPEED_AUTO);  
        } else {
          storage.setMotors(MotorSelection.NONE, 0, 0);
        }
      } else {
        // if you DO NOT see ball anywhere
        storage.setMotors(MotorSelection.NONE, 0, 0);
      }
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
