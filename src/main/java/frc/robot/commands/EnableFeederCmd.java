// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.StorageConstants;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.StorageSubsystem.MotorSelection;

public class EnableFeederCmd extends CommandBase {
  /** Creates a new EnableFeederCmd. */
  StorageSubsystem storage;

  public EnableFeederCmd(StorageSubsystem storage) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.storage = storage;
    addRequirements(storage);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("EnableFeederCmd started!");
    storage.setFirstBall(false);
    storage.setSecondBall(false);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*
    if (RobotContainer.shooter.bothFlywheelsAtSetpoint()) {
       storage.setMotors(MotorSelection.ALL, StorageConstants.INDEXER_SPEED_AUTO, StorageConstants.FEEDER_SPEED_AUTO);
       storage.runMotors();

       //System.out.println("Indexer and Feeder begun!");
    } else {
      storage.stopMotors();
      //System.out.println("Shooter is not at target velocity!");
    }
    */
    storage.setMotors(MotorSelection.ALL, StorageConstants.INDEXER_SPEED, StorageConstants.FEEDER_SPEED);
    storage.runMotors();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("EnableFeederCmd Ended!");
    storage.setFirstBall(false);
    storage.setSecondBall(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
