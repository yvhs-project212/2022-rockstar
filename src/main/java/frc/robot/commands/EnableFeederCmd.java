// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
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
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.shooter.bottomFlywheelAtSetpoint() 
    && RobotContainer.shooter.topFlywheelAtSetpoint()) {
       storage.setMotors(MotorSelection.ALL, 0.5, 0.5);
       storage.runMotors();
       System.out.println("Indexer and Feeder begun!");
    } else {
      storage.stopMotors();
      System.out.println("Shooter is not at target velocity!");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Indexer and Feeder ended!");
    System.out.println("EnableFeederCmd Ended!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
