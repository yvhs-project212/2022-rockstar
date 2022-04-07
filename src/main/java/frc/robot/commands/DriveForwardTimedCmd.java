// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveForwardTimedCmd extends CommandBase {
  /** Creates a new DriveForwardTimedCmd. */
  private final DrivetrainSubsystem drivetrainSubsystem;

  private double duration;
  private int negative;

  private double timeSet;
  
  public DriveForwardTimedCmd(DrivetrainSubsystem drive, double duration) {
    drivetrainSubsystem = drive;
    addRequirements(drive);

    this.duration = duration;

    timeSet = 0;
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("DriveForwardTimedCmd started!");

    if (duration < 0) {
      negative = -1;
    } else {
      negative = 1;
    }

    timeSet = Math.abs(duration) + Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftMotors = negative * DriveConstants.AUTO_LEFT_DRIVE_FORWARD_SPEED;
    double rightMotors = negative * DriveConstants.AUTO_RIGHT_DRIVE_FORWARD_SPEED;
    
    drivetrainSubsystem.setMotors(leftMotors, rightMotors); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    drivetrainSubsystem.setMotors(0, 0);
    System.out.println("DriveForwardTimedCmd started!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Timer.getFPGATimestamp() > timeSet) {
      return true;
    } else {
      return false;
    }
  }
}
