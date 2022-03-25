// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveForwardCmd extends CommandBase {
  /** Creates a new DriveForwardCmd. */
  private final DrivetrainSubsystem drivetrainSubsystem;
  private final double encoderSetpoint;

  public DriveForwardCmd(DrivetrainSubsystem drivetrainSubsystem, double distanceMeters) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrainSubsystem = drivetrainSubsystem;
    addRequirements(drivetrainSubsystem);

    encoderSetpoint = drivetrainSubsystem.getEncoderMeters() + distanceMeters;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("DriveForwardCmd started!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrainSubsystem.setMotors(DriveConstants.AUTO_LEFT_DRIVE_FORWARD_SPEED, DriveConstants.AUTO_RIGHT_DRIVE_FORWARD_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.setMotors(0, 0);
    System.out.println("DriveForwardCmd ended!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (drivetrainSubsystem.getEncoderMeters() > encoderSetpoint) {
      return true;
    } else {
      return false;
    }
  }
}
