// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HangSubsystem;
import frc.robot.subsystems.StorageSubsystem;

public class DriveForwardTimedCmd extends CommandBase {
  /** Creates a new DriveForwardTimedCmd. */
  private final DrivetrainSubsystem drivetrainSubsystem;
  private final StorageSubsystem storageSubsystem;
  private final HangSubsystem hangSubsystem;

  private final double timeSet;
  
  public DriveForwardTimedCmd(DrivetrainSubsystem drive, StorageSubsystem storage, 
  HangSubsystem hang, double duration) {
    drivetrainSubsystem = drive;
    storageSubsystem = storage;
    hangSubsystem = hang;
    addRequirements(drive, storage, hang);

    timeSet = duration + Timer.getFPGATimestamp();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("DriveForwardTimedCmd started!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Meters Driven", drivetrainSubsystem.getEncoderMeters(hangSubsystem.getHangLeftSelectedSensorPosition(),
    storageSubsystem.getFeederSensorPosition()));
    drivetrainSubsystem.setMotors(DriveConstants.AUTO_LEFT_DRIVE_FORWARD_SPEED, DriveConstants.AUTO_RIGHT_DRIVE_FORWARD_SPEED); 
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
