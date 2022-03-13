// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveWithJoysticksCmd extends CommandBase {
  /** Creates a new DriveWithJoysticksCmd. */
  DrivetrainSubsystem drive;
  public DriveWithJoysticksCmd(DrivetrainSubsystem drive) {
    this.drive = drive;
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("DriveWithJoysticksCmd started!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.driveWithJoysticks(RobotContainer.driverJoystick, DriveConstants.FORWARD_SPEED, DriveConstants.TURN_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("DriveWithJoysticksCmd ended!");}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
