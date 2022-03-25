// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class EnableShooterCmd extends CommandBase {
  /** Creates a new EnableShooterCmd. */
  ShooterSubsystem shooter;

  public EnableShooterCmd(ShooterSubsystem shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("EnableShooterCmd started!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*
    if (shooter.getManualMode()) {
      shooter.enable();
      //System.out.println("Shooter Manual Mode enabled!");
    } else {
      //System.out.println("Shooter Manual Mode disabled!");
      if (shooter.goalDetected()) {
        shooter.enable();
        //System.out.println("Shooter enabled!");
      }
      else {
        shooter.disable();
        //System.out.println("Shooter disabled!");
      }
    }
    */

    if (RobotContainer.gunnerJoystick.getPOV() > 135 &&
      RobotContainer.gunnerJoystick.getPOV() < 225) {
      // 180 POV
      // 7 FEET --> LINE 
      shooter.setStaticMode(true);
      shooter.setTargetTopFlyWheelVelocity(ShooterConstants.SEVEN_VELOCITY_TOP);
      shooter.setTargetBottomFlyWheelVelocity(ShooterConstants.SEVEN_VELOCITY_BOTTOM);
      shooter.enable();
    } else if (RobotContainer.gunnerJoystick.getPOV() > 45 &&
    RobotContainer.gunnerJoystick.getPOV() < 135) {
      // 90 POV
      // 10 FEET
      shooter.setStaticMode(true);
      shooter.setTargetTopFlyWheelVelocity(ShooterConstants.TEN_VELOCITY_TOP);
      shooter.setTargetBottomFlyWheelVelocity(ShooterConstants.TEN_VELOCITY_BOTTOM);
      shooter.enable();
    } else if (RobotContainer.gunnerJoystick.getPOV() == 0) {
      // 0 POV
      // 12 FEET
      shooter.setStaticMode(true);
      shooter.setTargetTopFlyWheelVelocity(ShooterConstants.TWELVE_VELOCITY_TOP);
      shooter.setTargetBottomFlyWheelVelocity(ShooterConstants.TWELVE_VELOCITY_BOTTOM);
      shooter.enable();
    } else {
      shooter.setStaticMode(false);
      shooter.setTargetTopFlyWheelVelocity(0);
      shooter.setTargetBottomFlyWheelVelocity(0);
      shooter.enable();
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.disable();
    System.out.println("EnableShooterCmd ended!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
