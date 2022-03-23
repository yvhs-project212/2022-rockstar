// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.TurretSubsystem;

public class TurretCmd extends CommandBase {
  /** Creates a new TurretCmd. */
  TurretSubsystem turret;

  public TurretCmd(TurretSubsystem turret) {
    this.turret = turret;
    addRequirements(turret);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("TurretCmd started!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (RobotContainer.gunnerJoystick.getXButton()) {
      turret.turretWithLimelight();
    } else {
      turret.turretWithJoysticks(RobotContainer.gunnerJoystick, TurretConstants.TURRET_SPEED);
    }
    

    //turret.turretWithJoysticks(RobotContainer.gunnerJoystick, TurretConstants.TURRET_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("TurretCmd ended!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
