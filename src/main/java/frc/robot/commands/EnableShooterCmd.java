// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.VelocityControlMode;

public class EnableShooterCmd extends CommandBase {
  /** Creates a new EnableShooterCmd. */
  private final ShooterSubsystem shooter;

  private final VelocityControlMode mode;


  public EnableShooterCmd(ShooterSubsystem shooter, VelocityControlMode mode) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    addRequirements(shooter);

    this.mode = mode;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("EnableShooterCmd started!");
    shooter.setVelocityControlMode(mode);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.enable(shooter.getTargetBottomFlyWheelVelocity(), shooter.getTargetTopFlyWheelVelocity());
    
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
