// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HangSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.ShooterSubsystem.VelocityControlMode;
import frc.robot.subsystems.StorageSubsystem.MotorSelection;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneBallAutoCmdGroup extends SequentialCommandGroup {
  /** Creates a new OneBallAutoCmdGroup. */

  public OneBallAutoCmdGroup(DrivetrainSubsystem drive, StorageSubsystem storage, 
  HangSubsystem hang, IntakeSubsystem intake, ShooterSubsystem shooter, TurretSubsystem turret) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    //addRequirements(drivetrainSubsystem);
    addCommands(
      new PrintCommand("OneBallAutoCmdGroup started!"),
      
      /**
       * Fix "DriveForwardCmd":
       * 
       * You cannot go backwards
       * 
       */
      new DriveForwardCmd(drive, storage, hang, -4),
      
      /**
       * Figure out how to do a ParallelDeadlineCommandGroup
       * 
       * So I can get out of shooting a ball once
       */
      
      new ParallelCommandGroup(
        new RunCommand(turret::turretWithLimelight, turret),
        new RunCommand(shooter::setTargetBottomFlyWheelVelocity),
        new RunCommand(shooter::setTargetBottomFlyWheelVelocity),
        new EnableShooterCmd(shooter, VelocityControlMode.LIMELIGHT),
        new WaitCommand(AutonomousConstants.FLYWHEEL_REV_TIME_SECONDS),
        new EnableFeederCmd(storage))
      .withTimeout(AutonomousConstants.TIMEOUT_SECONDS)
      .andThen(
        new InstantCommand(shooter::disable),
        new InstantCommand(() -> storage.setMotors(MotorSelection.NONE, 0, 0))
      ),
    
      new PrintCommand("OneBallAutoCmdGroup ended!")
      
    );
  }
}
