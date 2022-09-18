// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
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
      new InstantCommand(() -> drive.setGear(Value.kForward)),
      
      new InstantCommand(storage::stopMotors),
      new InstantCommand(turret::stopMotors),
      new InstantCommand(shooter::stopMotors),
      new InstantCommand(hang::stopMotors),

      new DriveForwardTimedCmd(drive, -4),
      
      new WaitCommand(AutonomousConstants.ONE_BALL_TIMEOUT_SECONDS),
      new PrintCommand("Shoot Ball started!"),
      new ParallelCommandGroup(
        new RunCommand(turret::turretWithLimelight, turret),
        new RunCommand(shooter::setTargetBottomFlyWheelVelocity),
        new RunCommand(shooter::setTargetTopFlyWheelVelocity),
        new EnableShooterCmd(shooter),
        new SequentialCommandGroup(
          new WaitCommand(AutonomousConstants.FLYWHEEL_REV_TIME_SECONDS),
          new EnableFeederCmd(storage))  
        )
      .withTimeout(AutonomousConstants.ONE_BALL_TIMEOUT_SECONDS)
      .andThen(
        new InstantCommand(shooter::disable),
        new InstantCommand(() -> storage.setMotors(MotorSelection.NONE, 0, 0)),
        new InstantCommand(storage::runMotors)
      ),
    
      new PrintCommand("OneBallAutoCmdGroup ended!")
      
    );
  }
}
