// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
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

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoBallAutoCmdGroup extends SequentialCommandGroup {
  /** Creates a new TwoBallAutoCmdGroup. */
  public TwoBallAutoCmdGroup(DrivetrainSubsystem drive, StorageSubsystem storage, 
  HangSubsystem hang, IntakeSubsystem intake, ShooterSubsystem shooter, TurretSubsystem turret) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new PrintCommand("TwoBallAutoCmdGroup started!"),  
      new InstantCommand(() -> drive.setGear(Value.kForward)),
      
      new InstantCommand(storage::stopMotors),
      new InstantCommand(turret::stopMotors),
      new InstantCommand(shooter::stopMotors),
      new InstantCommand(hang::stopMotors),

      new ParallelDeadlineGroup(
        new DriveForwardTimedCmd(drive, 4), 
        new IntakeCmdGroup(intake),
        new StorageCmd(storage)
        ),
      /**
       * 4.12.22
       * Post San Jose Regional
       * by Louie Labata
       * 
       * Problems with this autonomous I noticed in competiiton
       * 
       * 1) Timed base is inaccurate and is not repeatable
       * 
       * - was working for the first 4 games but not for the last 5 games
       * 
       * 2) I forgot to add the "StorageCmd(storage)" to index the ball after picking it up
       * 
       * 
       */
      new IntakeRetractCmdGroup(intake),

      new ParallelDeadlineGroup(
        new DriveTurnTimedCmd(drive, 1.3),
        new SequentialCommandGroup(
          new WaitCommand(AutonomousConstants.TURRET_TIMEOUT_SECONDS),
          new RunCommand(turret::turretWithLimelight, turret)
        )),
      
      new WaitCommand(AutonomousConstants.TWO_BALL_TIMEOUT_SECONDS),
      new PrintCommand("Shoot Ball started!"),
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new WaitCommand(AutonomousConstants.FLYWHEEL_REV_TIME_SECONDS),
          new EnableFeederCmd(storage),  
          new WaitCommand(1)
        ),
        new RunCommand(turret::turretWithLimelight, turret),
        new RunCommand(shooter::setTargetBottomFlyWheelVelocity),
        new RunCommand(shooter::setTargetTopFlyWheelVelocity),
        new EnableShooterCmd(shooter)
      ),
      new InstantCommand(shooter::disable),
      new InstantCommand(storage::stopMotors),
      new InstantCommand(turret::stopMotors),
      
      new PrintCommand("TwoBallAutoCmdGroup ended!")
      
    );
  }
}
