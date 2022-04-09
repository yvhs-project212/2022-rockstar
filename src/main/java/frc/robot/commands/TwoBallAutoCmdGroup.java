// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
import frc.robot.subsystems.StorageSubsystem.MotorSelection;

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

      new ParallelDeadlineGroup(
        new DriveForwardTimedCmd(drive, 4), 
        new IntakeCmdGroup(intake)
        ),
      new IntakeRetractCmdGroup(intake),
      new DriveTurnTimedCmd(drive, 4),
      
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
      
      new PrintCommand("TwoBallAutoCmdGroup ended!")
      
    );
  }
}
