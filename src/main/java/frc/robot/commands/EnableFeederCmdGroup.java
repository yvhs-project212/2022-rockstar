// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.StorageConstants;
import frc.robot.subsystems.StorageSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class EnableFeederCmdGroup extends SequentialCommandGroup {
  /** Creates a new EnableFeederCmdGroup. */
  public EnableFeederCmdGroup(StorageSubsystem storage) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new PrintCommand("EnableFeederCmdGroup started!"),
      new ConditionalCommand(
        new SequentialCommandGroup(
          new ShootOneBallCmdGroup(storage),
          new InstantCommand(() -> storage.setSecondBall(false)),
          new WaitUntilCommand(StorageConstants.TIME_BETWEEN_SHOTS),
          new ShootOneBallCmdGroup(storage),
          new InstantCommand(() -> storage.setFirstBall(false))
        ),

        new SequentialCommandGroup(
        new ShootOneBallCmdGroup(storage),
        new InstantCommand(() -> storage.setFirstBall(false)) 
        ),
      
        storage::getSecondBall),
      new PrintCommand("EnableFeederCmdGroup ended!")
    );

    /*
      new PrintCommand("EnableFeederCmdGroup started!"),

      // First if-else (is flywheel at setpoint?)   - LL 3.23.22
      new ConditionalCommand(
        // true
        new SequentialCommandGroup(
        new PrintCommand("Feeder started!")
        // if else
        // if: run two ball command
        // else: run one ball command
      )
      
      , // false 
        new SequentialCommandGroup(
        
        new PrintCommand("Feeder stopped!"),
        new InstantCommand(() -> storage.setMotors(MotorSelection.NONE, 0, 0)),
        new 
      )
      , // condition 
      RobotContainer.shooter::bothFlywheelsAtSetpoint),

      new PrintCommand("EnableFeederCmdGroup ended!")
    );

    /**
     *  if (flywheel at setpoint) { 
     *  begin feeding ball
     *  if (two ball) {
     *  feed one ball
     *  wait
     *  feed next ball
     * } else {
     *  feed ball
     * }
     * } else {
     *  storage off
     * }
     * 
     * 
     * 
     * 
     * 
     *  if (RobotContainer.shooter.bottomFlywheelAtSetpoint() 
    && RobotContainer.shooter.topFlywheelAtSetpoint()) {
       storage.setMotors(MotorSelection.ALL, 0.5, 0.5);
       storage.runMotors();
       
       System.out.println("Indexer and Feeder begun!");
    } else {
      storage.stopMotors();
      System.out.println("Shooter is not at target velocity!");
    }
  }
     */
  }
}
