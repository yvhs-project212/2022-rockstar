// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.StorageConstants;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.StorageSubsystem.MotorSelection;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootOneBallCmdGroup extends SequentialCommandGroup {
  /** Creates a new ShootOneBallCmdGroup. */
  public ShootOneBallCmdGroup(StorageSubsystem storage) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());


    // How I made this 
    /**
     * https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/frisbeebot/RobotContainer.java
     * 
     */
    addCommands(
      new PrintCommand("ShootOneBallCmdGroup started!"),
      new WaitUntilCommand(RobotContainer.shooter::bothFlywheelsAtSetpoint),
      new InstantCommand(() -> storage.setMotors(MotorSelection.ALL, StorageConstants.INDEXER_SPEED_AUTO, StorageConstants.FEEDER_SPEED_AUTO)), 
      new RunCommand(() -> storage.runMotors()),
      new WaitUntilCommand(storage::getOneBallPassedThrough),
      new PrintCommand("ShootOneBallCmdGroup ended!")
    );
  }
}
