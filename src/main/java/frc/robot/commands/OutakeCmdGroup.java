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
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OutakeCmdGroup extends SequentialCommandGroup {
  /** Creates a new OutakeCmdGroup. */
  public OutakeCmdGroup(IntakeSubsystem intakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new PrintCommand("OutakeCmdGroup started!"),

      new ParallelCommandGroup(
        new SequentialCommandGroup( 
          new InstantCommand(() -> intakeSubsystem.setPiston(Value.kReverse)),
          new WaitCommand(IntakeConstants.INTAKE_DEPLOY_TIME),
          new InstantCommand(() -> intakeSubsystem.setPiston(Value.kOff))
        ),
        new RunCommand(()-> intakeSubsystem.intakeWithButtons(-(IntakeConstants.INTAKE_SPEED)))
      ),
      new PrintCommand("OutakeCmdGroup ended!")
    );
  }
}
