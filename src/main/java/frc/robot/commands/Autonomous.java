// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AutonomousPickerSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HangSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class Autonomous extends CommandBase {
  private final AutonomousPickerSubsystem autonomousPicker;
  private final DrivetrainSubsystem drive;
  private final StorageSubsystem storage;
  private final HangSubsystem hang;
  private final IntakeSubsystem intake;
  private final ShooterSubsystem shooter;
  private final TurretSubsystem turret;

  /*StorageSubsystem storage, 
  HangSubsystem hang, IntakeSubsystem intake, ShooterSubsystem shooter, TurretSubsystem turret*/
  /** Creates a new Autonomous. */
  public Autonomous(AutonomousPickerSubsystem autonomousPicker, DrivetrainSubsystem drive, StorageSubsystem storage, 
  HangSubsystem hang, IntakeSubsystem intake, ShooterSubsystem shooter, TurretSubsystem turret ) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.autonomousPicker = autonomousPicker;
    this.drive = drive;
    this.storage = storage;
    this.hang = hang;
    this.intake = intake;
    this.shooter = shooter;
    this.turret = turret;

    addRequirements(autonomousPicker, drive, storage, hang, intake, shooter, turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Autonomous started!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  public Command runAutonomous() {
    Command autoCommands = new SequentialCommandGroup();

    switch(autonomousPicker.getAutonomous()) { //switch (Statement) ==> Switch what you do based on what you input/enter - LL
       case NONE: //This is the input of the user and the if of an if statement - LL
        System.out.println("No Balls");
        autoCommands = new SequentialCommandGroup(
          new InstantCommand(() -> drive.setGear(Value.kForward)),
          new DriveForwardTimedCmd(drive, -4)
        );
        break;
      case ONE:
        System.out.println("One Ball");
        autoCommands = new SequentialCommandGroup (
          new OneBallAutoCmdGroup(drive, storage, hang, intake, shooter, turret)
        );
        break;
      case TWO:
        System.out.println("Two Ball");
        autoCommands = new SequentialCommandGroup (
          new TwoBallAutoCmdGroup(drive, storage, hang, intake, shooter, turret)
        );
        break;
      default:
        autoCommands = new SequentialCommandGroup (
          new DriveForwardTimedCmd(drive, 0)
        );
        break;
    } //End of Switch
    return autoCommands;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Autonomous ended!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
