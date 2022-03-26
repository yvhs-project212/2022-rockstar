// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ExtendIntakeCmdGroup;
import frc.robot.commands.OneBallAutoCmdGroup;
import frc.robot.commands.DriveForwardCmd;
import frc.robot.commands.DriveWithJoysticksCmd;
import frc.robot.commands.DriveWithLimelightCmd;
import frc.robot.commands.EnableFeederCmd;
import frc.robot.commands.EnableShooterCmd;
import frc.robot.commands.HangCmd;
import frc.robot.commands.IntakeWithPaddlesCmd;
import frc.robot.commands.ShooterCmd;
import frc.robot.commands.StorageCmd;
import frc.robot.commands.TurretCmd;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HangSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.ShooterSubsystem.VelocityControlMode;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
  // DriveTrain files - LL
  public static DrivetrainSubsystem driveTrain = new DrivetrainSubsystem();
  private final DriveWithJoysticksCmd driveWithJoysticksCmd = new DriveWithJoysticksCmd(driveTrain);
  private final DriveForwardCmd driveForwardCmd = new DriveForwardCmd(driveTrain, storage, hang, 0);
  private final DriveWithLimelightCmd driveWithLimelightCmd = new DriveWithLimelightCmd(driveTrain);

  // Hang files - LL
  public static HangSubsystem hang = new HangSubsystem();
  private final HangCmd hangCmd = new HangCmd(hang);

  // Shooter files - LL
  public static ShooterSubsystem shooter = new ShooterSubsystem();
  private final ShooterCmd shooterCmd = new ShooterCmd(shooter);
  private final EnableShooterCmd enableShooterCmd = new EnableShooterCmd(shooter, VelocityControlMode.LIMELIGHT);

  // Storage files - LL
  public static StorageSubsystem storage = new StorageSubsystem();
  private final EnableFeederCmd enableFeederCmd = new EnableFeederCmd(storage);
  private final StorageCmd storageCmd = new StorageCmd(storage);
  //private final EnableFeederCmdGroup enableFeederCmdGroup = new EnableFeederCmdGroup(storage);

  // Intake files - LL
  public static IntakeSubsystem intake = new IntakeSubsystem();
  private final IntakeWithPaddlesCmd intakeWithPaddlesCmd = new IntakeWithPaddlesCmd(intake);
  private final ExtendIntakeCmdGroup deployIntakeCmdGroup = new ExtendIntakeCmdGroup(intake);
  
  // Turret files - LL
  public static TurretSubsystem turret = new TurretSubsystem();
  private final TurretCmd turretCmd = new TurretCmd(turret);
  
  // Autonomous - LL
  private final OneBallAutoCmdGroup autonomousCmdGroup = new OneBallAutoCmdGroup(driveTrain, storage, hang, intake, shooter, turret);

  // Controller files - LL
  public static XboxController driverJoystick = new XboxController(Constants.OI.DRIVER_NUMBER);
  public static XboxController gunnerJoystick = new XboxController(Constants.OI.GUNNER_NUMBER);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    System.out.println("In the Construtor for RobotContainer");

    //INITALIZE all the things we made - LL

    driveTrain.setDefaultCommand(driveWithJoysticksCmd);   //Drive is always looking to read this command - LL
    hang.setDefaultCommand(hangCmd);
    intake.setDefaultCommand(intakeWithPaddlesCmd);
    shooter.setDefaultCommand(shooterCmd);
    storage.setDefaultCommand(storageCmd);
    turret.setDefaultCommand(turretCmd);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Driver Joystick buttons  -----------------------------------------------------------------

    // Low gear
    new JoystickButton(driverJoystick, XboxController.Button.kA.value)
      .whenPressed(new InstantCommand(() -> driveTrain.setGear(Value.kForward)));
    // High gear
    new JoystickButton(driverJoystick, XboxController.Button.kB.value)
      .whenPressed(new InstantCommand(() -> driveTrain.setGear(Value.kReverse)));

    // Transerval forward
    new JoystickButton(driverJoystick, XboxController.Button.kLeftBumper.value)
      .whenPressed(new InstantCommand(() -> hang.setTransveral(Value.kForward)));

    // Transversal reverse
    new JoystickButton(driverJoystick, XboxController.Button.kRightBumper.value)
      .whenPressed(new InstantCommand(() -> hang.setTransveral(Value.kReverse)));



    // Gunner Joystick buttons  -----------------------------------------------------------------

    // Toggle Shooter
    final JoystickButton toggleShooter = new JoystickButton(gunnerJoystick, XboxController.Button.kLeftBumper.value);
    toggleShooter.whenHeld(enableShooterCmd);

    
    // Manual Mode Off
    new JoystickButton(gunnerJoystick, 7)
      .whenPressed(new SequentialCommandGroup(
        //new PrintCommand("Set Manual Mode: false"), 
        new InstantCommand(() -> shooter.setManualMode(false))));
      
    // Manual Mode On
    new JoystickButton(gunnerJoystick, 8)
      .whenPressed(new SequentialCommandGroup(
        //new PrintCommand("Set Manual Mode: true"), 
        new InstantCommand(() -> shooter.setManualMode(true))));
    

    // Extend Intake
    final JoystickButton deployIntake = new JoystickButton(gunnerJoystick, XboxController.Button.kB.value);
    deployIntake.whenPressed(deployIntakeCmdGroup);

    // Retract Intake
    new JoystickButton(gunnerJoystick, XboxController.Button.kA.value)
      .whenPressed(new InstantCommand(() -> intake.setPiston(Value.kReverse)));

    
    // Enable Feeder
    final JoystickButton enableFeeder = new JoystickButton(gunnerJoystick, XboxController.Button.kRightBumper.value);
    enableFeeder.whileHeld(enableFeederCmd);
    
    /*
    // Enable DriveWithLimelight
    new JoystickButton(gunnerJoystick, XboxController.Button.kX.value)
      .whenHeld(driveWithLimelightCmd);
    /*
    // Enable feeder Cmd Group (AUTO)
    final JoystickButton enableFeeder = new JoystickButton(gunnerJoystick, XboxController.Button.kRightBumper.value);
    enableFeeder.whileActiveOnce(enableFeederCmdGroup);
    */
  
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //return autonomousCmdGroup;
    //return null;
    return autonomousCmdGroup;
  }
}
