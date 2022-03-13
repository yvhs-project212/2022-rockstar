// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveWithJoysticksCmd;
import frc.robot.commands.HangCmd;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HangSubsystem;

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

  // Hang files - LL
  public static HangSubsystem hang = new HangSubsystem();
  private final HangCmd hangCmd = new HangCmd(hang);

  
  // Controller files - LL
  public static XboxController driverJoystick = new XboxController(Constants.OI.DRIVER_NUMBER);
  public static XboxController gunnerJoystick = new XboxController(Constants.OI.GUNNER_NUMBER);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    System.out.println("In the Construtor for RobotContainer");

    //INITALIZE all the things we made - LL

    driveTrain.setDefaultCommand(driveWithJoysticksCmd);   //Drive is always looking to read this command - LL
    hang.setDefaultCommand(hangCmd);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
