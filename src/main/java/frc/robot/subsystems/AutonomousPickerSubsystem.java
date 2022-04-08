// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutonomousPickerSubsystem extends SubsystemBase {
  /** Creates a new AutonomousPickerSubsystem. */
  
  public enum NumberOfBalls {
    NONE, ONE, TWO
  }

  public NumberOfBalls startPosition = NumberOfBalls.NONE;

  private SendableChooser<NumberOfBalls> numberOfBallsChooser;

  public AutonomousPickerSubsystem() {
    System.out.println("In auto picker");

    numberOfBallsChooser = new SendableChooser<>();
    numberOfBallsChooser.setDefaultOption("None", NumberOfBalls.NONE);
    numberOfBallsChooser.addOption("One", NumberOfBalls.ONE);
    numberOfBallsChooser.addOption("Two", NumberOfBalls.TWO);

    SmartDashboard.putData("Number of Balls", numberOfBallsChooser);
  }

  public NumberOfBalls getAutonomous() {
    return numberOfBallsChooser.getSelected(); //This is neccessary! - LL 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
