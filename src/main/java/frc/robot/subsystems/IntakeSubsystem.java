// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  WPI_TalonSRX intake;

  public IntakeSubsystem() {
    intake = new WPI_TalonSRX(Constants.PWM.Intake.INTAKE);
    intake.setInverted(false);

    intake.setNeutralMode(NeutralMode.Coast);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void intakeWithTriggers(XboxController controller, double speed) {
    intake.set((controller.getRightTriggerAxis() - controller.getLeftTriggerAxis()) * speed);
  }
  /**
   * DO THIS PLEASE!
   * https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
   * --> 
   * https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html#functionalcommand
   */
}
