// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HangSubsystem extends SubsystemBase {
  /** Creates a new HangSubsystem. */
  
  WPI_TalonSRX hangLeft;
  WPI_TalonSRX hangRight;

  
  public HangSubsystem() {  
    hangLeft = new WPI_TalonSRX(Constants.PWM.Hang.LEFT);
    hangLeft.setInverted(false);
    hangRight = new WPI_TalonSRX(Constants.PWM.Hang.RIGHT);
    hangRight.setInverted(true);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void hangWithPOV(XboxController controller, double [] speed) {
    if ((controller.getPOV() == 0)) {
      hangLeft.set(speed[0]);
      hangRight.set(speed[0]);
    } else if (controller.getPOV() == 180) {
      hangLeft.set(speed[1]);
      hangRight.set(speed[1]);
    } else {
      hangLeft.set(speed[2]);
      hangRight.set(speed[2]);
    }
  }
}
