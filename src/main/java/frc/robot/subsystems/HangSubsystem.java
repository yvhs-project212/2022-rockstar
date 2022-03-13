// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HangConstants;

public class HangSubsystem extends SubsystemBase {
  /** Creates a new HangSubsystem. */
  
  WPI_TalonSRX hangLeft;
  WPI_TalonSRX hangRight;

  public enum Direction {
    UP(HangConstants.UP_SPEED), DOWN(HangConstants.DOWN_SPEED), STOP(0);

    private Direction(final double direction) {
      this.direction = direction;
    }
    private double direction;

    public double getDirection() {
      return direction;
    }
  }
  
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
  public void hangWithPOV(XboxController controller) {
    if ((controller.getPOV() == 0)) {               // Both Up
      hangLeft.set(Direction.UP.getDirection());
      hangRight.set(Direction.UP.getDirection());
    } else if (controller.getPOV() == 180) {        // Both Down
      hangLeft.set(Direction.DOWN.getDirection());
      hangRight.set(Direction.DOWN.getDirection());
    } else if (controller.getPOV() == 45) {         // Right Up  
      hangLeft.set(Direction.STOP.getDirection());
      hangRight.set(Direction.UP.getDirection());
    } else if (controller.getPOV() == 135) {        // Right Down
      hangLeft.set(Direction.STOP.getDirection());
      hangRight.set(Direction.DOWN.getDirection());
    } else if (controller.getPOV() == 225) {        // Left Down
      hangLeft.set(Direction.DOWN.getDirection());
      hangRight.set(Direction.STOP.getDirection());
    } else if (controller.getPOV() == 315) {        // Left Up
      hangLeft.set(Direction.UP.getDirection());
      hangRight.set(Direction.STOP.getDirection());
    } else {                                        // Both Stop
      hangLeft.set(Direction.STOP.getDirection());
      hangRight.set(Direction.STOP.getDirection());
    }
  }
  public void stop() {
    hangLeft.stopMotor();
    hangRight.stopMotor();
  }
}
