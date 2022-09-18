// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HangConstants;
import frc.robot.Constants.Solenoid;

public class HangSubsystem extends SubsystemBase {
  /** Creates a new HangSubsystem. */
  
  private final CANSparkMax hangLeft;
  private final RelativeEncoder hangLeftEncoder;

  private final CANSparkMax hangRight;
  private final RelativeEncoder hangRightEncoder;

  private final DoubleSolenoid transveral;

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
    hangLeft = new CANSparkMax(Constants.PWM.Hang.LEFT, MotorType.kBrushless);
    hangLeft.restoreFactoryDefaults();
    hangLeft.setInverted(false);

    hangLeftEncoder = hangLeft.getEncoder();

    hangRight = new CANSparkMax(Constants.PWM.Hang.RIGHT, MotorType.kBrushless);
    hangRight.restoreFactoryDefaults();
    hangRight.setInverted(true);

    hangRightEncoder = hangRight.getEncoder();

    transveral = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Solenoid.Hang.LEFT, Solenoid.Hang.RIGHT);
    transveral.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Hang Encoder Position", hangLeftEncoder.getPosition());
    SmartDashboard.putNumber("Right Hang Encoder Position", hangRightEncoder.getPosition());
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

  public void setTransveral(DoubleSolenoid.Value value) {
    transveral.set(value);
  }

  /*
  public void resetEncoders() {
    hangLeft.setSelectedSensorPosition(0);
    hangRight.setSelectedSensorPosition(0);
  }

  public double getHangLeftSelectedSensorPosition() {
    return hangLeft.getSelectedSensorPosition();
  }
  
  public double getHangRightSelectedSensorPosition() {
    return hangRight.getSelectedSensorPosition();
  }
  */

  public void stopMotors() {
    hangLeft.stopMotor();
    hangRight.stopMotor();
  }
}
