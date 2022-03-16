// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {
  /** Creates a new DrivetrainSubsystem. */
  private final WPI_TalonSRX leftTopLeader;
  private final WPI_TalonSRX rightTopLeader;
  private final WPI_TalonSRX leftBottomFollower;
  private final WPI_TalonSRX rightBottomFollower;

  private final DifferentialDrive drive;
  
  private final DoubleSolenoid gearbox;

  // gyro
  
  public DrivetrainSubsystem() {
    leftTopLeader = new WPI_TalonSRX(Constants.PWM.Drive.LEFT_TOP);
    leftTopLeader.setInverted(true);
    rightTopLeader = new WPI_TalonSRX(Constants.PWM.Drive.RIGHT_TOP);
    rightTopLeader.setInverted(false);
    leftBottomFollower = new WPI_TalonSRX(Constants.PWM.Drive.LEFT_BOTTOM);
    leftBottomFollower.setInverted(true);
    rightBottomFollower = new WPI_TalonSRX(Constants.PWM.Drive.RIGHT_BOTTOM);
    rightBottomFollower.setInverted(false);

    leftBottomFollower.follow(leftTopLeader);
    rightBottomFollower.follow(rightTopLeader);

    drive = new DifferentialDrive(leftTopLeader, rightTopLeader);

    gearbox = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Solenoid.Drive.GEARBOX_LOW, Constants.Solenoid.Drive.GEARBOX_HIGH);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void driveWithJoysticks(XboxController controller, double forwardSpeed, double turnSpeed) {
    drive.arcadeDrive(controller.getRawAxis(Constants.OI.XBOX_Y_AXIS)*-(forwardSpeed), controller.getRawAxis(Constants.OI.XBOX_X_AXIS)*turnSpeed);
  }

  public void setGear(DoubleSolenoid.Value value) {
    gearbox.set(value);
  }

  public void stop() {
    drive.stopMotor();
  }

  public void setFalconMode(NeutralMode neutralMode) {
    leftBottomFollower.setNeutralMode(neutralMode);
    leftTopLeader.setNeutralMode(neutralMode);
    rightBottomFollower.setNeutralMode(neutralMode);
    rightTopLeader.setNeutralMode(neutralMode);   
  }

  public double getLeftEncoder() {
    return 0;
  }

  public double getRightEncoder() {
    return 0;
  }

  public double getPressure() {
    return Constants.pcmCompressor.getPressure();
  }

  public double getCurrent() {
    return Constants.pcmCompressor.getCurrent();
  }
}
