// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class DrivetrainSubsystem extends SubsystemBase {
  /** Creates a new DrivetrainSubsystem. */
  private final WPI_TalonSRX leftTopLeader;
  private final WPI_TalonSRX rightTopLeader;
  private final WPI_TalonSRX leftBottomFollower;
  private final WPI_TalonSRX rightBottomFollower;

  private final DifferentialDrive drive;
  
  private final DoubleSolenoid gearbox;

  public NetworkTable table;


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

    gearbox.set(Value.kForward);

    table = NetworkTableInstance.getDefault().getTable("limelight");
    
    // Drivetrain Meters
    SmartDashboard.putNumber("Drivetrain Meters", 0);
    // Left Gearbox Encoder
    SmartDashboard.putNumber("Left Gearbox Encoder", 0);
    // Right Gearbox Encoder
    SmartDashboard.putNumber("Right Gearbox Encoder", 0);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("Air Tank Presusre", getPressure());
    
  }
  public void driveWithJoysticks(XboxController controller, double forwardSpeed, double turnSpeed) {
    double forward = ((controller.getRightTriggerAxis() - 
    controller.getLeftTriggerAxis())*forwardSpeed);

    drive.arcadeDrive(forward, controller.getRawAxis(Constants.OI.XBOX_X_AXIS)*turnSpeed);
    //drive.arcadeDrive(controller.getRawAxis(Constants.OI.XBOX_Y_AXIS)*-(forwardSpeed), controller.getRawAxis(Constants.OI.XBOX_X_AXIS)*turnSpeed);
  }

  public void setMotors(double left, double right) {
    drive.tankDrive(left, right);
  }

  public double getEncoderMeters(double leftEncoder, double rightEncoder) {
    return (leftEncoder + -rightEncoder) / 2 * DriveConstants.kEncoderTick2Meter;
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

  /*
  public void resetEncoders() {
    RobotContainer.storage.resetEncoder();
    RobotContainer.hang.resetEncoder();
  }
  */

  public double getPressure() {
    return Constants.pcmCompressor.getPressure();
  }

  public double getCurrent() {
    return Constants.pcmCompressor.getCurrent();
  }

  public void driveWithLimelight() {
    
    NetworkTableEntry tx = table.getEntry("tx");
    double x = tx.getDouble(0.0);
    double heading_error = -x;
    double steering_adjust = 0.0;

    if (x > 1.0) {
      steering_adjust = -DriveConstants.kP * heading_error - DriveConstants.MIN_COMMAND; // -0.015 and 0.08
    }
    else if (x < 1.0) {
      steering_adjust = -DriveConstants.kP * heading_error + DriveConstants.MIN_COMMAND;
    }
    
    drive.arcadeDrive(0, steering_adjust);
  }
}
