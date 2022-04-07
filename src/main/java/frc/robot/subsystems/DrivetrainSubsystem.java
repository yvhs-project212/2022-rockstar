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

  /*
  private final Encoder leftEncoder;
  private final Encoder rightEncoder;
  */

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

    /*
    leftEncoder = new Encoder(PWM.Drive.LEFT_ENCODER_A, Drive.LEFT_ENCODER_B, true, EncodingType.k4X);
    // This is set up assuming a 6 inch wheel with a 4096 CPR encoder.
    leftEncoder.setDistancePerPulse(DriveConstants.perEncoderTick2Distance); // (Math.PI * 6) / 4096
    //leftEncoder.reset();

    rightEncoder = new Encoder(PWM.Drive.RIGHT_ENCODER_A, Drive.RIGHT_ENCODER_B, false, EncodingType.k4X);
    // This is set up assuming a 6 inch wheel with a 4096 CPR encoder.
    rightEncoder.setDistancePerPulse(DriveConstants.perEncoderTick2Distance);
    //rightEncoder.reset();
    */


    table = NetworkTableInstance.getDefault().getTable("limelight");
    
    // Drivetrain Meters
    SmartDashboard.putNumber("Drivetrain Distance (Feet)", 0);
    // Left Gearbox Encoder
    SmartDashboard.putNumber("Left Encoder Distance (Inches)", 0);
    // Right Gearbox Encoder
    SmartDashboard.putNumber("Right Encoder Distance (Inches)", 0);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    /*
    SmartDashboard.putNumber("Left Encoder Distance (Inches)", leftEncoder.getDistance());
    SmartDashboard.putNumber("Right Encoder Distance (Inches)", rightEncoder.getDistance());

    SmartDashboard.putNumber("Drivetrain Distance (Feet)", getDrivetrainFeet());
    */
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
  
  /*
  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  public double getLeftEncoderDistance() {
    return leftEncoder.getDistance();
  }

  public double getRightEncoderDistance() {
    return rightEncoder.getDistance();
  }

  public double getDrivetrainFeet() {
    // return the drive distance in feet
    return (getLeftEncoderDistance() + getRightEncoderDistance()) / 12;
  }
  */

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
