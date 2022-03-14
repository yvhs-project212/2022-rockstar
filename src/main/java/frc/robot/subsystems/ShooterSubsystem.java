// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  WPI_TalonFX bottomFlywheel;
  WPI_TalonFX topFlywheel;

  NetworkTable table;
  Boolean shooterOnOff;


  public ShooterSubsystem() {
    bottomFlywheel = new WPI_TalonFX(Constants.PWM.Shooter.BOTTOM_FLYWHEEL);
    bottomFlywheel.setInverted(false);
    bottomFlywheel.setNeutralMode(NeutralMode.Coast);
    
    topFlywheel = new WPI_TalonFX(Constants.PWM.Shooter.TOP_FLYWEEL);
    topFlywheel.setInverted(true);
    topFlywheel.setNeutralMode(NeutralMode.Coast);

    // table = new NetworkTableInstance.getDefault().getTable("limelight"));

    shooterOnOff = false;

    SmartDashboard.putNumber("Target Top Flywheel Velocity (Input)", 0);
    SmartDashboard.putNumber("Target Bottom Flywheel Velocity (Input)", 0);

    SmartDashboard.putNumber("Top Flywheel Velocity Output", 0);
    SmartDashboard.putNumber("Bottom Flywheel Velocity Output", 0);
    
    SmartDashboard.putBoolean("Goal Detected", false);

    SmartDashboard.putNumber("Distance from Goal (Inches)", 0);
    SmartDashboard.putNumber("Distance from Goal (Feet)", 0);

    SmartDashboard.putBoolean("Bottom Flywheel at Setpoint ", false);
    SmartDashboard.putBoolean("Top Flywheel at Setpoint ", false);


    
    /* Factory Default all hardware to prevent unexpected behaviour */
		bottomFlywheel.configFactoryDefault();
    topFlywheel.configFactoryDefault();
		
		/* Config neutral deadband to be the smallest possible */
    bottomFlywheel.configNeutralDeadband(0.001);
    topFlywheel.configNeutralDeadband(0.001);

		/* Config sensor used for Primary PID [Velocity] */
    bottomFlywheel.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, ShooterConstants.kPIDLoopIdx,ShooterConstants.kTimeoutMs);
    topFlywheel.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, ShooterConstants.kPIDLoopIdx,ShooterConstants.kTimeoutMs);

		/* Config the peak and nominal outputs */
		bottomFlywheel.configNominalOutputForward(0, ShooterConstants.kTimeoutMs);
		bottomFlywheel.configNominalOutputReverse(0, ShooterConstants.kTimeoutMs);
		bottomFlywheel.configPeakOutputForward(1, ShooterConstants.kTimeoutMs);
		bottomFlywheel.configPeakOutputReverse(-1, ShooterConstants.kTimeoutMs);

    topFlywheel.configNominalOutputForward(0, ShooterConstants.kTimeoutMs);
		topFlywheel.configNominalOutputReverse(0, ShooterConstants.kTimeoutMs);
		topFlywheel.configPeakOutputForward(1, ShooterConstants.kTimeoutMs);
		topFlywheel.configPeakOutputReverse(-1, ShooterConstants.kTimeoutMs);

		/* Config the Velocity closed loop gains in slot0 */
		bottomFlywheel.config_kF(ShooterConstants.kPIDLoopIdx, ShooterConstants.BottomFlywheelConstants.kGains_Velocit_Bottom_Flywheel.kF, ShooterConstants.kTimeoutMs);
		bottomFlywheel.config_kP(ShooterConstants.kPIDLoopIdx, ShooterConstants.BottomFlywheelConstants.kGains_Velocit_Bottom_Flywheel.kP, ShooterConstants.kTimeoutMs);
		bottomFlywheel.config_kI(ShooterConstants.kPIDLoopIdx, ShooterConstants.BottomFlywheelConstants.kGains_Velocit_Bottom_Flywheel.kI, ShooterConstants.kTimeoutMs);
		bottomFlywheel.config_kD(ShooterConstants.kPIDLoopIdx, ShooterConstants.BottomFlywheelConstants.kGains_Velocit_Bottom_Flywheel.kD, ShooterConstants.kTimeoutMs);

    topFlywheel.config_kF(ShooterConstants.kPIDLoopIdx, ShooterConstants.TopFlywheelConstants.kGains_Velocit_Top_Flywheel.kF, ShooterConstants.kTimeoutMs);
		topFlywheel.config_kP(ShooterConstants.kPIDLoopIdx, ShooterConstants.TopFlywheelConstants.kGains_Velocit_Top_Flywheel.kP, ShooterConstants.kTimeoutMs);
		topFlywheel.config_kI(ShooterConstants.kPIDLoopIdx, ShooterConstants.TopFlywheelConstants.kGains_Velocit_Top_Flywheel.kI, ShooterConstants.kTimeoutMs);
		topFlywheel.config_kD(ShooterConstants.kPIDLoopIdx, ShooterConstants.TopFlywheelConstants.kGains_Velocit_Top_Flywheel.kD, ShooterConstants.kTimeoutMs);
		/*
		 * Talon FX does not need sensor phase set for its integrated sensor
		 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
		 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
		 * 
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
		 */
        // _talon.setSensorPhase(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // To always know the velocity of the flywheels
    printEncoderVelocity();

    // To always know the distance of the goal
    getLimelightDistanceInches();
    // To always know if there is a goal 
    goalDetected();
    // To know if the top flywheel is at the setpoint velocity
    topFlywheelAtSetpoint();
    // To know if the bottom flywheel is at the setpoint velocity
    bottomFlywheelAtSetpoint();
  }

  public void enable() {
    /* Velocity Closed Loop */

    /**
     * Convert 2000 RPM to units / 100ms.
     * 2048 Units/Rev * 2000 RPM / 600 100ms/min in either direction:
     * velocity setpoint is in units/100ms
     */

    /** How to calculate the RPM of a flywheel for a specific velocity
     *  "It’s not too much physics. If we ignore slippage, 
     * the ball moves at the same linear speed as the surface of your wheels 
     * (assuming by 2 flywheels you mean the ball is between them). 
     * So if your wheels have a circumference of 12.5” and 
     * are spinning at 5000 rpms:"
     * 
     * 5000 rpms * 12.5 in / 12 in/ft / 60 sec/min = 87.27 ft/sec
     * 
     *  "You can rearrange to get rpms out for a given ft/sec desired"
     * 
     * X ft/sec * 60 sec/min * 12 in/ft / circum inches = rpms
     * 
     * https://www.reddit.com/r/FRC/comments/s7hn82/help_with_velocity_to_rpm_calculations/
     */
    shooterOnOff = true;

    //double targetVelocity_UnitsPer100ms = leftYstick * 2000.0 * 2048.0 / 600.0;
    //double bottomFlywheel_targetVelocity_UnitsPer100ms = SmartDashboard.getNumber("Top Flywheel Velocity Input", 0);
    //double topFlywheel_targetVelocity_UnitsPer100ms = SmartDashboard.getNumber("Top Flywheel Velocity Input", 0);
    
    //double bottomFlywheel_targetVelocity_UnitsPer100ms = 88.23 * (getLimelightDistance() / 12.0) + 6000.0;
    //double topFlywheel_targetVelocity_UnitsPer100ms = 720.58824 * (getLimelightDistance() / 12.0) - 900.0;

    /* 2000 RPM in either direction */

    //bottomFlywheel.set(ControlMode.Velocity, bottomFlywheel_targetVelocity_UnitsPer100ms);
    //topFlywheel.set(ControlMode.Velocity, topFlywheel_targetVelocity_UnitsPer100ms);

    bottomFlywheel.set(ControlMode.Velocity, getTargetBottomFlyWheelVelocity());
    topFlywheel.set(ControlMode.Velocity, getTargetTopFlyWheelVelocity());
  }

  public double getTargetBottomFlyWheelVelocity() {
    double m = ShooterConstants.BottomFlywheelConstants.BOTTOM_SLOPE;
    double b = ShooterConstants.BottomFlywheelConstants.BOTTOM_Y_INT;
    double bottomFlywheel_targetVelocity_UnitsPer100ms = (m * (getLimelightDistanceInches() / 12.0)) + (b);
    
    SmartDashboard.putNumber("Target Bottom Flywheel Velocity (Input)", 0);
    return bottomFlywheel_targetVelocity_UnitsPer100ms;
  }

  public double getTargetTopFlyWheelVelocity() {
    double m = ShooterConstants.TopFlywheelConstants.TOP_SLOPE;
    double b = ShooterConstants.TopFlywheelConstants.TOP_Y_INT;
    double topFlywheel_targetVelocity_UnitsPer100ms = (m * (getLimelightDistanceInches() / 12.0)) + (b);

    
    SmartDashboard.putNumber("Target Bottom Flywheel Velocity (Input)", 0);
    return topFlywheel_targetVelocity_UnitsPer100ms;
  }

  public void disable() {
    shooterOnOff = false;
    bottomFlywheel.stopMotor();
    topFlywheel.stopMotor();
    SmartDashboard.putNumber("Target Top Flywheel Velocity (Input)", getTargetTopFlyWheelVelocity() );
    SmartDashboard.putNumber("Target Bottom Flywheel Velocity (Input)", getTargetBottomFlyWheelVelocity());
  }

  public boolean goalDetected () {
    NetworkTableEntry tv = table.getEntry("tv");
    double validTargets = tv.getDouble(0.0);
        if (validTargets == 1.0) {
          SmartDashboard.putBoolean("Goal Detected", true);
          return true;
        } else {
          SmartDashboard.putBoolean("Goal Detected", false);
          return false;
        }
  }

  public double getLimelightDistanceInches() {
    NetworkTableEntry ty = table.getEntry("ty");
    double targetOffsetAngle_Vertical = ty.getDouble(0.0);

    double limelightMountAngleDegrees = ShooterConstants.LIMELIGHT_MOUNT_ANGLE_DEGREES; // 24 degrees  
    double limelightLensHeightInches = ShooterConstants.LIMELIGHT_LENS_HEIGHT_INCHES;  // 38 inches
    double goalHeightInches = ShooterConstants.GOAL_HEIGHT_INCHES;          // 104 inches

    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * 0.017453277777777776;

    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
    double distanceFromLimelightToGoalFeet = distanceFromLimelightToGoalInches / 12.0;

    SmartDashboard.putNumber("Distance from Goal (Inches)", distanceFromLimelightToGoalInches);
    SmartDashboard.putNumber("Distance from Goal (Feet)", distanceFromLimelightToGoalFeet);

    return distanceFromLimelightToGoalInches;
  }

    
  public double getBottomFlywheelVelocity() {
    return bottomFlywheel.getSelectedSensorVelocity();
  }
  
  public double getTopFlywheelVelocity() {
    return topFlywheel.getSelectedSensorVelocity();
  }

  public void printEncoderVelocity() {
    SmartDashboard.putNumber("Top Flywheel Velocity Output", topFlywheel.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Bottom Flywheel Velocity Output", bottomFlywheel.getSelectedSensorVelocity());
  }

  public Boolean bottomFlywheelAtSetpoint() {
    if (getBottomFlywheelVelocity() < getBottomFlywheelVelocity() + ShooterConstants.velocityTolerance 
    && getBottomFlywheelVelocity() > getBottomFlywheelVelocity() - ShooterConstants.velocityTolerance) {
      SmartDashboard.putBoolean("Bottom Flywheel at Setpoint ", true);
      return true;
    } else {
      SmartDashboard.putBoolean("Bottom Flywheel at Setpoint ", false);
      return false;
    }
  }

  public Boolean topFlywheelAtSetpoint() {
    if (getTopFlywheelVelocity() < getTopFlywheelVelocity() + ShooterConstants.velocityTolerance 
    && getTopFlywheelVelocity() > getTopFlywheelVelocity() - ShooterConstants.velocityTolerance) {
      SmartDashboard.putBoolean("Bottom Flywheel at Setpoint ", true);
      return true;
    } else {
      SmartDashboard.putBoolean("Bottom Flywheel at Setpoint ", false);
      return false;
    }
  }
  public void stopMotors() {
    topFlywheel.stopMotor();
    bottomFlywheel.stopMotor();
  }

}
