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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private final WPI_TalonFX bottomFlywheel;
  private final WPI_TalonFX topFlywheel;

  public NetworkTable table;
  public Boolean shooterOnOff;
  public Boolean manualMode;
  public Boolean staticMode;

  public double topFlywheel_targetVelocity_UnitsPer100ms;
  public double bottomFlywheel_targetVelocity_UnitsPer100ms;

  public double adjustableFactor;

  public enum VelocityControlMode {
    MANUAL,
    LIMELIGHT
  }

  VelocityControlMode velocityControlMode;

  public ShooterSubsystem() {
    bottomFlywheel = new WPI_TalonFX(Constants.PWM.Shooter.BOTTOM_FLYWHEEL);
    bottomFlywheel.setInverted(false);
    bottomFlywheel.setNeutralMode(NeutralMode.Coast);
    
    topFlywheel = new WPI_TalonFX(Constants.PWM.Shooter.TOP_FLYWEEL);
    topFlywheel.setInverted(true);
    topFlywheel.setNeutralMode(NeutralMode.Coast);

    table = NetworkTableInstance.getDefault().getTable("limelight");

    shooterOnOff = false;
    manualMode = true;
    staticMode = true;

    topFlywheel_targetVelocity_UnitsPer100ms = 0;
    bottomFlywheel_targetVelocity_UnitsPer100ms = 0;

    adjustableFactor = 0;


    SmartDashboard.putNumber("Target Top Flywheel Velocity", 0);
    SmartDashboard.putNumber("Target Bottom Flywheel Velocity", 0);

    SmartDashboard.putNumber("User-inputed Top Flywheel Velocity", 0);
    SmartDashboard.putNumber("User-inputed Bottom Flywheel Velocity", 0);

    SmartDashboard.putBoolean("Manual mode", false);
    SmartDashboard.putBoolean("Static mode", false);

    SmartDashboard.putNumber("Top Flywheel Velocity Output", 0);
    SmartDashboard.putNumber("Bottom Flywheel Velocity Output", 0);
    
    //SmartDashboard.putBoolean("Goal Detected", false);

    SmartDashboard.putNumber("Distance from Goal (Inches)", 0);
    SmartDashboard.putNumber("Distance from Goal (Feet)", 0);

    
    SmartDashboard.putNumber("Adjustable Factor", 0);
    //SmartDashboard.putBoolean("Bottom Flywheel at Setpoint ", false);
    //SmartDashboard.putBoolean("Top Flywheel at Setpoint ", false);

    /*
    SmartDashboard.putNumber("Top Flywheel kP", 0);
    SmartDashboard.putNumber("Top Flywheel kI", 0);
    SmartDashboard.putNumber("Top Flywheel kD", 0);
    SmartDashboard.putNumber("Top Flywheel kF", 0);

    SmartDashboard.putNumber("Bottom Flywheel kP", 0);
    SmartDashboard.putNumber("Bottom Flywheel kI", 0);
    SmartDashboard.putNumber("Bottom Flywheel kD", 0);
    SmartDashboard.putNumber("Bottom Flywheel kF", 0);
    */
    
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

    bottomFlywheel.config_kF(ShooterConstants.kPIDLoopIdx, ShooterConstants.BottomFlywheelConstants.kGains.kF, ShooterConstants.kTimeoutMs);
    bottomFlywheel.config_kP(ShooterConstants.kPIDLoopIdx, ShooterConstants.BottomFlywheelConstants.kGains.kP, ShooterConstants.kTimeoutMs);
    bottomFlywheel.config_kI(ShooterConstants.kPIDLoopIdx, ShooterConstants.BottomFlywheelConstants.kGains.kI, ShooterConstants.kTimeoutMs);
    bottomFlywheel.config_kD(ShooterConstants.kPIDLoopIdx, ShooterConstants.BottomFlywheelConstants.kGains.kD, ShooterConstants.kTimeoutMs);

    topFlywheel.config_kF(ShooterConstants.kPIDLoopIdx, ShooterConstants.TopFlywheelConstants.kGains.kF, ShooterConstants.kTimeoutMs);
    topFlywheel.config_kP(ShooterConstants.kPIDLoopIdx, ShooterConstants.TopFlywheelConstants.kGains.kP, ShooterConstants.kTimeoutMs);
    topFlywheel.config_kI(ShooterConstants.kPIDLoopIdx, ShooterConstants.TopFlywheelConstants.kGains.kI, ShooterConstants.kTimeoutMs);
    topFlywheel.config_kD(ShooterConstants.kPIDLoopIdx, ShooterConstants.TopFlywheelConstants.kGains.kD, ShooterConstants.kTimeoutMs);
    

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
    // To know if the Shooter is on
    SmartDashboard.putBoolean("Shooter On/Off", shooterOnOff);
    // to know the manual mode 
    SmartDashboard.putBoolean("Manual mode", manualMode);

    // to know the target velocity
    SmartDashboard.putNumber("Target Top Flywheel Velocity", getTargetTopFlyWheelVelocity() );
    SmartDashboard.putNumber("Target Bottom Flywheel Velocity", getTargetBottomFlyWheelVelocity());
  }

  /*
  public void enableButton(double bottomVelocity, double topVelocity) {
    shooterOnOff = true;

    bottomFlywheel.set(TalonFXControlMode.Velocity, bottomVelocity);
    topFlywheel.set(TalonFXControlMode.Velocity, topVelocity);
  }
  */

  public void enable(double bottomTargetVelocity, double topTargetVelocity) {
    
     
    shooterOnOff = true;
  
    bottomFlywheel.set(ControlMode.Velocity, bottomTargetVelocity); // 5000
    topFlywheel.set(ControlMode.Velocity, topTargetVelocity); // 7000
  }

  public void setVelocityControlMode (VelocityControlMode mode) {
    velocityControlMode = mode;
  }

  public VelocityControlMode getVelocityControlMode() {
    return velocityControlMode;
  }

  public double getAdjustableFactor() {
    /*
    if (SmartDashboard.getNumber("Adjustable Factor", 0) != adjustableFactor){
      adjustableFactor = SmartDashboard.getNumber("Adjustable Factor", 0);
    }
    */

    adjustableFactor = SmartDashboard.getNumber("Adjustable Factor", 0);
    
    
    return adjustableFactor;
  }
  public void setTargetBottomFlyWheelVelocity() {
    double m = ShooterConstants.BottomFlywheelConstants.BOTTOM_SLOPE;
    double b = ShooterConstants.BottomFlywheelConstants.BOTTOM_Y_INT;
    
    if (getVelocityControlMode() == VelocityControlMode.MANUAL) {
      bottomFlywheel_targetVelocity_UnitsPer100ms = 
      SmartDashboard.getNumber("User-inputed Bottom Flywheel Velocity", 0);
    } else if (getVelocityControlMode() == VelocityControlMode.LIMELIGHT) {
      bottomFlywheel_targetVelocity_UnitsPer100ms = 
      (m * (getLimelightDistanceInches() / 12.0)) + (b) + getAdjustableFactor();
    } else {
      bottomFlywheel_targetVelocity_UnitsPer100ms = 0;
    }   
  }

  public double getTargetBottomFlyWheelVelocity() {
    return bottomFlywheel_targetVelocity_UnitsPer100ms;
  }
  
  public void setTargetTopFlyWheelVelocity() {
    double m = ShooterConstants.TopFlywheelConstants.TOP_SLOPE;
    double b = ShooterConstants.TopFlywheelConstants.TOP_Y_INT;
    
    
    if (getVelocityControlMode() == VelocityControlMode.MANUAL) {
      topFlywheel_targetVelocity_UnitsPer100ms = 
      SmartDashboard.getNumber("User-inputed Top Flywheel Velocity", 0);
    } else if (getVelocityControlMode() == VelocityControlMode.LIMELIGHT) {
      topFlywheel_targetVelocity_UnitsPer100ms = 
      (m * (getLimelightDistanceInches() / 12.0)) + (b) + getAdjustableFactor();
    } else {
      topFlywheel_targetVelocity_UnitsPer100ms = 0;
    }   
  }

  public double getTargetTopFlyWheelVelocity() {
    return topFlywheel_targetVelocity_UnitsPer100ms;
  }
  

  public void disable() {
    shooterOnOff = false;
    bottomFlywheel.stopMotor();
    topFlywheel.stopMotor();
  }

  public boolean goalDetected () {
    NetworkTableEntry tv = table.getEntry("tv");
    double validTargets = tv.getDouble(0.0);

    boolean cache;

    if (validTargets == 1.0) {
      cache = true;
      /*
      if (cache != SmartDashboard.getBoolean("Goal Detected", false)) {
        SmartDashboard.putBoolean("Goal Detected", cache);
      }
      */
      return true;
    } else {
      cache = false;
      if (cache != SmartDashboard.getBoolean("Goal Detected", false)) {
        SmartDashboard.putBoolean("Goal Detected", cache);
      }
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
    SmartDashboard.putNumber("Top Flywheel Velocity Output", getTopFlywheelVelocity());
    SmartDashboard.putNumber("Bottom Flywheel Velocity Output", getBottomFlywheelVelocity());
  }

  public Boolean bothFlywheelsAtSetpoint() {
    if (bottomFlywheelAtSetpoint() && topFlywheelAtSetpoint()) {
      return true;
    } else {
      return false;
    }
  }
  public Boolean bottomFlywheelAtSetpoint() {
    boolean cache;

    if (getBottomFlywheelVelocity() < getTargetBottomFlyWheelVelocity() + ShooterConstants.velocityTolerance 
    && getBottomFlywheelVelocity() > getTargetBottomFlyWheelVelocity() - ShooterConstants.velocityTolerance) {
      cache = true;
      if (cache != SmartDashboard.getBoolean("Bottom Flywheel at Setpoint", false)) {
        SmartDashboard.putBoolean("Bottom Flywheel at Setpoint", cache);
      }
      return true;
    } else {
      cache = false;
      if (cache != SmartDashboard.getBoolean("Bottom Flywheel at Setpoint", false)) {
        SmartDashboard.putBoolean("Bottom Flywheel at Setpoint", cache);
      }
      return false;
    }
  }

  // Make this quicker
  public Boolean topFlywheelAtSetpoint() {
    boolean cache;

    if (getTopFlywheelVelocity() < getTargetTopFlyWheelVelocity() + ShooterConstants.velocityTolerance 
    && getTopFlywheelVelocity() > getTargetTopFlyWheelVelocity() - ShooterConstants.velocityTolerance) {
      cache = true;
      if (cache != SmartDashboard.getBoolean("Top Flywheel at Setpoint", false)) {
        SmartDashboard.putBoolean("Top Flywheel at Setpoint ", cache);
      }
      return true;
    } else {
      cache = false;
      if (cache != SmartDashboard.getBoolean("Top Flywheel at Setpoint", false)) {
        SmartDashboard.putBoolean("Top Flywheel at Setpoint ", cache);
      }
      return false;
    }
  }
  
  public void setStaticMode(boolean mode) {
    staticMode = mode;
  }
  public boolean getStaticMode() {
    boolean cache = staticMode;

    if (cache != SmartDashboard.getBoolean("Static mode", false)) {
      SmartDashboard.putBoolean("Static mode", cache);
    }
    return staticMode;
  }

  public void setManualMode(boolean manualMode) {
    this.manualMode = manualMode;
  }
  public boolean getManualMode() {
    if (manualMode) {
      SmartDashboard.putBoolean("Manual mode", true);
      return true;
    } else {
      SmartDashboard.putBoolean("Manual mode", false);
      return false;
    }
  }
  

  public void stopMotors() {
    topFlywheel.stopMotor();
    bottomFlywheel.stopMotor();
  }

}
