// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PWM;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {
  /** Creates a new TurretSubsystem. */
  private final WPI_TalonFX turret;
  public NetworkTable table;

  public TurretSubsystem() {
    turret = new WPI_TalonFX(PWM.Turret.TURRET);

    table = NetworkTableInstance.getDefault().getTable("limelight");

    turret.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void turretWithJoysticks(XboxController controller, double speed) {
    turret.set(controller.getRawAxis(0) * speed);

    // Negative = left/CCW
    // Positive = right/CW
  }

  public void turretWithLimelight() {
    NetworkTableEntry tx = table.getEntry("tx");
    double x = tx.getDouble(0.0);
    double heading_error = x;
    double steering_adjust = 0.0;
    
    // To the right
    if (x > 0) {
      // Turn to the right
      steering_adjust = TurretConstants.kP * heading_error + TurretConstants.MIN_COMMAND; // -0.015 and 0.08
    }
    // To the left
    else if (x < 0) {
      // Turn to the left
      steering_adjust = TurretConstants.kP * heading_error - TurretConstants.MIN_COMMAND;
      
    }
    
    //turret.set(steering_adjust);
    turret.set(steering_adjust);
  }

  public void stopMotors() {
    turret.stopMotor();
  }

  public void turretWithPID() {
    /**    Andrew try to find a code and paste it into here
     *    LL - 3.29.22
     * Andy was here
     *  
    */ 

  }
}
