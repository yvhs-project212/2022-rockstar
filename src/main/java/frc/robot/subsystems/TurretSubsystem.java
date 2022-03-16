// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PWM;

public class TurretSubsystem extends SubsystemBase {
  /** Creates a new TurretSubsystem. */
  private final WPI_TalonFX turret;
  public NetworkTable table;

  public TurretSubsystem() {
    turret = new WPI_TalonFX(PWM.Turret.TURRET);

    table = NetworkTableInstance.getDefault().getTable("limelight");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void turretWithJoysticks(XboxController controller, double speed) {
    turret.set(controller.getRawAxis(0) * speed);
  }

  public void turretWithLimelight() {
    NetworkTableEntry tx = this.table.getEntry("tx");
    double x = tx.getDouble(0.0);
    double heading_error = -x;
    double steering_adjust = 0.0;

    if (x > 1.0) {
      steering_adjust = -0.015 * heading_error - 0.08;
    }
    else if (x < 1.0) {
      steering_adjust = -0.015 * heading_error + 0.08;
    }
    
    turret.set(steering_adjust);
  }
}
