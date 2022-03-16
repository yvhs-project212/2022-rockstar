// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {
    public static final class PWM {
        public static final class Drive {
            public static final int LEFT_TOP = 1;
			public static final int RIGHT_TOP = 12;
			public static final int LEFT_BOTTOM = 0;
			public static final int RIGHT_BOTTOM = 15;
        }
        public static final class Shooter {
            public static final int BOTTOM_FLYWHEEL = 1;
            public static final int TOP_FLYWEEL = 0;

        }
        public static final class Intake {
            public static final int INTAKE = 103;
        }
        public static final class Hang {
            public static final int LEFT = 104;
            public static final int RIGHT = 105;
        } 
        public static final class Storage {
            public static final int INDEXER = 2;
            public static final int FEEDER = 3;
        } 
        public static final class Turret {
            public static final int TURRET = 10;
        }
    }

    public static final class Solenoid {
        public static final class Drive {
            public static final int GEARBOX_LOW = 0;
            public static final int GEARBOX_HIGH = 1;
        }
        public static final class Intake {
            public static final int LEFT = 2;
            public static final int RIGHT = 3;
        } 
    }

    public static final class OI {
        public static final int DRIVER_NUMBER = 0;
        public static final int GUNNER_NUMBER = 1;

        public static final int XBOX_Y_AXIS = 1;
        public static final int XBOX_X_AXIS = 4; 
    }

    public static final class DriveConstants {
		public static final double FORWARD_SPEED = 0.65;
		public static final double TURN_SPEED = 0.8;
        public static final double MIN_PRESSURE = 30;       //This should be in PSI's - LL
        public static final double MAX_PRESSURE = 60;

        public static final double WHEEL_DIAMETER = 6; // 6 Inches
        /*
        public static final double ENCODER_RESOLUTION = 4096;

        public static final double kDriveTick2Feet = 1.0 / 4096 * 6 * Math.PI / 12; //ADJUST PROPERLY --> Make this work - LL

        public static final LinearSystem<N2, N2, N2> kDrivetrainPlant = null;
        public static final DCMotor kDriveGearbox = null;
        public static final double kDriveGearing = 0;
        public static final double kTrackwidthMeters = 0;
        public static final double kWheelDiameterMeters = 0; 
        public static final double kP = 0.5;
        */
	}
	
	public static final class IntakeConstants {
		public static final double INTAKE_SPEED = 0.5;
	}

    public static final class HangConstants {
		public static final double UP_SPEED = 0.65;
		public static final double DOWN_SPEED = -0.65;
        
	}

    public static final class ShooterConstants {
        public static final double velocityTolerance = 10; // native sensor units per 100ms??? --> https://docs.ctre-phoenix.com/en/stable/ch16_ClosedLoop.html#velocity-closed-loop-control-mode 

        /**
         * Which PID slot to pull gains from. Starting 2018, you can choose from
         * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
         * configuration.
         */
        public static final int kSlotIdx = 0;

        /**
         * Talon FX supports multiple (cascaded) PID loops. For
         * now we just want the primary one.
         */
        public static final int kPIDLoopIdx = 0;

        /**
         * Set to zero to skip waiting for confirmation, set to nonzero to wait and
         * report to DS if action fails.
         */
        public static final int kTimeoutMs = 30;

        public static final double LIMELIGHT_MOUNT_ANGLE_DEGREES = 24;

        public static final double LIMELIGHT_LENS_HEIGHT_INCHES = 38;

        public static final double GOAL_HEIGHT_INCHES = 104;

        /**
         * PID Gains may have to be adjusted based on the responsiveness of control loop.
         * kF: 1023 represents output value to Talon at 100%, 20660 represents Velocity units at 100% output
         * 
         * 	                                    			  kP   	 kI    kD      kF          Iz    PeakOut 

        public final static Gains kGains_Velocit  = new Gains( 0.1, 0.001, 5, 1023.0/20660.0,  300,  1.00);
        */

        public final static class TopFlywheelConstants {
            public final static class kGains_Velocit_Top_Flywheel {
                public final static double kP = 0.1;
                public final static double kI = 0.001;
                public final static double kD = 5;
                public final static double kF = 0; // 1023.0 / 20660.0
                public final static double Iz = 300;
                public final static double PeakOut = 1.00;
            }
            
            public final static double TOP_SLOPE = 88.23;
            public final static double TOP_Y_INT = 6000;
        }
        
        public final static class BottomFlywheelConstants {
            public final static class kGains_Velocit_Bottom_Flywheel {
                public final static double kP = 0.1;
                public final static double kI = 0.001;
                public final static double kD = 5;
                public final static double kF = 0; // 1023.0 / 20660.0
                public final static double Iz = 300;
                public final static double PeakOut = 1.00;
            }
            public final static double BOTTOM_SLOPE = 720.58824;
            public final static double BOTTOM_Y_INT = -900;   
        }
        //public static final double kShooterRPM2Velocity = 1; // Per encoder tick to flywheel velocity (ft/min)

    }
    public static final class StorageConstants {
        public static final double INDEXER_SPEED = 0.5;
        public static final double FEEDER_SPEED = 0.5;

    }

	public static final class AutonomousConstants{
		public static final double DRIVE_FORWARD_TIME = 3.0;
		public static final double AUTONOMOUS_SPEED = 0.6;

        
	}

    public static final Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
}
