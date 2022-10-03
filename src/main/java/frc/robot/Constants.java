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

            public static final int LEFT_ENCODER_A = 1;
            public static final int LEFT_ENCODER_B = 0;

            public static final int RIGHT_ENCODER_A = 3;
            public static final int RIGHT_ENCODER_B = 2;
        }
        public static final class Shooter {
            public static final int BOTTOM_FLYWHEEL = 13;
            public static final int TOP_FLYWEEL = 3;

        }
        public static final class Intake {
            public static final int INTAKE = 8;
        }
        public static final class Hang {
            public static final int LEFT = 2;
            public static final int RIGHT = 14;
        } 
        public static final class Storage {
            public static final int INDEXER = 11;
            public static final int FEEDER = 4;
            
            public static final int BOTTOM_BEAM_BREAK = 4;

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
        public static final class Hang {
            public static final int LEFT = 4;
            public static final int RIGHT = 5;
        }
    }

    public static final class OI {
        public static final int DRIVER_NUMBER = 0;
        public static final int GUNNER_NUMBER = 1;

        public static final int XBOX_Y_AXIS = 1;
        public static final int XBOX_X_AXIS = 0; 
    }

    public static final class DriveConstants {
		public static final double FORWARD_SPEED = 0.8;
		public static final double TURN_SPEED = 0.8;
        public static final double MIN_PRESSURE = 30;       //This should be in PSI's - LL
        public static final double MAX_PRESSURE = 60;

        public static final double WHEEL_DIAMETER = 6; // 6 Inches

        public static final double AUTO_LEFT_DRIVE_FORWARD_SPEED = 0.4;
        public static final double AUTO_RIGHT_DRIVE_FORWARD_SPEED = 0.4;

        
        public static final double AUTO_LEFT_DRIVE_TURN_SPEED = 0.6;
        public static final double AUTO_RIGHT_DRIVE_TURN_SPEED = 0.6;
        /**
         * 1 / CPR * Diameter Wheel (m) * PI
         * 
         * Diameter * PI = Circumpherenece 
         */

        public static final double perEncoderTick2Distance  = (6 * Math.PI) / (12/36) * (30/54) * (4096) ; // distance per turn / ticks per turn
        
        public static final double kP = 0.29;
        public static final double MIN_COMMAND = 0.25;
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
        public static final double INTAKE_DEPLOY_TIME = 0.3; // seconds
	}

    public static final class HangConstants {
		public static final double UP_SPEED = 1;
		public static final double DOWN_SPEED = -1;
        
	}

    public static final class ShooterConstants {
        public static final double velocityTolerance = 500; // native sensor units per 100ms??? --> https://docs.ctre-phoenix.com/en/stable/ch16_ClosedLoop.html#velocity-closed-loop-control-mode 

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

        public static final double LIMELIGHT_MOUNT_ANGLE_DEGREES = 22;

        public static final double LIMELIGHT_LENS_HEIGHT_INCHES = 38;

        public static final double GOAL_HEIGHT_INCHES = 104;

        /**
         * PID Gains may have to be adjusted based on the responsiveness of control loop.
         * kF: 1023 represents output value to Talon at 100%, 20660 represents Velocity units at 100% output
         * 
         * 	                                    			  kP   	 kI    kD      kF          Iz    PeakOut 

        public final static Gains kGains_Velocit  = new Gains( 0.1, 0.001, 5, 1023.0/20660.0,  300,  1.00);
        */

        public final static class ShooterVelocities {
            //public final static double AUTO_BOTTOM = 5500;
            //public final static double AUTO_TOP = 8300;

            /**
             * soscistatistics.com/tests/regression/default.aspx
             * 
             * Bottom Line of best Fit
             * 
             * y = mx + b
             * m = 145.76638
             * b = 3976.78068
             * 
             * (6.97, 5000)
             * (9.2, 5300)
             * (10.72, 5550) 
             *
             * 
             * Top Line of Best Fit
             * 
             * y = mx + b
             * m = 226.48484
             * b = 3419.94093
             * 
             * (6.97, 5000)
             * (9.2, 5500)
             * (10.72, 5850) 
             * 
             */
            
            //public final static double BOTTOM = 5000;   // 6.97 feet
            //public final static double TOP = 5000;      // 6.97 feet

            //public final static double BOTTOM = 5300; //9.2 feet
            //public final static double TOP = 5500; // 9.2 feet
            
            //public final static double BOTTOM = 5550;   // 10.72 feet
            //public final static double TOP = 5850;      // 10.72 feet

        
        }

        public final static class TopFlywheelConstants {
            public final static class kGains {
                public final static double kP = 0.01;   // 9.2 feet
                public final static double kI = 0;      // 9.2 feet
                public final static double kD = 0;      // 9.2 feet
                public final static double kF = 0.1;    // 9.2 feet
                public final static double Iz = 300;
                public final static double PeakOut = 1.00;
            }
            
            // VICTOR P.
            /**
             * TOP_SLOPE = 200
             * TOP_Y_INT = 3419.17
             * 
             * y = mx + b
             * 
             * Shooter Velocity = (200)Distance + 3419.17
             * 
             */
            public final static double TOP_SLOPE = 200; 	// 200 (4.9.22 - 12:09 PM)
            public final static double TOP_Y_INT = 3695;	// 3595 (4.9.22 - 12:09 PM)
        }
        
        public final static class BottomFlywheelConstants {
            public final static class kGains {
                public final static double kP = 0.015;  // 0.015 default
                public final static double kI = 0;      // 0.00075 
                public final static double kD = 0;      // 5
                public final static double kF = 0.1;      // 0
                public final static double Iz = 300;
                public final static double PeakOut = 1.00;
            }
            // VICTOR
            public final static double BOTTOM_SLOPE = 194.417;	// 194.417 (4.8.22 - 12:09 PM)
            public final static double BOTTOM_Y_INT = 2725;  // 2675 (4.9.22 - 12:09 PM)
        }
        //public static final double kShooterRPM2Velocity = 1; // Per encoder tick to flywheel velocity (ft/min)
    }
    public static final class StorageConstants {
        
        // When you ARE pressing the FEEDER/SHOOTING
        public static final double INDEXER_SPEED = 0.75;
        public static final double FEEDER_SPEED = 0.7;

        // When you are NOT pressing the FEEDER/SHOOTING
        public static final double INDEXER_SPEED_AUTO = 0.7;
        public static final double FEEDER_SPEED_AUTO = 0.7;

        public static final double TIME_BETWEEN_SHOTS = 0.5; // seconds

        /**
         *  when an object is close the value of the proximity will be large (max 2047 with default
         * settings) and will approach zero when the object is far away
         */
        public static final double MIDDLE_PROXIMITY = 125;
        public static final double TOP_PROXIMITY = 300;
    }

	public static final class TurretConstants{
		public static final double TURRET_SPEED = 0.15;
		public static final double kP = 0.008;          // 0.015
		public static final double MIN_COMMAND = 0.03;   // 0.1

        
	}

    public static final class AutonomousConstants{
		public static final double DRIVE_FORWARD_TIME = 3.0;
		public static final double AUTONOMOUS_SPEED = 0.6;

        public static final double FLYWHEEL_REV_TIME_SECONDS = 2;
        public static final double ONE_BALL_TIMEOUT_SECONDS = 6.5; 

        public static final double TWO_BALL_TIMEOUT_SECONDS = 3.5; 
        public static final double TURRET_TIMEOUT_SECONDS = 0.9;
        
	}

    public static final Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
}
