package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

public interface Constants {

    public enum Autonomous {
        DO_NOTHING,
        SIMPLE_FORWARD,
        TEST
    }

    public interface Field {

        public interface Speaker {
            public double SPEAKER_BOTTOM_HEIGHT = 1.98;
            public double SPEAKER_TOP_HEIGHT = 2.11;
            public double SPEAKER_CENTER_HEIGHT = ( SPEAKER_BOTTOM_HEIGHT + SPEAKER_TOP_HEIGHT ) / 2;
            public double SUBWOOFER_TO_WALL = 0.92;
        }
        public double distanceFromBottomTargetToSpeaker = 0.92;
        // 49 to bottom of april tag, 54.5 to middle, 78 in to bottom of speaker
        // Bottom of Speaker: 1.981 m
        public double aprilTagLength = 0.1651; // in meters
        public double aprilTagDistanceFromGround = 1.368552 + (aprilTagLength / 2); // in meters, this is from the center of the target
        public double speakerDistanceFromGround = 1.981; // 1.981
        public double speakerXOffsetFromAprilTag = 0.13;
        public double fieldWidth = 8.21; // in meters, width is always the shorter side
        public double fieldLength = 16.54; // in meters, length is always the longer side

    }

    public enum RobotState {
        /*TODO */
    }

    public interface Vision {
    
        public enum LedMode {
            PIPELINE, OFF, BLINK, ON
        }
    
        public enum CameraMode {
            VISION, CAMERA
        }
    
        public enum Pipeline {
            _0, _1, _2, _3, _4, _5, _6, _7, _8, _9
        }
    
        public enum Stream {
            STANDARD, SECONDARY, PRIMARY
        }
    
        public enum BotPoseOptions {
            WPIBLUE,
            WPIRED,
            TARGETSPACE
        }
    
        public enum PoseOptions {
            ROBOTSPACE,
            TARGETSPACE
        }
    }

    public interface Controllers {
        int DRIVER = 0;
        int OPERATOR = 1;
    }

    public interface Drivetrain {

        double MAX_DRIVE_SPEED = 4.0;
        double MAX_TURN_SPEED = 2 * Math.PI;
        double SLOW_DRIVE_SPEED = 1.0;
        double SLOW_TURN_SPEED = MAX_TURN_SPEED / ( MAX_DRIVE_SPEED / SLOW_DRIVE_SPEED );

        public interface Swerve {

            public interface Drive {

                double GEAR_RATIO = 6.75;
                double WHEEL_DIAMETER = 0.102;
                double POSITION_CONVERSION = Math.PI * WHEEL_DIAMETER / GEAR_RATIO ; // Rotations to m
                double VELOCITY_CONVERSION = POSITION_CONVERSION / 60; // RPM to m/s

                public interface PID {
                    double kP = 1.2;
                    double kI = 0;
                    double kD = 0.001;
                }

                public interface Feedforward {
                    double kS = 0.5; // volts
                    double kV = 2.3; // volts / velocity
                }

            }
            public interface Turn {

                double MAX_VELOCITY = 2 * Math.PI;
                double MAX_ACCELERATION = 2 * Math.PI;

                public interface PID {
                    double kP = 12;
                    double kI = 0;
                    double kD = 0.01;
                }
                public interface Feedforward {
                    double kS = 0; // Volts
                    double kV = 0; // Volts/Velocity
                }
            }
        }

        public interface Translation {
            Translation2d FRONT_LEFT = new Translation2d(0.63/2, -0.63/2);
            Translation2d FRONT_RIGHT = new Translation2d(0.63/2, 0.63/2);
            Translation2d BACK_LEFT = new Translation2d(-0.63/2, -0.63/2);
            Translation2d BACK_RIGHT = new Translation2d(-0.63/2, 0.63/2);
        }

        public interface Ports {
            int FL_DRIVE = 22;
            int FL_TURN = 23;
            int FL_ENCODER = 30;
            int FR_DRIVE = 16;
            int FR_TURN = 17;
            int FR_ENCODER = 31;
            int BL_TURN = 21;
            int BL_DRIVE = 20;
            int BL_ENCODER = 32;
            int BR_TURN = 19;
            int BR_DRIVE = 18;
            int BR_ENCODER = 33;
        }
    }

    public interface Intake {

        public interface Ports {
            int MOTOR = 40;
            int LOWER_SENSOR = 2;
            int UPPER_SENSOR = 1;
        }

        double INTAKE_SPEED = -0.5;

    }

    public interface Climber {
        public interface Ports {
            int BOTTOM_MOTOR = 6;
            int TOP_MOTOR = 50;
        }

        public enum Mode {
            CLIMBER_DOWN,
            CLIMBER_UP,
            COAST,
            IDLE
        }

        // 30 cm -> 63.28 Rotations

        double TOP_POSITION_CONVERSION = 0.30/63.28;
        float TOP_REVERSE_LIMIT = (float) 0;
        float TOP_FORWARD_LIMIT = (float) 0.3;
        double TOP_UP_SPEED = 0.15;
        double TOP_DOWN_SPEED = -0.15;

        double BOTTOM_POSITION_CONVERSION = 0.3/325;
        float BOTTOM_REVERSE_LIMIT = (float) 0;
        float BOTTOM_FORWARD_LIMIT = (float) 0.3;
        double BOTTOM_UP_SPEED = 1;
        double BOTTOM_DOWN_SPEED = 0;
        
    }

    public interface Shooter {

        public enum Mode {
            INTAKING,
            MANUAL_AIMING,
            IDLE,
            AMP_FIRING,
            AMP_AIM,
            SPEAKER_AIM,
            SPEAKER_AIM_MANUAL,
            SPEAKER_FIRING,
            TEST_INTAKING,
            TEST_REV,
            TEST_FIRE,
            TEST_IDLE,
        }

        public interface Ports {
            int TOP_MOTOR = 60;
            int BOTTOM_MOTOR = 61;
            int ARM_MOTOR = 55;
            int STAGING_SERVO = 5;
            int SENSOR = 2;
        }

        public interface StagingServo {
            double INTAKE_SPEED = 180;
            double FIRE_SPEED = 0;
            double STOP_SPEED = 90;
        }

        public interface Arm {

            public interface PID {
                double kP = 40;
                double kI = 0;
                double kD = 1;
            }
            public interface Feedforward {
                double kS = 12; // Static Gain
                double kG = 0.4; // Gravity Gain
                double kV = 7.29; // Velocity Gain, Volts * Seconds / Radian
                double kA = 0; // Acceleration Gain
            }

            double PIVOT_POINT_HEIGHT = 0.61;
            double ANGLE_OFFSET = 1.15;
            double SPEAKER_ANGLE_SHORT = 1.8; // 1 Meter
            double AMP_ANGLE = Math.PI;
            double INTAKE_ANGLE = 0.1;
            float MAX_ANGLE = (float) AMP_ANGLE;
            float MIN_ANGLE = (float) INTAKE_ANGLE;
            double MAX_VELOCITY = 2 * Math.PI;
            double MAX_ACCELERATION = 2 * Math.PI;
            double POSITION_CONVERSION = 0.0293;
            double VELOCITY_CONVERSION = 0.0;
        }

        public interface FlyWheels {

            double SPEED_CONVERSION = 10/18;
            double SPEAKER_SPEED = 2500;
            double AMP_SPEED = 50;
            double INTAKE_SPEED = -50;

            public enum Mode {
                INTAKING,
                AMP_FIRE,
                SPEAKER_FIRE,
                IDLE
            }
        }
    }
}
