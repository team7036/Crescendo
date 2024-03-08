package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

public interface Constants {

    public interface Field {
        public double aprilTagLength = 0.1651; // in meters
        public double aprilTagDistanceFromGround = 1.22 + (aprilTagLength / 2); // in meters, this is from the center of the target
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

        double MAX_DRIVE_SPEED = 3.0;
        double MAX_ANGULAR_VELOCITY = 2 * Math.PI;
        double MAX_ANGULAR_ACCELERATION = 2 * Math.PI;
        double SLOW_DRIVE_SPEED = MAX_DRIVE_SPEED / 2;

        public interface Swerve {

            public interface Drive {

                double GEAR_RATIO = 6.75;
                double WHEEL_DIAMETER = 0.102;
                double POSITION_CONVERSION = Math.PI * WHEEL_DIAMETER / GEAR_RATIO ; // Rotations to m
                double VELOCITY_CONVERSION = POSITION_CONVERSION / 60; // RPM to m/s

                public interface PID {
                    double kP = 0.01;
                    double kI = 20;
                    double kD = 0.01;
                }

                public interface Feedforward {
                    double kS = 1; // volts
                    double kV = 2.6; // volts / velocity
                }

            }
            public interface Turn {

                double MAX_VELOCITY = 2 * Math.PI;
                double MAX_ACCELERATION = 2 * Math.PI;

                public interface PID {
                    double kP = 9;
                    double kI = 0.05;
                    double kD = 0.03;
                }
                public interface Feedforward {
                    double kS = 1;
                    double kV = 0.5;
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
            int INTAKE_SENSOR_INPUT = 6;
            int INTAKE_SENSOR_OUTPUT = 5;
            int LOADED_SENSOR = 7;
        }

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
            TEST_IDLE
        }

        public interface Ports {
            int TOP_MOTOR = 61;
            int BOTTOM_MOTOR = 60;
            int ARM_MOTOR = 55;
            int STAGING_SERVO = 5;
            int SENSOR = 2;
        }

        public interface Arm {

            public interface PID {
                double kP = 30;
                double kI = 10;
                double kD = 2;    
            }
            public interface Feedforward {
                double kS = 1;
                double kG = 1;
                double kV = 4.4;
            }

            double AMP_ANGLE = Math.PI;
            double INTAKE_ANGLE = 0;
            float MAX_ANGLE = (float) AMP_ANGLE;
            float MIN_ANGLE = (float) INTAKE_ANGLE;
            double MAX_VELOCITY = 2 * Math.PI;
            double MAX_ACCELERATION = 2 * Math.PI;
            double POSITION_CONVERSION = 0.0293;
            double VELOCITY_CONVERSION = 0.0;
            double SPEAKER_ANGLE_SHORT = 1.538; // 1 Meter
            double SPEAKER_ANGLE_LONG = 1.2801; // 3.3528 Meters
            double INTAKE_SPEED = -100;
        }

        public interface FlyWheels {
            double SPEED_CONVERSION = 10/18;
            double SHOOTING_SPEED = 2500;
        }
    }
}
