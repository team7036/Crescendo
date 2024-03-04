package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

public interface Constants {

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
                    double kP = 1;
                    double kI = 0;
                    double kD = 0;
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
            int SENSOR = 3;
        }

    }

    public interface Shooter {

        public enum Mode {
            READY_TO_INTAKE,
            INTAKING,
            AUTO_AIMING,
            MANUAL_AIMING,
            FIRING,
            READY_TO_FIRE,
            IDLE,
            AMP_SCORE,
            AMP_AIM,
            SPEAKER_AIM,
            SPEAKER_SCORE
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

            
            float MAX_ANGLE = 3.5f;
            float MIN_ANGLE = 0;
            double AMP_ANGLE = Math.PI;
            double MAX_VELOCITY = 2 * Math.PI;
            double MAX_ACCELERATION = 2 * Math.PI;
            double POSITION_CONVERSION = 0.0293;
            double VELOCITY_CONVERSION = 0.0;
            double SPEAKER_ANGLE = Math.PI / 2;
        }

        public interface FlyWheels {
            double SPEED_CONVERSION = 10/18;
            double SHOOTING_SPEED = 2500;
        }

        double INTAKE_ANGLE = 0;
        double INTAKE_SPEED = -100;
    }
}
