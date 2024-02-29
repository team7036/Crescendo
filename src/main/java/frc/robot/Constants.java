package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

public interface Constants {

    public interface Controllers {
        int DRIVER = 0;
    }

    public interface Drivetrain {

        public interface Swerve {
            double WHEEL_DIAMETER = 0.102;
            double GEAR_RATIO = 6.75;
            double MAX_DRIVE_SPEED = 3.0;
            double MAX_ANGULAR_VELOCITY = Math.PI;
            double MAX_ANGULAR_ACCELERATION = Math.PI;
            double POSITION_CONVERSION_FACTOR = Math.PI * WHEEL_DIAMETER / GEAR_RATIO;
            double VELOCITY_CONVERSION_FACTOR = POSITION_CONVERSION_FACTOR / 60;
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
            DISABLED,
            TEST,
            INTAKE,
            AUTO_AIM,
            MANUAL_AIM,
            FIRE,
            IDLE
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
                double kI = 5;
                double kD = 1;    
            }
            float MAX_ANGLE = (float) Math.toRadians(200);
            float MIN_ANGLE = 0;
            double MAX_VELOCITY = 2 * Math.PI;
            double MAX_ACCELERATION = 2 * Math.PI;
            double POSITION_CONVERSION = 0.0293;
            double VELOCITY_CONVERSION = 0.0;
        }

        double SPEED_CONVERSION = 10/18;
        double INTAKE_ANGLE = 0.2;
        double INTAKE_SPEED = -100;
    }
}
