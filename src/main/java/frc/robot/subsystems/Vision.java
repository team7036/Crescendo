package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.lang.Math;
import java.util.Arrays;
import java.util.List;

import frc.robot.Constants;
import frc.robot.Constants.Shooter.Arm;

public class Vision {

    private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");


    public static boolean hasValidTargets(  ) {
        Alliance alliance = DriverStation.getAlliance().get();
        if ((table.getEntry("tv").getDouble(0) == 1) 
            && ((( alliance == Alliance.Red ) && (getTagID() == 3 || getTagID() == 4))
            || (( alliance == Alliance.Blue ) && (getTagID() == 7 || getTagID() == 8)))) 
            {
            System.out.println("VALID TARGET");
            return true;
        } else {
            System.out.println("NO VALID TARGET");
            return false;
        }
    }

    public static double getHorizontalOffset(){ // Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees / LL2: -29.8 to 29.8 degrees)
        return table.getEntry("tx").getDouble(0.0);
    }

    public static double getTagID() {
        return table.getEntry("tid").getDouble(0.0);
    }

    public static double getVerticalOffset(){ // Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees / LL2: -24.85 to 24.85 degrees)
        return table.getEntry("ty").getDouble(0.0);
    }

    public static double getTargetArea(){ // Target Area (0% of image to 100% of image)
        return table.getEntry("ta").getDouble(0.0);
    }

    public static double getPipelineLatency(){ // The pipeline's latency contribution (ms). Add to "cl" to get total latency.
        return table.getEntry("tl").getDouble(0.0);
    }

    public static double getTotalLatency(){ // Capture pipeline latency (ms). Time between the end of the exposure of the middle row of the sensor to the beginning of the tracking pipeline.
        return table.getEntry("cl").getDouble(0.0);
    }

    public static class AprilTag {

        public static double[] getBotPose(){
            return table.getEntry("botpose").getDoubleArray(new double[6]);
        }

        public static double[] getBotPose(Constants.Vision.BotPoseOptions pose){
            return table.getEntry("botpose_"+pose.toString().toLowerCase()).getDoubleArray(new double[6]);
        }

        public static double[] getCameraPose(Constants.Vision.PoseOptions pose){
            return table.getEntry("camerapose_"+pose.toString().toLowerCase()).getDoubleArray(new double[6]);
        }

        public static double[] getTargetPose(Constants.Vision.PoseOptions pose){
            return table.getEntry("targetpose_"+pose.toString().toLowerCase()).getDoubleArray(new double[6]);
        }

        public static int getID(){
            // why is the double array 6
            return (int) table.getEntry("tid").getDoubleArray(new double[6])[0];
        }

    }

    public static class Camera {

        public static void setLedMode(Constants.Vision.LedMode mode){
            table.getEntry("ledMode").setNumber(mode.ordinal());
        }

        public static void setCamMode(Constants.Vision.CameraMode mode){
            table.getEntry("camMode").setNumber(mode.ordinal());
        }

        public static void setPipeline(Constants.Vision.Pipeline pipeline){
            table.getEntry("pipeline").setNumber(pipeline.ordinal());
        }

        public static void setStream(Constants.Vision.Stream stream){
            table.getEntry("pipeline").setNumber(stream.ordinal());
        }

    }

    public static double calculateArmAngle(  ) {
        if ( !hasValidTargets() ) {// Check to see if Limelight has a target){ 
            return Arm.SPEAKER_ANGLE_SHORT;
        } else {
            System.out.println("BotPosX: " + AprilTag.getBotPose()[0]);
            System.out.println("BotPosY: " + AprilTag.getBotPose()[1]);
            double botXPosFromTarget = (Constants.Field.fieldLength / 2) - (Math.abs(AprilTag.getBotPose()[0]));
            System.out.println("xPosFromTarget: " + botXPosFromTarget);
            double botYPosFromTarget = Math.abs(AprilTag.getBotPose()[1]);
            System.out.println("yPosFromTarget: " + botYPosFromTarget);
            double botDistanceFromTarget =  Math.sqrt((Math.pow(botXPosFromTarget, 2)) + (Math.pow(botYPosFromTarget, 2)));
            System.out.println("distanceFromTarget: " + botDistanceFromTarget);
            double armAngle = Math.atan((Constants.Field.aprilTagDistanceFromGround / botDistanceFromTarget));
            System.out.println("Arm Angle: " + armAngle);
            // return armAngle;
            return 1;
        }
    }
}
