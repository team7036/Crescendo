package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision {

    public final double startingDistance = 101.9175; //centimeters

    private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

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

    public static boolean hasValidTargets() { // Whether the limelight has any valid targets (0 or 1)
        return table.getEntry("tv").getDouble(0) == 1;
    }

    public static double getHorizontalOffset(){ // Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees / LL2: -29.8 to 29.8 degrees)
        return table.getEntry("tx").getDouble(0.0);
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

        public static double[] getBotPose(BotPoseOptions pose){
            return table.getEntry("botpose_"+pose.toString().toLowerCase()).getDoubleArray(new double[6]);
        }

        public static double[] getCameraPose(PoseOptions pose){
            return table.getEntry("camerapose_"+pose.toString().toLowerCase()).getDoubleArray(new double[6]);
        }

        public static double[] getTargetPose(PoseOptions pose){
            return table.getEntry("targetpose_"+pose.toString().toLowerCase()).getDoubleArray(new double[6]);
        }

        public static double[] getID(){
            // why is the double array 6
            return table.getEntry("tid").getDoubleArray(new double[6]);
        }

    }

    public static class Camera {

        public static void setLedMode(LedMode mode){
            table.getEntry("ledMode").setNumber(mode.ordinal());
        }

        public static void setCamMode(CameraMode mode){
            table.getEntry("camMode").setNumber(mode.ordinal());
        }

        public static void setPipeline(Pipeline pipeline){
            table.getEntry("pipeline").setNumber(pipeline.ordinal());
        }

        public static void setStream(Stream stream){
            table.getEntry("pipeline").setNumber(stream.ordinal());
        }

    }
}
