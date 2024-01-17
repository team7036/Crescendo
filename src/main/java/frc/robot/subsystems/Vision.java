package frc.robot.subsystems;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;



public class Vision {
    /*
     * Finds brightest target on Camera
     * 
     */

    private final static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    private enum LightMode {
        PIPELINE_DEFAULT,
        OFF,
        BLINK,
        ON
    }

    private enum CamMode {
        Vision_Processor,
        Driver_Camera
    }

    private enum PipelineNumber {
        Zero,
        One,
        Two,
        Three,
        Four,
        Five,
        Six,
        Seven,
        Eight,
        Nine
    }
    
    public static boolean hasTarget() { //is target detected by limelight
        return table.getEntry("tv").getDouble(0) == 1;
    }

    public static double getTargetX() { //horizontal difference between the center of the target and camera crosshair
        return table.getEntry("tx").getDouble(0.00);
    }

    public static double getTargetY() { //vertical differerence between the center of the target and camera crosshair
        return table.getEntry("ty").getDouble(0.00);
    }

    public static double getTargetArea() { //area (in terms of percentage) of the camera that the target takes up
        return table.getEntry("ta").getDouble(0.00);
    }

    public static double getTargetSkew() {//Gets target skew(rotation) 
        return table.getEntry("ts").getDouble(0.00);
    }
    public static boolean setLedMode(LightMode mode) { //current mode of the LEDs on the limelight (0 for pipeline default, 1 for off, 2 for blink, 3 for on)
        return table.getEntry("ledMode").setNumber(mode.ordinal());
    }

    public static boolean setCamMode(CamMode mode) {//Camera mode of operation
        return table.getEntry("camMode").setNumber(mode.ordinal());
    }

    public static boolean setLimelightPipeline(PipelineNumber mode) {
        return table.getEntry("pipelineNumber").setNumber(mode.ordinal());
    }

    //next is to upload these values to smartdashboard
}