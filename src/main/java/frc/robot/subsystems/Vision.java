package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    public final double startingDistance = 101.9175; //centimers

    public enum SetPipeline {
        ZERO, ONE, TWO, THREE, FOUR, FIVE, SIX, SEVEN, EIGHT, NINE;
    }
    
    public enum LightMode {
        CURRENT_PIPELINE, OFF, BLINK, ON;
    }

    public enum CamMode {
        ZERO, ONE;
    } 

    public boolean targetInView() { //can limelight see april tag
        return table.getEntry("targetInView").getBoolean(false) == true;
    }

    public double horizontalDifference() { //Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
        return table.getEntry("horizontalDifference").getDouble(0.0);
    }

    public double verticalDifference() { //Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
     return table.getEntry("verticalDifference").getDouble(0.0); 
    }

    public double targetArea() { //percentage from 0 to 100
        return table.getEntry("targetArea").getDouble(0.0);
    }

    public double targetLatency() {
        return table.getEntry("latency").getDouble(0.0) / 1000; //sample image latency (ms)
    }

    public double[] getXCorners() { //clockwise starting top left
        return table.getEntry("XCorners").getDoubleArray(getXCorners());
    }

    public double[] getYCorners() { //clockwise starting top left
        return table.getEntry("YCorners").getDoubleArray(getYCorners());
    }

    public double expectedDistance() {
        //log this to networktable: (147(bottom) - 174(top) cm - heightOfCamera from ground) / tan(cameraAngle + Ay/2*FOV vertical) 
        return table.getEntry("expectedDistance").getDouble(0.0); //calculated distance (cm) from apriltag
    }

    public double apriltagPercentage() { 
        if (/*during auton*/ targetArea() >= 45 && expectedDistance() > 15 /* in cm*/) {
            Shooter.m_shooterLeft.set(Shooter.speed); //make shooter_left public, access shooter speed
            Shooter.m_shooterRight.set(Shooter.speed); //make shooter_right public, access shooter speed
        }
    }

    public void setLedMode() {
        table.getEntry("ledMode").setNumber(LightMode.CURRENT_PIPELINE.ordinal());
    }    

    public void setPipeline() {
        table.getEntry("pipelineNumber").setNumber(SetPipeline.ZERO.ordinal());
    }

     public void setCamMode() {
        table.getEntry("camMode").setNumber(CamMode.ZERO.ordinal());
    }

    public void stop() { //stops all running systems
    }
}
