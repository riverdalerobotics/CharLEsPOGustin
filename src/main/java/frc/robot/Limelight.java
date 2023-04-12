package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    //Adapted from https://docs.limelightvision.io/en/latest/getting_started.html#basic-programming
    //all measurements in degrees
    /**The NetworkTable we are reading limelight values from*/
    NetworkTable table;
    private final double mountAngle = 0; //TODO: find mount angle (use trig)
    private final double mountHeight = 0; //TODO: find mount height
    /** Creates a new limelight object. Sets up the NetworkTable object */
    public Limelight(){
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }
    /** 
     * @return The horizontal offset in degrees. Returns 360 if no target found.
     */
    public double getTX(){
        NetworkTableEntry tx = table.getEntry("tx");
        return tx.getDouble(360);
    }
    /**
     * @return The vertical offset in degrees. Returns 360 if no target found
     */
    public double getTY(){
        NetworkTableEntry ty = table.getEntry("ty");
        return ty.getDouble(360);
    }
    /**
     * Due to problems experienced using tv, we now use this.
     * This implementation checks tx and ty
     * @return true if there is a target, false otherwise
     */
    public boolean getValidTarget(){
        return !(getTX()==360 && getTY()==360);
    }
    /**
     * @return the latency
     */
    public double getTL(){
        NetworkTableEntry tl = table.getEntry("tl");
        return tl.getDouble(-1);
    }
    /**
     * @return the target area (of the camera stream)
     */
    public double getTA(){
        NetworkTableEntry ta = table.getEntry("ta");
        return ta.getDouble(0);
    }
    /**
     * @return The distance to the target
     */
    public double calculateDistance(double targetHeight){
        return Math.abs(targetHeight-mountHeight)/Math.tan((mountAngle + getTX())*Math.PI/180);
    }
    /**
     * @return The robot pose (x,y,z,Roll,Pitch,Yaw). Returns (0,0,0,0,0,0) if no value
     */
    public double[] getBotPose(){
        NetworkTableEntry botpose = table.getEntry("botpose");
        return botpose.getDoubleArray(new double[6]);
    }
}
