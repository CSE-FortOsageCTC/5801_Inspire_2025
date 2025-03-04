package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

/**
 * this class retrieves limelight values from the networktable
 */
public class LimeLightSubsystem extends SubsystemBase {

    private NetworkTableEntry botPoseEntry;
    private Pose2d botPose;
    private NetworkTable table;
    private NetworkTableEntry tid, tv, tx, ty, ta, ts;
    public NetworkTableEntry ts0, ts1, ts2;
    private double lastBotPoseTimestamp;
    private static LimeLightSubsystem limelightRight;
    private static LimeLightSubsystem limelightLeft;
    private static LimeLightSubsystem limelightSky;
    private static LimeLightSubsystem limelight;

    private String limelightString;

    public static LimeLightSubsystem getRightInstance() {
        if (limelightRight == null) {
            limelightRight = new LimeLightSubsystem(Constants.limelightRight);

            LimelightHelpers.setCameraPose_RobotSpace(Constants.limelightRight, -.003175, .28575, 0.2159, 0, 0, 172);
        }
        return limelightRight;
    }

    public static LimeLightSubsystem getLeftInstance() {
        if (limelightLeft == null) {
            limelightLeft = new LimeLightSubsystem(Constants.limelightLeft);
            LimelightHelpers.setCameraPose_RobotSpace(Constants.limelightLeft, 0, -0.2921, 0.2159, 0, 0, -171);

        }
        return limelightLeft;
    }

    public static LimeLightSubsystem getSkyInstance() {
        if (limelightSky == null) {
            limelightSky = new LimeLightSubsystem(Constants.limelightSky);
            LimelightHelpers.setCameraPose_RobotSpace(Constants.limelightSky, 0.2286, 0.19685, 0.2794, 180, 50, -1);

        }
        return limelightSky;
    }

    public static LimeLightSubsystem getInstance(String limelight) {
            switch (limelight) {
                case Constants.limelightLeft:
                    return getLeftInstance();
                case Constants.limelightRight:
                    return getRightInstance();
                case Constants.limelightSky:
                    return getSkyInstance();
                default:
                    break;
            }
        return limelightSky;
    }

    /**
     * Constructs Limelight Class
     */
    private LimeLightSubsystem(String limelightName) {
        limelightString = limelightName;
        botPose = new Pose2d();
        table = NetworkTableInstance.getDefault().getTable(limelightString);
        table.getEntry("pipeline").setNumber(0);
        botPoseEntry = table.getEntry("botpose");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        ts = table.getEntry("ts");
        tv = table.getEntry("tv");
        tid = table.getEntry("tid");
        ts0 = table.getEntry("ts0");
        ts1 = table.getEntry("ts1");
        ts2 = table.getEntry("ts2");

    }

    /**
     * Retrieving the april tag ID value
     * 
     * @return April tag ID value
     */
    public int getAprilValue() {
        tid = table.getEntry("tid");
        return (int) tid.getDouble(0.0);
    }

    /**
     * Checks if the April tag is detected
     * 
     * @return <code>true</code> when april tag is detected, <code>false</code>
     *         otherwise
     */
    public boolean hasTag() {
        tv = table.getEntry("tv");
        return tv.getDouble(0.0) == 1.0;
    }

    /**
     * Retrieves Horizontal Offset From Crosshair To Target
     */
    public double getX() {
        tx = table.getEntry("tx");
        return tx.getDouble(0.0);
    }

    /**
     * Retrieves Vertical Offset From Crosshair To Target
     */
    public double getY() {
        ty = table.getEntry("ty");
        return ty.getDouble(0.0);
    }

    /**
     * retrieves Target Area (0% of image to 100% of image)
     */
    public double getArea() {
        ta = table.getEntry("ta");
        return ta.getDouble(0.0);
    }

    /**
     * Retrieves Skew or rotation (-90 degrees to 0 degrees)
     */
    public double getSkew() {
        ts = table.getEntry("ts");
        return ts.getDouble(0.0);
    }

    public double getSkew0() {
        ts0 = table.getEntry("ts0");
        return ts0.getDouble(0.0);
    }

    public double getSkew1() {
        ts1 = table.getEntry("ts1");
        return ts1.getDouble(0.0);
    }

    public double getSkew2() {
        ts2 = table.getEntry("ts2");
        return ts2.getDouble(0.0);
    }

    /**
     * retrieves limelight values and prints them onto the log and smartdashboard
     */
    public void outputValues() {
        table = NetworkTableInstance.getDefault().getTable(limelightString);
        System.out.println(getAprilValue());
        System.out.println(hasTag());
        System.out.println(getX());
        System.out.println(getY());
        System.out.println(getSkew());
        System.out.println(getArea());

    }

    /*
     * Returns 0 for all values if no Apriltag detected
     */
    public Pose2d getBotPose() {
        return botPose;
    }

    public void UpdateBotPose() {
        botPoseEntry = table.getEntry("botpose_wpiblue");
        double[] botpose = botPoseEntry.getDoubleArray(new double[7]);
        Pose2d visionPose = new Pose2d(botpose[0], botpose[1], Rotation2d.fromDegrees((botpose[5] + 360) % 360));
        this.lastBotPoseTimestamp = Timer.getFPGATimestamp() - (botpose[6] / 1000);
        botPose = visionPose;
    }

    public double getLastBotPoseTimestamp() {
        return this.lastBotPoseTimestamp;
    }

    @Override
    public void periodic() {
        UpdateBotPose();
    }
}