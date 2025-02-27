package frc.robot;

import java.util.NoSuchElementException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.Swerve;

public enum AlignPosition {
    LeftOffset(),
    CenterOffset(),
    RightOffset(),
    NoPos();

    private static AlignPosition alignPosition;
    private static Pose2d alignOffset;
    private static double rotationDegrees;
    private static double tagX;
    private static double tagY;
    private static AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    private static LimeLightSubsystem s_LimeLightSubsystem = LimeLightSubsystem.getRightInstance();
    private static Swerve s_Swerve = Swerve.getInstance();
    
    public static AlignPosition getPosition(){

        if (alignPosition == null) {
            alignPosition = LeftOffset;
        }

        if(alignOffset == null){
            //layout.setOrigin(new Pose3d(0,0,0, new Rotation3d()));

            int tagID = s_LimeLightSubsystem.getAprilValue();

            tagX = fieldLayout.getTagPose(tagID).get().getTranslation().toTranslation2d().getX();
            tagY = fieldLayout.getTagPose(tagID).get().getTranslation().toTranslation2d().getY();
            rotationDegrees = fieldLayout.getTagPose(tagID).get().getRotation().toRotation2d().getRadians();
            SmartDashboard.putNumber("AprilTag Rotation Degrees", rotationDegrees);

            switch(alignPosition){
                case LeftOffset:
                    alignOffset = new Pose2d((tagX * 2) - (Math.sin(rotationDegrees) * Units.inchesToMeters(Constants.scoringDx) + (Math.cos(rotationDegrees) * Units.inchesToMeters(Constants.scoringDy))), (tagY * 2) - (Math.cos(rotationDegrees) * Units.inchesToMeters(Constants.scoringDx) + (Math.sin(rotationDegrees) * Units.inchesToMeters(Constants.scoringDy))), Rotation2d.fromRadians(rotationDegrees));
                    break;
                case CenterOffset:
                    alignOffset = new Pose2d(Units.inchesToMeters(tagX), Units.inchesToMeters(tagY), Rotation2d.fromDegrees(rotationDegrees));
                    break;
                case RightOffset:
                    alignOffset = new Pose2d((tagX * 2) + (Math.sin(rotationDegrees) * Units.inchesToMeters(Constants.scoringDx) + (Math.cos(rotationDegrees) * Units.inchesToMeters(Constants.scoringDy))), (tagY * 2) + (Math.cos(rotationDegrees) * Units.inchesToMeters(Constants.scoringDx) + (Math.sin(rotationDegrees) * Units.inchesToMeters(Constants.scoringDy))), Rotation2d.fromRadians(rotationDegrees));
                    break;
                case NoPos:
                    alignOffset = s_Swerve.getEstimatedPosition();
                    break;
            }
        }

        return alignPosition;
    }

    public static Pose2d getAlignOffset() {
        // SmartDashboard.putString("April Align Offset", "(" + alignOffset.getX() + ", " + alignOffset.getY() + ")");
        return alignOffset;
    }

    public static void setPosition(AlignPosition alignPos){
        SmartDashboard.putString("Align Pos", alignPos.toString());
        alignPosition = alignPos;
        
        int tagID = s_LimeLightSubsystem.getAprilValue();

        if (tagID == -1) {
            alignOffset = s_Swerve.getEstimatedPosition();
            return;
        }


        tagX = fieldLayout.getTagPose(tagID).get().getTranslation().toTranslation2d().getX();
        tagY = fieldLayout.getTagPose(tagID).get().getTranslation().toTranslation2d().getY();
        rotationDegrees = fieldLayout.getTagPose(tagID).get().getRotation().toRotation2d().getDegrees();
        SmartDashboard.putNumber("Tag X", tagX);
        SmartDashboard.putNumber("Tag Y", tagY);
        SmartDashboard.putNumber("Tag Rotation", rotationDegrees);

        double distance = Math.sqrt((Constants.scoringDx * Constants.scoringDx) + (Constants.scoringDy * Constants.scoringDy));
        double theta = 60;//(2*Math.PI) - Math.atan2(Constants.scoringDy, Constants.scoringDx);
        double xResult;
        double yResult;
        switch(alignPosition){
            case LeftOffset:

                xResult = tagX + ((Constants.scoringDy * Math.cos(Units.degreesToRadians(rotationDegrees))) - (Constants.scoringDx * Math.sin(Units.degreesToRadians(rotationDegrees))));
                yResult = tagY + ((Constants.scoringDy * Math.sin(Units.degreesToRadians(rotationDegrees))) + (Constants.scoringDx * Math.cos(Units.degreesToRadians(rotationDegrees))));
                alignOffset = new Pose2d(xResult, yResult, Rotation2d.fromDegrees(rotationDegrees));
                break;
            case CenterOffset:
                alignOffset = new Pose2d(Units.inchesToMeters(tagX), Units.inchesToMeters(tagY), Rotation2d.fromDegrees(rotationDegrees));
                break;
            case RightOffset:

                xResult = tagX + (Math.cos(Units.degreesToRadians(rotationDegrees) - theta) * distance);
                yResult = tagY + (Math.sin(Units.degreesToRadians(rotationDegrees) - theta) * distance);
                alignOffset = new Pose2d(xResult, yResult, Rotation2d.fromDegrees(rotationDegrees));
                // alignOffset = new Pose2d(tagX + (Math.cos(Units.degreesToRadians(rotationDegrees + 15)) * Units.inchesToMeters(Constants.scoringDx)), tagY + (Math.sin(Units.degreesToRadians(rotationDegrees + 15)) * Units.inchesToMeters(Constants.scoringDy)), Rotation2d.fromDegrees(rotationDegrees));
                break;
            case NoPos:
                alignOffset = s_Swerve.getEstimatedPosition();
                break;
        }

        SmartDashboard.putNumber("Translated April X", alignOffset.getX());
        SmartDashboard.putNumber("Translated April Y", alignOffset.getY());

    }
        
    AlignPosition(){
    }
}