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
    private static int rotationDegrees;
    private static double rotationRadians;
    private static double correctedX;
    private static double correctedY;
    private static double tagX;
    private static double tagY;
    private static double distance;
    private static double theta;
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
            rotationDegrees = (int) Math.round(fieldLayout.getTagPose(tagID).get().getRotation().toRotation2d().getDegrees());
            rotationRadians = fieldLayout.getTagPose(tagID).get().getRotation().toRotation2d().getRadians() - (Math.PI/2);
            SmartDashboard.putNumber("AprilTag Rotation Degrees", rotationDegrees);
            double distance = Math.sqrt((Constants.scoringDx * Constants.scoringDx) + (Constants.scoringDy * Constants.scoringDy));
            double thetaRadians = Math.atan2(Constants.scoringDx, Constants.scoringDy);

            SmartDashboard.putNumber("Cos Radians", Math.cos(rotationRadians));
            SmartDashboard.putNumber("Cos Degrees", Math.cos(Units.degreesToRadians(rotationDegrees)));

            switch(alignPosition){
                case LeftOffset:
                    theta = Math.atan2(Constants.scoringDy, Constants.scoringDx) + rotationRadians;
                    correctedX = tagX + Math.cos(theta) * distance;
                    correctedY = tagY + Math.sin(theta) * distance;
                    // correctedX = tagX + (Constants.scoringDx * Math.cos(rotationRadians) + (Constants.scoringDy * Math.sin(rotationRadians)));
                    // correctedY = tagY + (Constants.scoringDx * Math.sin(rotationRadians) - (Constants.scoringDy * Math.cos(rotationRadians)));
                    alignOffset = new Pose2d(correctedX, correctedY, Rotation2d.fromDegrees(rotationDegrees));
                    //alignOffset = new Pose2d(tagX + (Math.cos(rotationRadians + thetaRadians) * distance), tagY + (Math.sin(rotationRadians + thetaRadians) * distance), Rotation2d.fromRadians(rotationRadians));
                    //alignOffset = new Pose2d((tagX * 2) - (Math.sin(rotationDegrees) * Units.inchesToMeters(Constants.scoringDx) + (Math.cos(rotationDegrees) * Units.inchesToMeters(Constants.scoringDy))), (tagY * 2) - (Math.cos(rotationDegrees) * Units.inchesToMeters(Constants.scoringDx) + (Math.sin(rotationDegrees) * Units.inchesToMeters(Constants.scoringDy))), Rotation2d.fromRadians(rotationDegrees));
                    break;
                case CenterOffset:
                    alignOffset = new Pose2d(Units.inchesToMeters(tagX), Units.inchesToMeters(tagY), Rotation2d.fromDegrees(rotationDegrees));
                    break;
                case RightOffset:
                    theta = Math.atan2(Constants.scoringDy, -Constants.scoringDx) + rotationRadians;
                    correctedX = tagX + Math.cos(theta) * distance;
                    correctedY = tagY + Math.sin(theta) * distance;
                    // correctedX = tagX + (Constants.scoringDx * Math.cos(rotationRadians) - (Constants.scoringDy * Math.sin(rotationRadians)));
                    // correctedY = tagY + (Constants.scoringDx * Math.sin(rotationRadians) + (Constants.scoringDy * Math.cos(rotationRadians)));
                    alignOffset = new Pose2d(correctedX, correctedY, Rotation2d.fromDegrees(rotationDegrees));
                    //alignOffset = new Pose2d(tagX + (Math.cos(rotationRadians - thetaRadians) * distance), tagY + (Math.sin(rotationRadians - thetaRadians) * distance), Rotation2d.fromRadians(rotationRadians));

                    //alignOffset = new Pose2d((tagX * 2) + (Math.sin(rotationDegrees) * Units.inchesToMeters(Constants.scoringDx) + (Math.cos(rotationDegrees) * Units.inchesToMeters(Constants.scoringDy))), (tagY * 2) + (Math.cos(rotationDegrees) * Units.inchesToMeters(Constants.scoringDx) + (Math.sin(rotationDegrees) * Units.inchesToMeters(Constants.scoringDy))), Rotation2d.fromRadians(rotationDegrees));
                    break;
                case NoPos:
                    alignOffset = s_Swerve.getEstimatedPosition();
                    break;
            }
        }

        return alignPosition;
    }

    public static Pose2d getAlignOffset() {
        SmartDashboard.putString("April Align Offset", "(" + alignOffset.getX() + ", " + alignOffset.getY() + ")");
        return alignOffset;
    }

    public static void setPosition(AlignPosition alignPos){
        SmartDashboard.putString("Align Pos", alignPos.toString());
        alignPosition = alignPos;
        
        int tagID = s_LimeLightSubsystem.getAprilValue();

        s_Swerve.resetAlignApril();

        if (tagID == -1) {
            alignOffset = s_Swerve.getEstimatedPosition();
            return;
        }

        tagID = s_LimeLightSubsystem.getAprilValue();

        rotationDegrees = (int) Math.round(fieldLayout.getTagPose(tagID).get().getRotation().toRotation2d().getDegrees());
        rotationRadians = fieldLayout.getTagPose(tagID).get().getRotation().toRotation2d().getRadians() - (Math.PI/2);
        double thetaRadians = Math.atan2(Constants.scoringDx, Constants.scoringDy);
        tagX = fieldLayout.getTagPose(tagID).get().getTranslation().toTranslation2d().getX();
        tagY = fieldLayout.getTagPose(tagID).get().getTranslation().toTranslation2d().getY();
        rotationDegrees = (int) Math.round(fieldLayout.getTagPose(tagID).get().getRotation().toRotation2d().getDegrees());
        SmartDashboard.putNumber("Tag X", tagX);
        SmartDashboard.putNumber("Tag Y", tagY);
        SmartDashboard.putNumber("Tag Rotation", rotationDegrees);

        double distance = Math.sqrt((Constants.scoringDx * Constants.scoringDx) + (Constants.scoringDy * Constants.scoringDy));

        switch(alignPosition){
            case LeftOffset:
                theta = Math.atan2(Constants.scoringDy, Constants.scoringDx) + rotationRadians;
                correctedX = tagX + Math.cos(theta) * distance;
                correctedY = tagY + Math.sin(theta) * distance;
                // correctedX = tagX + (Constants.scoringDx * Math.cos(rotationRadians) + (Constants.scoringDy * Math.sin(rotationRadians)));
                // correctedY = tagY + (Constants.scoringDx * Math.sin(rotationRadians) - (Constants.scoringDy * Math.cos(rotationRadians)));
                alignOffset = new Pose2d(correctedX, correctedY, Rotation2d.fromDegrees(rotationDegrees));
                //alignOffset = new Pose2d(tagX + (Math.cos(rotationRadians + thetaRadians) * distance), tagY + (Math.sin(rotationRadians + thetaRadians) * distance), Rotation2d.fromRadians(rotationRadians));
                //alignOffset = new Pose2d((tagX * 2) - (Math.sin(rotationDegrees) * Units.inchesToMeters(Constants.scoringDx) + (Math.cos(rotationDegrees) * Units.inchesToMeters(Constants.scoringDy))), (tagY * 2) - (Math.cos(rotationDegrees) * Units.inchesToMeters(Constants.scoringDx) + (Math.sin(rotationDegrees) * Units.inchesToMeters(Constants.scoringDy))), Rotation2d.fromRadians(rotationDegrees));
                break;
            case CenterOffset:
                alignOffset = new Pose2d(Units.inchesToMeters(tagX), Units.inchesToMeters(tagY), Rotation2d.fromDegrees(rotationDegrees));
                break;
            case RightOffset:
                theta = Math.atan2(Constants.scoringDy, -Constants.scoringDx) + rotationRadians;
                correctedX = tagX + Math.cos(theta) * distance;
                correctedY = tagY + Math.sin(theta) * distance;
                // correctedX = tagX + (Constants.scoringDx * Math.cos(rotationRadians) - (Constants.scoringDy * Math.sin(rotationRadians)));
                // correctedY = tagY + (Constants.scoringDx * Math.sin(rotationRadians) + (Constants.scoringDy * Math.cos(rotationRadians)));
                alignOffset = new Pose2d(correctedX, correctedY, Rotation2d.fromDegrees(rotationDegrees));
                //alignOffset = new Pose2d(tagX + (Math.cos(rotationRadians - thetaRadians) * distance), tagY + (Math.sin(rotationRadians - thetaRadians) * distance), Rotation2d.fromRadians(rotationRadians));

                //alignOffset = new Pose2d((tagX * 2) + (Math.sin(rotationDegrees) * Units.inchesToMeters(Constants.scoringDx) + (Math.cos(rotationDegrees) * Units.inchesToMeters(Constants.scoringDy))), (tagY * 2) + (Math.cos(rotationDegrees) * Units.inchesToMeters(Constants.scoringDx) + (Math.sin(rotationDegrees) * Units.inchesToMeters(Constants.scoringDy))), Rotation2d.fromRadians(rotationDegrees));
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