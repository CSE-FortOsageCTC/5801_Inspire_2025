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
    private static double rotationRadians;
    private static double tagX;
    private static double tagY;
    private static AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    private static LimeLightSubsystem s_LimeLightSubsystem = LimeLightSubsystem.getInstance();
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
            rotationRadians = fieldLayout.getTagPose(tagID).get().getRotation().toRotation2d().getRadians();

            switch(alignPosition){
                case LeftOffset:
                    alignOffset = new Pose2d((tagX * 2) - (Math.sin(rotationRadians) * Units.inchesToMeters(Constants.limelightScoringOffsetInches) + (Math.cos(rotationRadians) * Units.inchesToMeters(Constants.limelightScoringDistance))), (tagY * 2) - (Math.cos(rotationRadians) * Units.inchesToMeters(Constants.limelightScoringOffsetInches) + (Math.sin(rotationRadians) * Units.inchesToMeters(Constants.limelightScoringDistance))), Rotation2d.fromRadians(rotationRadians));
                    break;
                case CenterOffset:
                    alignOffset = new Pose2d(Units.inchesToMeters(tagX), Units.inchesToMeters(tagY), Rotation2d.fromDegrees(rotationRadians));
                    break;
                case RightOffset:
                    alignOffset = new Pose2d((tagX * 2) + (Math.sin(rotationRadians) * Units.inchesToMeters(Constants.limelightScoringOffsetInches) + (Math.cos(rotationRadians) * Units.inchesToMeters(Constants.limelightScoringDistance))), (tagY * 2) + (Math.cos(rotationRadians) * Units.inchesToMeters(Constants.limelightScoringOffsetInches) + (Math.sin(rotationRadians) * Units.inchesToMeters(Constants.limelightScoringDistance))), Rotation2d.fromRadians(rotationRadians));
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
        rotationRadians = fieldLayout.getTagPose(tagID).get().getRotation().toRotation2d().getDegrees();
        SmartDashboard.putNumber("Tag X", tagX);
        SmartDashboard.putNumber("Tag Y", tagY);
        SmartDashboard.putNumber("Tag Rotation", rotationRadians);

        

        switch(alignPosition){
            case LeftOffset:
                alignOffset = new Pose2d(tagX - (Math.sin(rotationRadians) * Units.inchesToMeters(Constants.limelightScoringOffsetInches)), tagY - (Math.cos(rotationRadians) * Units.inchesToMeters(Constants.limelightScoringOffsetInches)), Rotation2d.fromDegrees(rotationRadians));
                break;
            case CenterOffset:
                alignOffset = new Pose2d(Units.inchesToMeters(tagX), Units.inchesToMeters(tagY), Rotation2d.fromDegrees(rotationRadians));
                break;
            case RightOffset:
                alignOffset = new Pose2d(tagX + (Math.sin(rotationRadians) * Units.inchesToMeters(Constants.limelightScoringOffsetInches)), tagY + (Math.cos(rotationRadians) * Units.inchesToMeters(Constants.limelightScoringOffsetInches)), Rotation2d.fromDegrees(rotationRadians));
                break;
            case NoPos:
                alignOffset = s_Swerve.getEstimatedPosition();
                break;
        }

    }
        
    AlignPosition(){
    }
}