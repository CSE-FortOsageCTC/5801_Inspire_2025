package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.LimeLightSubsystem;

public enum AlignPosition {
    LeftOffset(),
    CenterOffset(),
    RightOffset();

    private static AlignPosition alignPosition;
    private static Pose2d alignOffset;
    private static double rotationDegrees;
    private static double tagX;
    private static double tagY;
    private static AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    private static LimeLightSubsystem s_LimeLightSubsystem = LimeLightSubsystem.getInstance();
    
    public static AlignPosition getPosition(){

        if (alignPosition == null) {
            alignPosition = LeftOffset;
        }

        if(alignOffset == null){
            //layout.setOrigin(new Pose3d(0,0,0, new Rotation3d()));

            int tagID = s_LimeLightSubsystem.getAprilValue();

            tagX = fieldLayout.getTagPose(tagID).get().getTranslation().toTranslation2d().getX();
            tagY = fieldLayout.getTagPose(tagID).get().getTranslation().toTranslation2d().getY();
            rotationDegrees = fieldLayout.getTagPose(tagID).get().getRotation().toRotation2d().getDegrees();

            switch(alignPosition){
                case LeftOffset:
                    alignOffset = new Pose2d(Units.inchesToMeters(tagX - (Math.sin(rotationDegrees) * Constants.limelightScoringOffsetInches)), Units.inchesToMeters(tagY - (Math.cos(rotationDegrees) * Constants.limelightScoringOffsetInches)), Rotation2d.fromDegrees(rotationDegrees));
                    break;
                case CenterOffset:
                    alignOffset = new Pose2d(Units.inchesToMeters(tagX), Units.inchesToMeters(tagY), Rotation2d.fromDegrees(rotationDegrees));
                    break;
                case RightOffset:
                    alignOffset = new Pose2d(Units.inchesToMeters(tagX + (Math.sin(rotationDegrees) * Constants.limelightScoringOffsetInches)), Units.inchesToMeters(tagY + (Math.cos(rotationDegrees) * Constants.limelightScoringOffsetInches)), Rotation2d.fromDegrees(rotationDegrees));
                    break;
            }
        }

    

        return alignPosition;
    }

    public static Pose2d getAlignOffset() {
        return alignOffset;
    }

    public static void setPosition(AlignPosition alignPos){
        SmartDashboard.putString("Align Pos", alignPos.toString());
        alignPosition = alignPos;
        
        int tagID = s_LimeLightSubsystem.getAprilValue();

        tagX = fieldLayout.getTagPose(tagID).get().getTranslation().toTranslation2d().getX();
        tagY = fieldLayout.getTagPose(tagID).get().getTranslation().toTranslation2d().getY();
        rotationDegrees = fieldLayout.getTagPose(tagID).get().getRotation().toRotation2d().getDegrees();

        switch(alignPosition){
            case LeftOffset:
                alignOffset = new Pose2d(Units.inchesToMeters(tagX - (Math.sin(rotationDegrees) * Constants.limelightScoringOffsetInches)), Units.inchesToMeters(tagY - (Math.cos(rotationDegrees) * Constants.limelightScoringOffsetInches)), Rotation2d.fromDegrees(rotationDegrees));
                break;
            case CenterOffset:
                alignOffset = new Pose2d(Units.inchesToMeters(tagX), Units.inchesToMeters(tagY), Rotation2d.fromDegrees(rotationDegrees));
                break;
            case RightOffset:
                alignOffset = new Pose2d(Units.inchesToMeters(tagX + (Math.sin(rotationDegrees) * Constants.limelightScoringOffsetInches)), Units.inchesToMeters(tagY + (Math.cos(rotationDegrees) * Constants.limelightScoringOffsetInches)), Rotation2d.fromDegrees(rotationDegrees));
                break;
        }

    }
        
    AlignPosition(){
    }
}