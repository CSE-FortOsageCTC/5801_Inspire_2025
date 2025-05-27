package frc.robot;

import java.util.NoSuchElementException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.Swerve;

public enum AlignPosition {
    LeftOffset(),
    CenterOffset(),
    RightOffset(),
    L1Offset(),
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
    public static AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    // private static LimeLightSubsystem s_LimeLightSubsystem =
    // LimeLightSubsystem.getRightInstance();
    private static Swerve s_Swerve = Swerve.getInstance();

    private static boolean scoring = true;

    public static AlignPosition getPosition() {

        if (alignPosition == null) {
            alignPosition = LeftOffset;
        }

        if (alignOffset == null) {
            setPosition(NoPos, true);
        }

        return alignPosition;
    }

    public static Pose2d getAlignOffset() {
        SmartDashboard.putString("April Align Offset", "(" + alignOffset.getX() + ", " + alignOffset.getY() + ")");
        return alignOffset;
    }

    public static void setPosition(AlignPosition alignPos, boolean isScoring) {
        SmartDashboard.putString("Align Pos", alignPos.toString());
        alignPosition = alignPos;

        // int tagID = s_LimeLightSubsystem.getAprilValue();

        s_Swerve.resetAlignApril();

        scoring = isScoring;
        // if (tagID == -1) {
        // alignOffset = s_Swerve.getEstimatedPosition();
        // return;
        // }

        // tagID = s_LimeLightSubsystem.getAprilValue();
        boolean isRed = DriverStation.Alliance.Red.equals(DriverStation.getAlliance().get());

        System.out.println(isRed);

        rotationDegrees = (int) Math.round(isScoring ? getNearestScoringPos().getRotation().getDegrees()
                : getNearestHumanPos().getRotation().getDegrees());
        rotationRadians = (isScoring ? getNearestScoringPos().getRotation().getRadians()
                : getNearestHumanPos().getRotation().getRadians()) - (Math.PI / 2);
        // double thetaRadians = Math.atan2(Constants.scoringDx, Constants.scoringDy);
        tagX = isScoring ? getNearestScoringPos().getX() : getNearestHumanPos().getX();
        tagY = isScoring ? getNearestScoringPos().getY() : getNearestHumanPos().getY();
        SmartDashboard.putNumber("Tag X", tagX);
        SmartDashboard.putNumber("Tag Y", tagY);
        SmartDashboard.putNumber("Tag Rotation", rotationDegrees);

        distance = isScoring? 
        Math.sqrt(((Constants.scoringDx * Constants.scoringDx)) + (Constants.scoringDy * Constants.scoringDy)) : 
        (isRed? Constants.redCoralDy:Constants.blueCoralDy);

        switch (alignPosition) {
            case LeftOffset:
                theta = Math.atan2(Constants.scoringDy, Constants.scoringDx) + rotationRadians;
                tagX = isScoring ? getNearestScoringPos().getX() : getNearestHumanPos().getX();
                tagY = isScoring ? getNearestScoringPos().getY() : getNearestHumanPos().getY();
                correctedPos();
                break;
            case CenterOffset:
                double hpOffset = isRed? Constants.redCoralDy:Constants.blueCoralDy;
                System.out.println(hpOffset);
                theta = Math.atan2(isScoring? Constants.centerDy:hpOffset, 0) + rotationRadians;
                distance = Constants.centerDy;
                tagX = isScoring ? getNearestScoringPos().getX() : getNearestHumanPos().getX();
                tagY = isScoring ? getNearestScoringPos().getY() : getNearestHumanPos().getY();
                correctedPos();
                break;
            case RightOffset:
                theta = Math.atan2(Constants.scoringDy, -Constants.scoringDx) + rotationRadians;
                tagX = isScoring ? getNearestScoringPos().getX() : getNearestHumanPos().getX();
                tagY = isScoring ? getNearestScoringPos().getY() : getNearestHumanPos().getY();
                correctedPos();
                break;
            case L1Offset:
                theta = Math.atan2(Constants.lOneDy, -Constants.lOneDx) + (rotationRadians);
                rotationDegrees += 32;
                distance = Math.sqrt(((Constants.lOneDx * Constants.lOneDx)) + (Constants.lOneDy * Constants.lOneDy));
                tagX = isScoring ? getNearestScoringPos().getX() : getNearestHumanPos().getX();
                tagY = isScoring ? getNearestScoringPos().getY() : getNearestHumanPos().getY();
                correctedPos();
                break;
            case NoPos:
                alignOffset = s_Swerve.getEstimatedPosition();
                break;
        }

        alignOffset = new Pose2d(correctedX, correctedY, Rotation2d.fromDegrees(rotationDegrees));

        SmartDashboard.putNumber("Translated April X", alignOffset.getX());
        SmartDashboard.putNumber("Translated April Y", alignOffset.getY());

    }

    public static void correctedPos() {
        correctedX = tagX + Math.cos(theta) * distance;
        correctedY = tagY + Math.sin(theta) * distance;
    }

    public static Pose2d getNearestScoringPos() {
        boolean isRed = DriverStation.Alliance.Red.equals(DriverStation.getAlliance().get());
        int tagStartRange = isRed ? Constants.redScoringTagStart : Constants.blueScoringTagStart;
        double closestDistance = 1000;
        Pose2d botPose = s_Swerve.getEstimatedPosition();
        Pose2d closestPose = botPose;
        for (int i = tagStartRange; i < tagStartRange + 6; i++) {
            Pose2d tagPose = fieldLayout.getTagPose(i).get().toPose2d();
            double tagDist = tagPose.getTranslation().getDistance(botPose.getTranslation());
            if (tagDist < closestDistance) {
                closestDistance = tagDist;
                closestPose = tagPose;
            }
        }
        return closestPose;
    }

    public static Pose2d getNearestHumanPos() {
        boolean isRed = DriverStation.Alliance.Red.equals(DriverStation.getAlliance().get());
        int tagStartRange = isRed ? Constants.redHumanPTagStart : Constants.blueHumanPTagStart;
        double closestDistance = 1000;
        Pose2d botPose = s_Swerve.getEstimatedPosition();
        Pose2d closestPose = botPose;
        for (int i = tagStartRange; i < tagStartRange + 2; i++) {
            Pose2d tagPose = fieldLayout.getTagPose(i).get().toPose2d();
            double tagDist = tagPose.getTranslation().getDistance(botPose.getTranslation());
            if (tagDist < closestDistance) {
                closestDistance = tagDist;
                closestPose = tagPose;
            }
        }
        return closestPose;
    }

    public static boolean getIsScoring() {
        return scoring;
    }

    public static void setIsScoring(boolean isScoring) {
        scoring = isScoring;
    }

    AlignPosition() {
    }
}