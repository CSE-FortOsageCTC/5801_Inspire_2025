package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public enum AlignPosition {
            AmpPos(),
            SpeakerPos(),
            StagePos(),
            SourcePos(),
            ClimbPos(),
            AutoPickup(),
            Manual(), 
            TrapPos1(),
            TrapPos2(),
            TrapPos3();

            private static AlignPosition alignPosition;
            private static Pose2d alignPose;

            public static AlignPosition getPosition(){

                if (alignPosition == null) {
                    alignPosition = SpeakerPos;
                }

                if(alignPose == null){
                    boolean isRed = Constants.isRedAlliance;
                    //layout.setOrigin(new Pose3d(0,0,0, new Rotation3d()));
                    switch(alignPosition){
                        case AmpPos:
                            alignPose = isRed? new Pose2d(Units.inchesToMeters(578.77 + 36), Units.inchesToMeters(323 - 12), Rotation2d.fromDegrees(270)): new Pose2d(Units.inchesToMeters(72.5 + 36), Units.inchesToMeters(323 - 12), Rotation2d.fromDegrees(270));
                            break;
                        case StagePos:
                        case SpeakerPos:
                            alignPose = isRed? new Pose2d(Units.inchesToMeters(652.73), Units.inchesToMeters(218.42), null): new Pose2d(Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42), null);
                            break;
                        case SourcePos:
                            alignPose = isRed? new Pose2d(Units.inchesToMeters(593.68), Units.inchesToMeters(9.68), Rotation2d.fromDegrees(-145)): new Pose2d(Units.inchesToMeters(57.54), Units.inchesToMeters(9.68), Rotation2d.fromDegrees(-55));
                            break;
                        case ClimbPos:
                            alignPose = isRed? new Pose2d(Units.inchesToMeters(441.74), Units.inchesToMeters(161.62), null): new Pose2d(Units.inchesToMeters(209.48), Units.inchesToMeters(161.62), null);
                            break;
                        case AutoPickup:
                            alignPose = null;
                            break;
                        case Manual:
                            alignPose = null;
                            break;
                        case TrapPos1:
                            alignPose = isRed? new Pose2d(Units.inchesToMeters(441.74), Units.inchesToMeters(161.62), null): new Pose2d(Units.inchesToMeters(209.48), Units.inchesToMeters(161.62), null);
                            break;
                        case TrapPos2:
                            alignPose = isRed? new Pose2d(Units.inchesToMeters(468.69), Units.inchesToMeters(177.1), Rotation2d.fromDegrees(120)): new Pose2d(Units.inchesToMeters(182.73), Units.inchesToMeters(177.1), Rotation2d.fromDegrees(300));
                            break;
                        case TrapPos3:
                            alignPose = isRed? new Pose2d(Units.inchesToMeters(468.69), Units.inchesToMeters(146.19), Rotation2d.fromDegrees(0)): new Pose2d(Units.inchesToMeters(182.73), Units.inchesToMeters(146.19), Rotation2d.fromDegrees(180));
                            break;
                    }
                }

                

                return alignPosition;
            }

            public static Pose2d getAlignPose() {
                return alignPose;
            }

            public static void setPosition(AlignPosition alignPos){
                SmartDashboard.putString("Align Pos", alignPos.toString());
                alignPosition = alignPos;
                boolean isRed = Constants.isRedAlliance;
                    switch(alignPosition){
                        case AmpPos:                                                                                    
                            alignPose = isRed? new Pose2d(Units.inchesToMeters(578.77 + 6.5), Units.inchesToMeters(323 - 12), Rotation2d.fromDegrees(270)): new Pose2d(Units.inchesToMeters(72.5 + 6.5), Units.inchesToMeters(323 - 12), Rotation2d.fromDegrees(270));
                            break;
                        case StagePos:
                        case SpeakerPos:
                            alignPose = isRed? new Pose2d(Units.inchesToMeters(652.73), Units.inchesToMeters(218.42 - 6.5), null): new Pose2d(Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42 - 6.5), null);
                            //alignPose = isRed? layout.getTagPose(4).get().toPose2d() : layout.getTagPose(7).get().toPose2d();
                            break;
                        case SourcePos:

                            alignPose = isRed? new Pose2d(Units.inchesToMeters(593.68), Units.inchesToMeters(9.68), Rotation2d.fromDegrees(-145)): new Pose2d(Units.inchesToMeters(57.54), Units.inchesToMeters(9.68), Rotation2d.fromDegrees(-55));
                            break;
                        case ClimbPos:
                            alignPose = isRed? new Pose2d(Units.inchesToMeters(441.74), Units.inchesToMeters(161.62), null): new Pose2d(Units.inchesToMeters(209.48), Units.inchesToMeters(161.62), null);
                            break;
                        case AutoPickup:
                            alignPose = null;
                            break;
                        case Manual:
                            alignPose = null;
                            break;
                        case TrapPos1:
                            alignPose = isRed? new Pose2d(Units.inchesToMeters(441.74), Units.inchesToMeters(161.62), null): new Pose2d(Units.inchesToMeters(209.48), Units.inchesToMeters(161.62), null);
                            break;
                        case TrapPos2:
                            alignPose = isRed? new Pose2d(Units.inchesToMeters(468.69), Units.inchesToMeters(177.1), Rotation2d.fromDegrees(120)): new Pose2d(Units.inchesToMeters(182.73), Units.inchesToMeters(177.1), Rotation2d.fromDegrees(300));
                            break;
                        case TrapPos3:
                            alignPose = isRed? new Pose2d(Units.inchesToMeters(468.69), Units.inchesToMeters(146.19), Rotation2d.fromDegrees(0)): new Pose2d(Units.inchesToMeters(182.73), Units.inchesToMeters(146.19), Rotation2d.fromDegrees(180));
                            break;
                    }

            }
                
            AlignPosition(){
                
            }
        }