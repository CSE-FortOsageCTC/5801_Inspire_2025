package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.125;

    public static final String limelightRight = "limelight-right";
    public static final String limelightLeft = "limelight-left";
    public static final String limelightSky = "limelight-sky";

    public static final boolean isRedAlliance = DriverStation.getAlliance().get().equals(Alliance.Red) ? true : false;

    public static final double feedForwardAngle = 30;

    public static final double speakerHeightMeters = 1.78;

    public static final double stageHeightMeters = 4; // 2.54

    public static final double scoringDx = Units.inchesToMeters(7);
    public static final double scoringDy = Units.inchesToMeters(18); // might need to change at comp :)
    public static final double centerDy = Units.inchesToMeters(19);
    public static final double lOneDx = Units.inchesToMeters(1);
    public static final double lOneDy = Units.inchesToMeters(21);
    public static final double redCoralDy = Units.inchesToMeters(18);
    public static final double blueCoralDy = Units.inchesToMeters(18);

    // True if new Ground Intake : False if Central MO intake
    public static final boolean isGroundIntake = true;
    

    public static final double coralIntakeSpeed = -1;
    public static final double algaeIntakeSpeed = -1;

    public static final class Swerve {
        public static final int pigeonID = 10;

        public static final COTSTalonFXSwerveConstants chosenModule = // TODO: This must be tuned to specific robot
                // COTSTalonFXSwerveConstants.SDS.MK4i.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);
                COTSTalonFXSwerveConstants.SDS.MK4i.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(21.5); // TODO: This must be tuned to specific
                                                                            // robot
        public static final double wheelBase = Units.inchesToMeters(21.5); // TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 1; // TODO: This must be tuned to specific robot || og value 0.12
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32; // TODO: This must be tuned to specific robot
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */

        public static final double maxSpeed = 11; // TODO: This must be tuned to specific robot


        /** Radians per Second */
        public static final double maxAngularVelocity = 13.0; // TODO: This must be tuned to specific robot
        public static final double panicRotation = 25.0;

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        public static final double minElevatorValue = 0;
        public static final double maxElevatorValue = -62.5;

        public static final double minElevatorAngle = 29.2;
        public static final double maxElevatorAngle = 61.2;

        public static final double degreesToEncoderValue = maxElevatorValue / (maxElevatorAngle - minElevatorAngle);
                    // max Distance from robot - wrist angle shenanigans
        public static final double XLimit = 38 - 10;

        // public static final double
        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 12;

            // BEVELS TO THE RIGHT
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-47.373046875); // 134.47265625
                                                                                                // //133.06640625
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 14;

            // BEVELS TO THE RIGHT
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(57.39257812500001); // -120.41015624999999
                                                                                                    // // -122.6953125
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 16;

            // BEVELS TO THE RIGHT
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-35.244140625); // 144.4921875 //
                                                                                                // 144.31640625
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 18;

            // BEVELS TO THE RIGHT
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-99.052734375); // 75.41015625 //
                                                                                                // 80.947265625
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 32;
        public static final double kPYController = 32;
        public static final double kPThetaController = 8;

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final double wristUpperLimitExtended = 21.49755859375; //17.648437;
    public static final double wristLowerLimit = 0;
    public static final double pivotUpperLimit = 0;
    public static final double pivotLowerLimit = -60;
    public static final double pivotExtensionLimit = -40;
    public static final double extensionUpperLimit = 0;
    public static final double extensionLowerLimit = -21.6;

    public static final double pivotDegreesPerEncoder = 1.73;
    public static final double extensionInchesPerEncoder = 2.16;

    public static final int blueScoringTagStart = 17;
    public static final int redScoringTagStart = 6;

    public static final int blueHumanPTagStart = 12;
    public static final int redHumanPTagStart = 1;

    // Original "claw" intake from Central MO (einstien)
    //ur mom lol (get owned noob!)
    public enum ArmPosition {
        Travel(-1, -.5, -1),
        L1 (-29.418701171875, -0.8544921875, 21.56298828125),
        L2(-29.869384765625, -0.8447265625, 1.619140625),
        L3(-42.489501953125, -6.85302734375, 5.236328125),
        L4(-52.29443359375, -20.2578125, 13.787109375), // Original: (-51.948974609375, -21.0, 12.2900390625) different rn because the new pvc
        HighAlgae(-41.728271484375, -5.177734375, 17.84521484375),
        LowAlgae(-33.37744140625, -0.037109375, 17.7041015625),
        Ground(0.134521484375, 0, 17.20068359375),
        GroundP(-26.415283203125, -0.00146484375, 17.28955078125),
        HumanP(-34.74365234375, 0.0029296875, 12.40283203125),
        Climb1(-45.785400390625, -4.8828125E-4, 13.1533203125),
        Climb2(-10.561767578125, -0.00244140625, 17.3291015625),
        Net(-59.96142578125, -21.3369140625, 21.56298828125),
        NetP(-59.97314453125, -2.77880859375, 21.56494140625),
        StartingConfig(-38.552490234375, 0.0068359375, 0.0),
        Manual(-1, -1, -1);

        public double pivot;
        public double extension;
        public double manipulator;

        public static ArmPosition currentPosition = ArmPosition.Manual;

        ArmPosition(double pivot, double extension, double manipulator) {
            this.pivot = pivot;
            this.extension = extension;
            this.manipulator = manipulator;
        }

        public static ArmPosition getPosition() {
            return currentPosition;
        }

        public static void setPosition(ArmPosition position) {
            // If trying to go to Ground and previous setpoint was anything near the reef, set it to StartingConfig instead
            // if (Ground.equals(position) && (L1.equals(currentPosition) || L2.equals(currentPosition) || L3.equals(currentPosition) 
            //  || L4.equals(currentPosition) || HighAlgae.equals(currentPosition) || LowAlgae.equals(currentPosition))) {

                // currentPosition = StartingConfig;
                // return;                
            //}

            if (Net.equals(position) && !NetP.equals(currentPosition)) {

                currentPosition = NetP;
                return;                
            } else if (Net.equals(position) && NetP.equals(currentPosition)) {
                currentPosition = Net;
                return;
            }

            if (ArmPosition.Ground.equals(position) && (ArmPosition.L4.equals(currentPosition) || ArmPosition.L3.equals(currentPosition) || ArmPosition.L2.equals(currentPosition) || ArmPosition.L1.equals(currentPosition))) {
                return;
            }

            currentPosition = position;
        }
    }

}