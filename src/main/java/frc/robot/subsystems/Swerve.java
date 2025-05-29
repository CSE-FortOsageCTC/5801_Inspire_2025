package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Pigeon2Configurator;
import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.hardware.Pigeon2;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AlignPosition;
import frc.robot.AutoRotateUtil;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.SwerveModule;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    private SwerveDrivePoseEstimator swerveEstimator;
    // private SwerveDrivePoseEstimator limeLightSwerveEstimator;
    private static Swerve swerve;
    public double gyroOffset;
    public PIDController rotationPidController;
    public PIDController yTranslationPidController;
    public PIDController xTranslationPidController;
    public LimeLightSubsystem f_Limelight;
    public AutoRotateUtil s_AutoRotateUtil;
    public Debouncer pieceSeenDebouncer;
    public Field2d field;
    public StructPublisher<Pose3d> publisher;
    public StructArrayPublisher<Pose3d> arrayPublisher;
    public Pose3d poseA = new Pose3d();
    public Pose3d poseB = new Pose3d();

    public Orchestra orchestra = new Orchestra();

    public boolean isRed;
    // public ProfiledPIDController translationXController = new
    // ProfiledPIDController(10, 0, 0, new TrapezoidProfile.Constraints(1, .5));
    // public ProfiledPIDController translationYController = new
    // ProfiledPIDController(10, 0, 0, new TrapezoidProfile.Constraints(1, .5));

    public ProfiledPIDController translationXController = new ProfiledPIDController(0.5, 0, 0, new TrapezoidProfile.Constraints(400, 250));
    public ProfiledPIDController translationYController = new ProfiledPIDController(0.5, 0, 0,new TrapezoidProfile.Constraints(400, 250));

    private final PIDController autoXController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController autoYController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController autoHeadingController = new PIDController(8.0, 0.0, 0.0);

    public DriveParams autoPickupDriveParams;

    public static Swerve getInstance() {
        if (swerve == null) {
            swerve = new Swerve();
        }
        return swerve;
    }

    private Swerve() {
        isRed = DriverStation.getAlliance().get().equals(Alliance.Red);
        pieceSeenDebouncer = new Debouncer(.1, Debouncer.DebounceType.kBoth);
        gyro = new Pigeon2(Constants.Swerve.pigeonID);

        Pigeon2Configuration configuration = new Pigeon2Configuration();
        // configuration.withMountPose(new MountPoseConfigs().withMountPoseYaw(isRed? 180:0));
        // gyro.getConfigurator().apply(configuration);


        gyro.reset();
        //gyro.setYaw(isRed? 180:0);
        f_Limelight = LimeLightSubsystem.getRightInstance();
        s_AutoRotateUtil = new AutoRotateUtil(0);
        field = new Field2d();

        autoHeadingController.enableContinuousInput(-Math.PI, Math.PI);

        translationXController.setTolerance(0.015);
        translationYController.setTolerance(0.015);

        publisher = NetworkTableInstance.getDefault().getStructTopic("MyPose", Pose3d.struct).publish();
        arrayPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("MyPoseArray", Pose3d.struct).publish();

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod0.constants),
                new SwerveModule(1, Constants.Swerve.Mod1.constants),
                new SwerveModule(2, Constants.Swerve.Mod2.constants),
                new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        for(SwerveModule mod : mSwerveMods){
            orchestra.addInstrument(mod.mDriveMotor);
            orchestra.addInstrument(mod.mAngleMotor);
        }
        List music = new ArrayList();
        music.add("midis/jeopardy.chrp")
        music.add("midis/freeBird.chrp")
        music.add("midis/mario.chrp")
        music.add("midis/pirate.chrp")
        music.add("midis/pokemon.chrp")
        music.add("midis/star.chrp")

        int index = (int)(Math.random() * music.size());
        orchestra.loadMusic(music.get(index));

        playOrchestra();

        // swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics,
        // getGyroYaw(), getModulePositions());
        swerveEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getGyroRot2d(),
                getModulePositions(), new Pose2d(0, 0, new Rotation2d()));
        // limeLightSwerveEstimator = new
        // SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getGyroRot2d(),
        // getModulePositions(), new Pose2d(0, 0, new Rotation2d()));

        swerveEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(.5)));
        // limeLightSwerveEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.5,
        // 0.5, Units.degreesToRadians(.5)));

        swerveEstimator.resetPosition(getGyroRot2d(), getModulePositions(), new Pose2d(8.77, 4.05, new Rotation2d()));
    }

    public void playOrchestra() {
        orchestra.play();
    }

    public void stopOrchestra() {
        orchestra.stop();
    }

    public void updatePoseEstimator() {
        SwerveModulePosition[] getModPos = getModulePositions();
        // if (DriverStation.isAutonomousEnabled()) {

        // }

        swerveEstimator.updateWithTime(Timer.getFPGATimestamp(), getGyroRot2d().rotateBy(Rotation2d.fromDegrees(isRed ? 180 : 0)), getModPos);
        // limeLightSwerveEstimator.updateWithTime(Timer.getFPGATimestamp(),
        // getGyroYaw(), getModPos);
    }

    public void setTrajectory(Trajectory<SwerveSample> traj) {
        field.getObject("traj").setTrajectory(
                TrajectoryGenerator.generateTrajectory(List.of(traj.getPoses()), new TrajectoryConfig(10000, 10000)));
    }

    public Pose2d getEstimatedPosition() {
        return swerveEstimator.getEstimatedPosition();
        // return swerveOdometry.getPoseMeters();
    }

    public double getVelocityCorrectionDistance(double distance, ChassisSpeeds speeds) {
        double noteAirTime = distance / 10.415;
        double robotDistance = noteAirTime * speeds.vyMetersPerSecond;
        robotDistance = DriverStation.getAlliance().equals(Alliance.Red) ? robotDistance * 0 : robotDistance;
        return robotDistance + distance;
    }

    public double getVelocityCorrection(double distance, ChassisSpeeds speeds) {
        double noteAirTime = distance / 10.415;
        double robotDistanceCorrection = noteAirTime * speeds.vyMetersPerSecond;
        return robotDistanceCorrection;
    }

    public void updateWithVision(Pose2d pose2d, double timestamp) {
        Pose2d test = new Pose2d(pose2d.getTranslation(), Rotation2d.fromDegrees(correctedYaw()));
        swerveEstimator.addVisionMeasurement(test, timestamp);
        // limeLightSwerveEstimator.addVisionMeasurement(test, timestamp);
        // setHeading(Rotation2d.fromDegrees(gyroOffset-180));
        // swerveEstimator.resetPosition(getGyroYaw(), getModulePositions(), test);
    }

    public void updateWithVisionLLEsitmator(Pose2d pose2d, double timestamp) {
        Pose2d test = new Pose2d(pose2d.getTranslation(), Rotation2d.fromDegrees(correctedYaw()));
        // limeLightSwerveEstimator.addVisionMeasurement(test, timestamp);
        // setHeading(Rotation2d.fromDegrees(gyroOffset-180));
        // swerveEstimator.resetPosition(getGyroYaw(), getModulePositions(), test);
    }

    public void teleopDrive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getHeading())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        swerveEstimator.getEstimatedPosition().getRotation())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public void autoDrive(SwerveSample sample) {
        // Get the current pose of the robot
        Pose2d pose = getPose();

        // Generate the next speeds for the robot
        ChassisSpeeds speeds = new ChassisSpeeds(
                sample.vx + autoXController.calculate(pose.getX(), sample.x),
                sample.vy + autoYController.calculate(pose.getY(), sample.y),
                sample.omega + autoHeadingController.calculate(MathUtil.angleModulus(pose.getRotation().getRadians()),
                        MathUtil.angleModulus(sample.heading)));

        // SmartDashboard.putNumber("auto Measurement",
        // MathUtil.angleModulus(pose.getRotation().getRadians()));
        // SmartDashboard.putNumber("auto Setpoint",
        // MathUtil.angleModulus(sample.heading));

        // Apply the generated speeds
        drive(new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond), -speeds.omegaRadiansPerSecond,
                true, true);
    }

    public void setAutoDriveParams(Translation2d translation, double rotation, boolean fieldRelative,
            boolean isOpenLoop) {
        this.autoPickupDriveParams = new DriveParams(translation, rotation, fieldRelative, isOpenLoop);
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        // SmartDashboard.putNumber("Estimated x Pose",
        // swerveOdometry.getPoseMeters().getX());
        // SmartDashboard.putNumber("Estimated y Pose",
        // swerveOdometry.getPoseMeters().getY());
        return swerveEstimator.getEstimatedPosition();
        // return swerveEstimator.getEstimatedPosition();
    }

    // public Pose2d getAutoLimelightBotPose(){
    // if (f_Limelight.getArea() >= 0.17) {
    // // ChassisSpeeds speeds =
    // Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    // // if (Math.abs(speeds.vxMetersPerSecond) < .25 &&
    // Math.abs(speeds.vyMetersPerSecond) < 1){
    // // Pose2d visionPose = f_Limelight.getBotPose();
    // // updateWithVisionLLEsitmator(visionPose,
    // f_Limelight.getLastBotPoseTimestamp());
    // // }
    // Pose2d visionPose = f_Limelight.getBotPose();
    // updateWithVisionLLEsitmator(visionPose,
    // f_Limelight.getLastBotPoseTimestamp());
    // }

    // return limeLightSwerveEstimator.getEstimatedPosition();
    // }

    // public Pose2d getTeleopLimelightBotPose(){
    // if (f_Limelight.getArea() >= 0.17) {
    // // ChassisSpeeds speeds =
    // Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    // // if (Math.abs(speeds.vxMetersPerSecond) < .25 &&
    // Math.abs(speeds.vyMetersPerSecond) < 1){
    // // Pose2d visionPose = f_Limelight.getBotPose();
    // // updateWithVisionLLEsitmator(visionPose,
    // f_Limelight.getLastBotPoseTimestamp());
    // // }
    // Pose2d visionPose = f_Limelight.getBotPose();
    // updateWithVisionLLEsitmator(visionPose,
    // f_Limelight.getLastBotPoseTimestamp());
    // }

    // return limeLightSwerveEstimator.getEstimatedPosition();
    // }

    public Pose2d getLimelightBotPose() {
        if ((DriverStation.isDisabled() || !DriverStation.isAutonomous()) && f_Limelight.getArea() >= 0.18) {
            Pose2d visionPose = f_Limelight.getBotPose();
            updateWithVision(visionPose, f_Limelight.getLastBotPoseTimestamp());
        }
        return getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        swerveEstimator.resetPosition(getGyroRot2d(), getModulePositions(), pose);
        // limeLightSwerveEstimator.resetPosition(getGyroYaw(), getModulePositions(),
        // pose);
    }

    public Rotation2d getHeading() {
        return getGyroYaw();// swerveEstimator.getEstimatedPosition().getRotation();
    }

    public void setHeading(Rotation2d heading) {
        Rotation2d gyroYaw = getGyroYaw();
        swerveEstimator.resetPosition(getGyroRot2d(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), heading));
        // limeLightSwerveEstimator.resetPosition(gyroYaw, getModulePositions(), new
        // Pose2d(limeLightSwerveEstimator.getEstimatedPosition().getTranslation(),
        // heading));
        gyroOffset = gyroYaw.getDegrees() - heading.getDegrees();
        // gyro.setYaw(gyroOffset);
    }

    public void zeroHeading() {
        Rotation2d gyroYaw = getGyroRot2d();// getGyroYaw();
        // swerveEstimator.resetPosition(gyroYaw, getModulePositions(), new
        // Pose2d(getPose().getTranslation(), new Rotation2d()));
        gyroOffset = -gyroYaw.getDegrees();
        // gyro.setYaw(0);
        // swerveEstimator.resetPosition(getGyroRot2d(), getModulePositions(),
        // swerveEstimator.getEstimatedPosition());
        // LimelightHelpers.SetIMUMode("limelight-front", 1);
        // LimelightHelpers.SetRobotOrientation("limelight-front",
        // swerveEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0
        // ,0);

    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble()).rotateBy(Rotation2d.fromDegrees(gyroOffset));
    }

    public Rotation2d getGyroRot2d() {
        return gyro.getRotation2d();
    }

    public double getGyroRoll() {
        return gyro.getRoll().getValueAsDouble();
    }

    public double correctedYaw() {
        return (((gyro.getYaw().getValueAsDouble() - gyroOffset) % 360) + 360) % 360;
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public ChassisSpeeds getEstimatedFieldRelativeSpeeds() {
        ChassisSpeeds speed = Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
        speed = ChassisSpeeds.fromRobotRelativeSpeeds(speed, getHeading());
        return speed;
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {

        robotRelativeSpeeds.omegaRadiansPerSecond *= -1;

        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, .02);
        SwerveModuleState[] setpointStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, Constants.Swerve.maxSpeed);

        for (int i = 0; i < 4; i++) {
            mSwerveMods[i].setDesiredState(setpointStates[i], false);
        }
    }

    public Translation2d translateToApril() {
        double speedX;
        double speedY;
        if (!translationXController.atGoal()) {
            speedX = translationXController.calculate(swerveEstimator.getEstimatedPosition().getX(),
                    AlignPosition.getAlignOffset().getX());

        } else {
            speedX = 0;
        }

        if (!translationYController.atGoal()) {
            speedY = translationYController.calculate(swerveEstimator.getEstimatedPosition().getY(),
                    AlignPosition.getAlignOffset().getY());
        } else {
            speedY = 0;
        }
        // Added clamps
        // speedX = MathUtil.clamp(speedX, -0.17, 0.17);
        // speedY = MathUtil.clamp(speedY, -0.17, 0.17);
        return new Translation2d(speedX, speedY);

    }

    public double rotateToApril(double angle) {

        double output = (((angle - swerveEstimator.getEstimatedPosition().getRotation().getDegrees())) + 360) % 360;
        s_AutoRotateUtil.updateTargetAngle(output);

        SmartDashboard.putNumber("Auto Rotate Output", output);

        return s_AutoRotateUtil.calculateRotationSpeed();
    }

    // public double rotateToAmp() {
    // double headingError =
    // AlignPosition.getAlignOffset().getRotation().getDegrees() - correctedYaw();

    // s_AutoRotateUtil.updateTargetAngle(headingError);

    // return s_AutoRotateUtil.calculateRotationSpeed();
    // }

    public double rotateToNote() {
        boolean noteInView = f_Limelight.pieceDetected();

        noteInView = pieceSeenDebouncer.calculate(noteInView);

        if (noteInView) {
            s_AutoRotateUtil.updateTargetAngle(-f_Limelight.getX() / 2); // divided by 2 to account for latency
            return s_AutoRotateUtil.calculateRotationSpeed();
        }

        return 0;
    }

    public Translation2d noteTranslation() {
        double xValue = f_Limelight.getX(); // gets the limelight X Coordinate
        double areaValue = f_Limelight.getArea();
        // creating yTranslationPidController and setting the tolerance and setpoint
        yTranslationPidController = new PIDController(0, 0, 0);
        yTranslationPidController.setTolerance(1);
        yTranslationPidController.setSetpoint(0);

        // creating xTranslationPidController and setting the tolerance and setpoint
        xTranslationPidController = new PIDController(0, 0, 0);
        xTranslationPidController.setTolerance(0);
        xTranslationPidController.setSetpoint(0);

        if (pieceSeenDebouncer.calculate(f_Limelight.pieceDetected())) {

            // Calculates the x and y speed values for the translation movement
            double ySpeed = yTranslationPidController.calculate(xValue);
            double xSpeed = xTranslationPidController.calculate(areaValue);

            Translation2d translation = new Translation2d(xSpeed, ySpeed).times(Constants.Swerve.maxSpeed);

            return translation;
        } else {
            return new Translation2d(0, 0);
        }
    }

    public void alignAprilTag(AlignPosition alignPos, boolean isScoring) {

        AlignPosition.setPosition(alignPos, isScoring);

    }

    public void resetAlignApril() {
        Pose2d postion=swerveEstimator.getEstimatedPosition();
        translationXController.reset(postion.getX());
        translationYController.reset(postion.getY());
        s_AutoRotateUtil.reset();
    }

    public boolean alignAprilFinished() {
        return translationXController.atSetpoint() && translationYController.atSetpoint()
                && s_AutoRotateUtil.isFinished();
    }

    public void resetAutoRotateUtil() {
        s_AutoRotateUtil.end();
    }

    public void setLimelightOdometryMT2(String limelightName) {
        double robotAngle = swerveEstimator.getEstimatedPosition().getRotation().getDegrees();
        boolean doRejectUpdate = false;

        LimelightHelpers.SetIMUMode(limelightName, 0);
        LimelightHelpers.SetRobotOrientation(limelightName,
                    robotAngle , 0, 0, 0, 0, 0);

        // LimelightHelpers.SetRobotOrientation("limelight-front",
        // swerveEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0,
        // 0, 0);
        LimelightHelpers.PoseEstimate mt2Right = LimelightHelpers
                .getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        if (Math.abs(gyro.getAngularVelocityZWorld().getValueAsDouble()) > 720) // if our angular velocity is greater
                                                                                // than 720 degrees per second, ignore
                                                                                // vision updates
        {
            doRejectUpdate = true;
        }
        if (mt2Right == null || mt2Right.tagCount == 0) {
            doRejectUpdate = true;
        }
        if (!doRejectUpdate) {
            // Add logic here to check the distance to the april tag (getAprilTag() in the
            // relevant limelight subsystem, get the position of that id, etc.)

            swerveEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999));
            swerveEstimator.addVisionMeasurement(mt2Right.pose, mt2Right.timestampSeconds);
        }
    }

    public void setLimelightOdometryMT1(String limelightName) {
        boolean doRejectUpdate = false;
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        if (Math.abs(gyro.getAngularVelocityZWorld().getValueAsDouble()) > 720) // if our angular velocity is greater
                                                                                // than 720 degrees per second, ignore
                                                                                // vision updates
        {
            doRejectUpdate = true;
        }
        if (mt1 == null || mt1.tagCount == 0) {
            doRejectUpdate = true;
        }
        if (!doRejectUpdate) {
            // Add logic here to check the distance to the april tag (getAprilTag() in the
            // relevant limelight subsystem, get the position of that id, etc.)

            swerveEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999));
            swerveEstimator.addVisionMeasurement(mt1.pose, mt1.timestampSeconds);
        }

    }

    @Override
    public void periodic() {
        // swerveOdometry.update(getGyroYaw(), getModulePositions());
        // SmartDashboard.putNumber("Corrected gyro", correctedYaw());
        // SmartDashboard.putNumber("gyro", gyro.getYaw().getValueAsDouble());

        updatePoseEstimator();

        AlignPosition currentAlignPosition = AlignPosition.getPosition();

        // for(SwerveModule mod : mSwerveMods){
        //     mod.mDriveMotor.setControl(new MusicTone(1500));
        //     mod.mAngleMotor.setControl(new MusicTone(1500));
        // // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder",
        // //mod.getCANcoder().getDegrees());
        // }

        

        Pose2d estimatedPose = swerveEstimator.getEstimatedPosition();
        poseA = new Pose3d(estimatedPose);
        publisher.set(poseA);
        arrayPublisher.set(new Pose3d[] { poseA, poseB });

        double odometryX = estimatedPose.getX();
        double odometryY = estimatedPose.getY();
        field.setRobotPose(estimatedPose);
        SmartDashboard.putData("Field", field);
        SmartDashboard.putNumber("Odometry X", odometryX);
        SmartDashboard.putNumber("Odometry Y", odometryY);

        if (AlignPosition.getIsScoring()) {
            setLimelightOdometryMT2(Constants.limelightLeft);
            setLimelightOdometryMT2(Constants.limelightRight);
        } else {
            setLimelightOdometryMT1(Constants.limelightSky);
        }

    }

    public class DriveParams {

        public Translation2d translation;
        public double rotation;
        public boolean fieldRelative;
        public boolean isOpenLoop;

        public DriveParams(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
            this.translation = translation;
            this.rotation = rotation;
            this.fieldRelative = fieldRelative;
            this.isOpenLoop = isOpenLoop;
        }
    }
}
