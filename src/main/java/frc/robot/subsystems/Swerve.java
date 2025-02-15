package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
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




public class Swerve extends SubsystemBase{
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    private SwerveDrivePoseEstimator swerveEstimator;
    //private SwerveDrivePoseEstimator limeLightSwerveEstimator;
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
    //public ProfiledPIDController translationXController = new ProfiledPIDController(10, 0, 0, new TrapezoidProfile.Constraints(1, .5));
    //public ProfiledPIDController translationYController = new ProfiledPIDController(10, 0, 0, new TrapezoidProfile.Constraints(1, .5));

    public PIDController translationXController = new PIDController(0.1, 0, 0);
    public PIDController translationYController = new PIDController(0.1, 0, 0);

    private final PIDController autoXController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController autoYController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController autoHeadingController = new PIDController(8.0, 0.0, 0.0);

    public boolean readyToPickUp = false;
    public DriveParams autoPickupDriveParams;

    public static Swerve getInstance() {
        if (swerve == null) {
            swerve = new Swerve();
        }
        return swerve;
    }

    private Swerve() {
        pieceSeenDebouncer = new Debouncer(.1, Debouncer.DebounceType.kBoth);
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        //gyro.setYaw(0);
        f_Limelight = LimeLightSubsystem.getInstance();
        s_AutoRotateUtil = new AutoRotateUtil(0);
        field = new Field2d();

        autoHeadingController.enableContinuousInput(-Math.PI, Math.PI);

        translationXController.setTolerance(0.1);
        translationYController.setTolerance(0.1);
        
        publisher = NetworkTableInstance.getDefault().getStructTopic("MyPose", Pose3d.struct).publish();
        arrayPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("MyPoseArray", Pose3d.struct).publish();


        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        // swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());
        swerveEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getGyroRot2d(), getModulePositions(), new Pose2d(0, 0, new Rotation2d()));
        //limeLightSwerveEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getGyroRot2d(), getModulePositions(), new Pose2d(0, 0, new Rotation2d()));

        swerveEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(.5)));
        //limeLightSwerveEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(.5)));

    }
    public void updatePoseEstimator() {
        SwerveModulePosition[] getModPos = getModulePositions();
        // if (DriverStation.isAutonomousEnabled()) {
            
        // }

        swerveEstimator.updateWithTime(Timer.getFPGATimestamp(), getGyroRot2d(), getModPos);
        //limeLightSwerveEstimator.updateWithTime(Timer.getFPGATimestamp(), getGyroYaw(), getModPos);
    }

    public void setTrajectory(Trajectory<SwerveSample> traj) {
        field.getObject("traj").setTrajectory(TrajectoryGenerator.generateTrajectory(List.of(traj.getPoses()), new TrajectoryConfig(10000, 10000)));
    }

    public Pose2d getEstimatedPosition(){
        return swerveEstimator.getEstimatedPosition();
        // return swerveOdometry.getPoseMeters();
    }
    public double getVelocityCorrectionDistance(double distance, ChassisSpeeds speeds){
        double noteAirTime = distance / 10.415;
        double robotDistance = noteAirTime * speeds.vyMetersPerSecond;
        robotDistance = DriverStation.getAlliance().equals(Alliance.Red)? robotDistance * 0 : robotDistance;
        return robotDistance + distance;
    }
    public double getVelocityCorrection(double distance, ChassisSpeeds speeds){
        double noteAirTime = distance / 10.415;
        double robotDistanceCorrection = noteAirTime * speeds.vyMetersPerSecond;
        return robotDistanceCorrection;
    }

    public void updateWithVision(Pose2d pose2d, double timestamp){
        Pose2d test = new Pose2d(pose2d.getTranslation(), Rotation2d.fromDegrees(correctedYaw()));
        swerveEstimator.addVisionMeasurement(test, timestamp);
        //limeLightSwerveEstimator.addVisionMeasurement(test, timestamp);
        //setHeading(Rotation2d.fromDegrees(gyroOffset-180));
        // swerveEstimator.resetPosition(getGyroYaw(), getModulePositions(), test);
    }
    public void updateWithVisionLLEsitmator(Pose2d pose2d, double timestamp){
        Pose2d test = new Pose2d(pose2d.getTranslation(), Rotation2d.fromDegrees(correctedYaw()));
        //limeLightSwerveEstimator.addVisionMeasurement(test, timestamp);
        //setHeading(Rotation2d.fromDegrees(gyroOffset-180));
        // swerveEstimator.resetPosition(getGyroYaw(), getModulePositions(), test);
    }
    public void teleopDrive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    swerveEstimator.getEstimatedPosition().getRotation()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
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
            sample.omega + autoHeadingController.calculate(MathUtil.angleModulus(pose.getRotation().getRadians()), MathUtil.angleModulus(sample.heading))
        );

        SmartDashboard.putNumber("auto Measurement", MathUtil.angleModulus(pose.getRotation().getRadians()));
        SmartDashboard.putNumber("auto Setpoint", MathUtil.angleModulus(sample.heading));

        // Apply the generated speeds
        drive(new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond), -speeds.omegaRadiansPerSecond, true, true);
    }

    public void setAutoDriveParams(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        this.autoPickupDriveParams = new DriveParams(translation, rotation, fieldRelative, isOpenLoop);
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
    //     SmartDashboard.putNumber("Estimated x Pose", swerveOdometry.getPoseMeters().getX());
    //     SmartDashboard.putNumber("Estimated y Pose", swerveOdometry.getPoseMeters().getY());
        return swerveEstimator.getEstimatedPosition();
        // return swerveEstimator.getEstimatedPosition();
    }

    // public Pose2d getAutoLimelightBotPose(){
    //     if (f_Limelight.getArea() >= 0.17) {
    //         // ChassisSpeeds speeds = Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    //         // if (Math.abs(speeds.vxMetersPerSecond) < .25 && Math.abs(speeds.vyMetersPerSecond) < 1){
    //         //     Pose2d visionPose = f_Limelight.getBotPose();
    //         //     updateWithVisionLLEsitmator(visionPose, f_Limelight.getLastBotPoseTimestamp());
    //         // }
    //         Pose2d visionPose = f_Limelight.getBotPose();
    //         updateWithVisionLLEsitmator(visionPose, f_Limelight.getLastBotPoseTimestamp());
    //     }
        
    //     return limeLightSwerveEstimator.getEstimatedPosition();
    // }

    // public Pose2d getTeleopLimelightBotPose(){
    //     if (f_Limelight.getArea() >= 0.17) {
    //         // ChassisSpeeds speeds = Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    //         // if (Math.abs(speeds.vxMetersPerSecond) < .25 && Math.abs(speeds.vyMetersPerSecond) < 1){
    //         //     Pose2d visionPose = f_Limelight.getBotPose();
    //         //     updateWithVisionLLEsitmator(visionPose, f_Limelight.getLastBotPoseTimestamp());
    //         // }
    //         Pose2d visionPose = f_Limelight.getBotPose();
    //         updateWithVisionLLEsitmator(visionPose, f_Limelight.getLastBotPoseTimestamp());
    //     }
        
    //     return limeLightSwerveEstimator.getEstimatedPosition();
    // }

    public Pose2d getLimelightBotPose(){
        if ((DriverStation.isDisabled() || !DriverStation.isAutonomous()) && f_Limelight.getArea() >= 0.18) {
            Pose2d visionPose = f_Limelight.getBotPose();
            updateWithVision(visionPose, f_Limelight.getLastBotPoseTimestamp());
        }
        return getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        swerveEstimator.resetPosition(getGyroRot2d(), getModulePositions(), pose);
        //limeLightSwerveEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading(){
        return getGyroYaw();//swerveEstimator.getEstimatedPosition().getRotation();
    }

    public void setHeading(Rotation2d heading){
        Rotation2d gyroYaw = getGyroYaw();
        swerveEstimator.resetPosition(getGyroRot2d(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
        //limeLightSwerveEstimator.resetPosition(gyroYaw, getModulePositions(), new Pose2d(limeLightSwerveEstimator.getEstimatedPosition().getTranslation(), heading));
        gyroOffset = gyroYaw.getDegrees() - heading.getDegrees();
        //gyro.setYaw(gyroOffset);
    }

    public void zeroHeading(){
        Rotation2d gyroYaw = getGyroRot2d();//getGyroYaw();
        //swerveEstimator.resetPosition(gyroYaw, getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
        gyroOffset = -gyroYaw.getDegrees();
        //gyro.setYaw(0);
        //swerveEstimator.resetPosition(getGyroRot2d(), getModulePositions(), swerveEstimator.getEstimatedPosition());
        // LimelightHelpers.SetIMUMode("limelight-front", 1);
        // LimelightHelpers.SetRobotOrientation("limelight-front", swerveEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0 ,0);
        
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble()).rotateBy(Rotation2d.fromDegrees(gyroOffset));
    }

    public Rotation2d getGyroRot2d(){
        return gyro.getRotation2d();
    }

    public double getGyroRoll() {
        return gyro.getRoll().getValueAsDouble();
    }
    public double correctedYaw() {
        return (((gyro.getYaw().getValueAsDouble() - gyroOffset) % 360) + 360) % 360;
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }
    public ChassisSpeeds getRobotRelativeSpeeds(){
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }
    
    public ChassisSpeeds getEstimatedFieldRelativeSpeeds(){
        ChassisSpeeds speed = Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
        speed = ChassisSpeeds.fromRobotRelativeSpeeds(speed, getHeading());
        return speed;
    }
    
    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds){

        robotRelativeSpeeds.omegaRadiansPerSecond *= -1;

        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, .02);
        SwerveModuleState[] setpointStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, Constants.Swerve.maxSpeed);

        for(int i = 0; i < 4; i++){
            mSwerveMods[i].setDesiredState(setpointStates[i], false);
        }
    }

    public Translation2d translateToApril() {
        double speedX;
        double speedY;
        if (!translationXController.atSetpoint()) {
            speedX = translationXController.calculate(swerveEstimator.getEstimatedPosition().getX(), AlignPosition.getAlignOffset().getX());
        } else {
            speedX = 0;
        }
        
        if (!translationYController.atSetpoint()) {
            speedY = translationYController.calculate(swerveEstimator.getEstimatedPosition().getY(), AlignPosition.getAlignOffset().getY());
        } else {
            speedY = 0;
        }

        return new Translation2d(speedX, speedY);

    }

    public double rotateToApril(double angle){

        double output = (((angle - swerveEstimator.getEstimatedPosition().getRotation().getDegrees())) + 360) % 360;
        s_AutoRotateUtil.updateTargetAngle(output); 
        
        return s_AutoRotateUtil.calculateRotationSpeed();
    }

    // public double rotateToAmp() {
    //     double headingError = AlignPosition.getAlignOffset().getRotation().getDegrees() - correctedYaw();

    //     s_AutoRotateUtil.updateTargetAngle(headingError);
        
    //     return s_AutoRotateUtil.calculateRotationSpeed();
    // }

    public double rotateToNote(){
        boolean noteInView = f_Limelight.hasTag();

        noteInView = pieceSeenDebouncer.calculate(noteInView);

        if (noteInView){
            s_AutoRotateUtil.updateTargetAngle(-f_Limelight.getX() / 2); //divided by 2 to account for latency
            return s_AutoRotateUtil.calculateRotationSpeed();
        }
        
        return 0;
    }
    public Translation2d noteTranslation(){
        double xValue = f_Limelight.getX(); //gets the limelight X Coordinate
        double areaValue = f_Limelight.getArea();
         // creating yTranslationPidController and setting the tolerance and setpoint
         yTranslationPidController = new PIDController(0, 0, 0);
         yTranslationPidController.setTolerance(1);
         yTranslationPidController.setSetpoint(0);
         
         // creating xTranslationPidController and setting the tolerance and setpoint
         xTranslationPidController = new PIDController(0, 0, 0);
         xTranslationPidController.setTolerance(0);
         xTranslationPidController.setSetpoint(0);
 
        if (pieceSeenDebouncer.calculate(f_Limelight.hasTag())){

            // Calculates the x and y speed values for the translation movement
            double ySpeed = yTranslationPidController.calculate(xValue);
            double xSpeed = xTranslationPidController.calculate(areaValue);
            
            Translation2d translation = new Translation2d(xSpeed, ySpeed).times(Constants.Swerve.maxSpeed);

            return translation;
        }
        else{
            return new Translation2d(0,0);
        }
    }

    public void alignAprilTag(boolean isLeft) {

        double tagID = f_Limelight.getAprilValue();

        SmartDashboard.putNumber("April Tag Looking", tagID);

        // pigeon 2 gyro is the value for rotation
        // press the button and align a variable distance from the actual april tag forward and side to side and do that
        if (isLeft) {
            AlignPosition.setPosition(AlignPosition.LeftOffset);
        } else {
            AlignPosition.setPosition(AlignPosition.RightOffset);
        }

    }

    public void resetAlignApril() {
        translationXController.reset();
        translationYController.reset();
        s_AutoRotateUtil.reset();
    }

    public void resetAutoRotateUtil(){
        s_AutoRotateUtil.end();
    }
    
    @Override
    public void periodic(){
        //swerveOdometry.update(getGyroYaw(), getModulePositions());
        // SmartDashboard.putNumber("Corrected gyro", correctedYaw());
        // SmartDashboard.putNumber("gyro", gyro.getYaw().getValueAsDouble());

        updatePoseEstimator();

        // for(SwerveModule mod : mSwerveMods){
        //     // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
        // }
        Pose2d estimatedPose = swerveEstimator.getEstimatedPosition();
        poseA = new Pose3d(estimatedPose);
        publisher.set(poseA);
        arrayPublisher.set(new Pose3d[] {poseA, poseB});

        double odometryX = estimatedPose.getX();
        double odometryY = estimatedPose.getY();
        field.setRobotPose(estimatedPose);
        SmartDashboard.putData("Field", field);
        SmartDashboard.putNumber("Odometry X", odometryX);
        SmartDashboard.putNumber("Odometry Y", odometryY);
        
        boolean doRejectUpdate = false;
        // if (DriverStation.isEnabled()) {
        //     LimelightHelpers.SetIMUMode("limelight-front", 2);
        // } else if (DriverStation.isDisabled()) {
        //     LimelightHelpers.SetIMUMode("limelight-front", 1);
        //     LimelightHelpers.SetRobotOrientation("limelight-front", -swerveEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        // }

        LimelightHelpers.SetIMUMode("limelight-front", 0);
            LimelightHelpers.SetRobotOrientation("limelight-front", swerveEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        
        // LimelightHelpers.SetRobotOrientation("limelight-front", swerveEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front");
        if(Math.abs(gyro.getAngularVelocityZWorld().getValueAsDouble()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
        {
          doRejectUpdate = true;
        }
        if(mt2 == null || mt2.tagCount == 0)
        {
          doRejectUpdate = true;
        }
        if(!doRejectUpdate)
        {
            
            swerveEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7,0.7,0.7));
            swerveEstimator.addVisionMeasurement(
                mt2.pose,
                mt2.timestampSeconds);
            //SmartDashboard.putNumber("MT2 X", mt2.pose.getX());
            //SmartDashboard.putNumber("MT2 Y", mt2.pose.getY());

        }
    }

    public class DriveParams {

        public Translation2d translation;
        public double rotation;
        public boolean fieldRelative;
        public boolean isOpenLoop;

        public DriveParams(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop){
            this.translation = translation;
            this.rotation = rotation;
            this.fieldRelative = fieldRelative;
            this.isOpenLoop = isOpenLoop;
        }
    }
}
