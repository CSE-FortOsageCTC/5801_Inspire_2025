package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.AlignPosition;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.ArmPosition;
import frc.robot.autoCommands.ManipulateCoral;
import frc.robot.autoCommands.PushForPoints;
import frc.robot.autoCommands.ResetArm;
import frc.robot.commands.AlignToApril;
import frc.robot.commands.AutoPickupPiece;
import frc.robot.commands.IntakeCommand;

import java.sql.Driver;
import java.util.Optional;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.*;

public class ChoreoSubsystem extends SubsystemBase {

    private Swerve s_Swerve;
    private AutoFactory autoFactory;

    private PIDController autoXPID = new PIDController(Constants.AutoConstants.kPXController, 0, 0);
    private PIDController autoYPID = new PIDController(Constants.AutoConstants.kPYController, 0, 0);
    private PIDController autoThetaPID = new PIDController(Constants.AutoConstants.kPThetaController, 0, 0);

    private static ChoreoSubsystem s_ChoreoSubsystem;

    private Trajectory<SwerveSample> trajectory;

    public static ChoreoSubsystem getInstance() {
        if (s_ChoreoSubsystem == null) {
            s_ChoreoSubsystem = new ChoreoSubsystem();
        }
        return s_ChoreoSubsystem;
    }

    public ChoreoSubsystem() {

        s_Swerve = Swerve.getInstance();

        autoFactory = new AutoFactory(
                this::getPose,
                this::setPose,
                this::autoDrive,
                isRed(),
                s_Swerve);

        // autoFactory.bind("ArmGround", new InstantCommand(() -> ArmPosition.setPosition(ArmPosition.Ground)));
        // autoFactory.bind("GroundIntake", new InstantCommand(() -> IntakeSubsystem.getInstance().setIntakeSpeed(-1, -1)));
    }

    private void switchPipelines(int pipeline) {

        LimeLightSubsystem.getLeftInstance().setPipeline(pipeline);
        LimeLightSubsystem.getRightInstance().setPipeline(pipeline);
    }

    private boolean isRed() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        System.out.println(alliance);
        return alliance.isPresent() && alliance.get().equals(Alliance.Red);
    }

    private Pose2d getPose() {
        // s_Swerve.setPose(trajectory.getInitialPose());
        // System.out.println(s_Swerve.getPose().getRotation().toString());
        return s_Swerve.getPose();
    }

    private void setPose(Pose2d pose) {
        s_Swerve.setPose(pose);
    }

    private void autoDrive(SwerveSample sample) {
        s_Swerve.autoDrive(sample);
    }

    public Command setupAutonomousChoreoPath(String traj) {

        return autoFactory.newRoutine(traj).cmd();

    }

    // public AutoRoutine onePieceAuto() {
    //     System.out.println("this is before the auto routine");
    //     AutoRoutine routine = autoFactory.newRoutine("onePiece");

    //     System.out.println("this is the top of the auto code");

    //     // Load the routine's trajectories
    //     AutoTrajectory traj_startToHG = routine.trajectory("startToHG");

    //     // When the routine begins, reset odometry and start the first trajectory
    //     routine.active().onTrue(
    //             Commands.sequence(
    //                     // traj_startToIJ.resetOdometry(),
    //                     // new InstantCommand(() ->
    //                     // ArmPosition.setPosition(ArmPosition.StartingConfig)),
    //                     new InstantCommand(() -> s_Swerve.setHeading(Rotation2d.fromDegrees(0))), // rotateBy(180);
    //                     traj_startToHG.cmd(),
    //                     new InstantCommand(() -> ArmPosition.setPosition(ArmPosition.L4)),
    //                     new AlignToApril(AlignPosition.LeftOffset, true, 0),
    //                     new ManipulateCoral(false),
    //                     new InstantCommand(() -> ArmPosition.setPosition(ArmPosition.StartingConfig))));

    //     return routine;
    // }

    // public AutoRoutine twoPieceIJAuto() {
    //     AutoRoutine routine = autoFactory.newRoutine("threePiece");

    //     // Load the routine's trajectories
    //     AutoTrajectory traj_startToIJ = routine.trajectory("startToIJ");
    //     AutoTrajectory traj_IJtoHP = routine.trajectory("IJToHP");
    //     AutoTrajectory traj_HPtoIJ = routine.trajectory("HPToIJ");
    //     AutoTrajectory traj_KLtoHP = routine.trajectory("KLtoHP");
    //     AutoTrajectory traj_HPtoKL = routine.trajectory("HPtoKL");

    //     ArmPosition scoringPosition = ArmPosition.L4;

    //     // When the routine begins, reset odometry and start the first trajectory
    //     routine.active().onTrue(
    //             Commands.sequence(
    //                     // traj_startToIJ.resetOdometry(), //rotateBy(180);
    //                     // new InstantCommand(() ->
    //                     // ArmPosition.setPosition(ArmPosition.StartingConfig)),
    //                     new InstantCommand(() -> s_Swerve.setHeading(Rotation2d.fromDegrees(0))),
    //                     traj_startToIJ.cmd(),
    //                     new InstantCommand(() -> ArmPosition.setPosition(ArmPosition.L4)),
    //                     new AlignToApril(AlignPosition.RightOffset, true, 0).withTimeout(3),
    //                     new ManipulateCoral(false),
    //                     new ResetArm(),
    //                     traj_IJtoHP.cmd(),
    //                     // new AlignToApril(AlignPosition.CenterOffset, false).withTimeout(3),
    //                     new AutoPickupPiece(0).withTimeout(4),
    //                     // new InstantCommand(() -> switchPipelines(0)),
    //                     new InstantCommand(() -> ArmPosition.setPosition(ArmPosition.StartingConfig)),
    //                     traj_HPtoKL.cmd(),
    //                     new InstantCommand(() -> s_Swerve.drive(new Translation2d(0, 0), 0, true, true)),
    //                     new InstantCommand(() -> ArmPosition.setPosition(ArmPosition.L4)),
    //                     new AlignToApril(AlignPosition.RightOffset, true, 30).withTimeout(4),
    //                     new ManipulateCoral(false),
    //                     new ResetArm(true),
    //                     traj_KLtoHP.cmd(),
    //                     // new AlignToApril(AlignPosition.CenterOffset, false),
    //                     new AutoPickupPiece(0).withTimeout(4),
    //                     new InstantCommand(() -> switchPipelines(0)),
    //                     new InstantCommand(() -> ArmPosition.setPosition(ArmPosition.StartingConfig)),
    //                     traj_HPtoKL.cmd(),
    //                     new InstantCommand(() -> s_Swerve.drive(new Translation2d(0, 0), 0, true, true)),
    //                     new InstantCommand(() -> ArmPosition.setPosition(ArmPosition.L4)),
    //                     new AlignToApril(AlignPosition.LeftOffset, true, 30),
    //                     new ManipulateCoral(false),
    //                     new ResetArm()));

    //     return routine;
    // }

    // public AutoRoutine twoPieceEFAuto() {
    //     AutoRoutine routine = autoFactory.newRoutine("threePiece");

    //     // Load the routine's trajectories
    //     AutoTrajectory traj_startToEF = routine.trajectory("startToEF");
    //     AutoTrajectory traj_EFtoHP = routine.trajectory("EFToHP");
    //     AutoTrajectory traj_HPtoEF = routine.trajectory("HPToEF");
    //     AutoTrajectory traj_CDtoHP = routine.trajectory("CDtoHP");
    //     AutoTrajectory traj_KLtoHP = routine.trajectory("KLtoHP");
    //     AutoTrajectory traj_HPtoCD = routine.trajectory("HPtoCD");

    //     ArmPosition scoringPosition = ArmPosition.L4;

    //     // When the routine begins, reset odometry and start the first trajectory
    //     routine.active().onTrue(
    //             Commands.sequence(
    //                     // traj_startToIJ.resetOdometry(), //rotateBy(180);
    //                     // new InstantCommand(() ->
    //                     // ArmPosition.setPosition(ArmPosition.StartingConfig)),
    //                     new InstantCommand(() -> s_Swerve.setHeading(Rotation2d.fromDegrees(0))),
    //                     traj_startToEF.cmd(),
    //                     new InstantCommand(() -> ArmPosition.setPosition(ArmPosition.L4)),
    //                     new AlignToApril(AlignPosition.LeftOffset, true, 0).withTimeout(3),
    //                     new ManipulateCoral(false),
    //                     new ResetArm(),
    //                     traj_EFtoHP.cmd(),
    //                     // new AlignToApril(AlignPosition.CenterOffset, false).withTimeout(3),
    //                     new AutoPickupPiece(0).withTimeout(4),
    //                     // new InstantCommand(() -> switchPipelines(0)),
    //                     new InstantCommand(() -> ArmPosition.setPosition(ArmPosition.StartingConfig)),
    //                     traj_HPtoCD.cmd(),
    //                     new InstantCommand(() -> s_Swerve.drive(new Translation2d(0, 0), 0, true, true)),
    //                     new InstantCommand(() -> ArmPosition.setPosition(ArmPosition.L4)),
    //                     new AlignToApril(AlignPosition.RightOffset, true, 30).withTimeout(4),
    //                     new ManipulateCoral(false),
    //                     new ResetArm(),
    //                     traj_CDtoHP.cmd(),
    //                     // new AlignToApril(AlignPosition.CenterOffset, false),
    //                     new AutoPickupPiece(0).withTimeout(4),
    //                     new InstantCommand(() -> switchPipelines(0)),
    //                     new InstantCommand(() -> ArmPosition.setPosition(ArmPosition.StartingConfig)),
    //                     traj_HPtoCD.cmd(),
    //                     new InstantCommand(() -> s_Swerve.drive(new Translation2d(0, 0), 0, true, true)),
    //                     new InstantCommand(() -> ArmPosition.setPosition(ArmPosition.L4)),
    //                     new AlignToApril(AlignPosition.LeftOffset, true, 30),
    //                     new ManipulateCoral(false),
    //                     new ResetArm()));

    //     return routine;
    // }

    // public AutoRoutine twoPieceEFPush() {
    //     AutoRoutine routine = autoFactory.newRoutine("threePiece");

    //     // Load the routine's trajectories
    //     AutoTrajectory traj_startToEF = routine.trajectory("startToEF");
    //     AutoTrajectory traj_EFtoHP = routine.trajectory("EFToHP");
    //     AutoTrajectory traj_HPtoEF = routine.trajectory("HPToEF");
    //     AutoTrajectory traj_CDtoHP = routine.trajectory("CDtoHP");
    //     AutoTrajectory traj_KLtoHP = routine.trajectory("KLtoHP");
    //     AutoTrajectory traj_HPtoCD = routine.trajectory("HPtoCD");

    //     ArmPosition scoringPosition = ArmPosition.L4;

    //     // When the routine begins, reset odometry and start the first trajectory
    //     routine.active().onTrue(
    //             Commands.sequence(
    //                     // traj_startToIJ.resetOdometry(), //rotateBy(180);
    //                     // new InstantCommand(() ->
    //                     // ArmPosition.setPosition(ArmPosition.StartingConfig)),
    //                     new PushForPoints(),
    //                     new InstantCommand(() -> s_Swerve.setHeading(Rotation2d.fromDegrees(0))),
    //                     traj_startToEF.cmd(),
    //                     new InstantCommand(() -> ArmPosition.setPosition(ArmPosition.L4)),
    //                     new AlignToApril(AlignPosition.LeftOffset, true, 0).withTimeout(3),
    //                     new ManipulateCoral(false),
    //                     new ResetArm(),
    //                     traj_EFtoHP.cmd(),
    //                     // new AlignToApril(AlignPosition.CenterOffset, false).withTimeout(3),
    //                     new AutoPickupPiece(0).withTimeout(4),
    //                     // new InstantCommand(() -> switchPipelines(0)),
    //                     new InstantCommand(() -> ArmPosition.setPosition(ArmPosition.StartingConfig)),
    //                     traj_HPtoCD.cmd(),
    //                     new InstantCommand(() -> s_Swerve.drive(new Translation2d(0, 0), 0, true, true)),
    //                     new InstantCommand(() -> ArmPosition.setPosition(ArmPosition.L4)),
    //                     new AlignToApril(AlignPosition.RightOffset, true, 30).withTimeout(4),
    //                     new ManipulateCoral(false),
    //                     new ResetArm(),
    //                     traj_CDtoHP.cmd(),
    //                     // new AlignToApril(AlignPosition.CenterOffset, false),
    //                     new AutoPickupPiece(0).withTimeout(4),
    //                     new InstantCommand(() -> switchPipelines(0)),
    //                     new InstantCommand(() -> ArmPosition.setPosition(ArmPosition.StartingConfig)),
    //                     traj_HPtoCD.cmd(),
    //                     new InstantCommand(() -> s_Swerve.drive(new Translation2d(0, 0), 0, true, true)),
    //                     new InstantCommand(() -> ArmPosition.setPosition(ArmPosition.L4)),
    //                     new AlignToApril(AlignPosition.LeftOffset, true, 30),
    //                     new ManipulateCoral(false),
    //                     new ResetArm()));

    //     return routine;
    // }

    // public AutoRoutine twoPieceIJPush() {
    //     AutoRoutine routine = autoFactory.newRoutine("threePiece");

    //     // Load the routine's trajectories
    //     AutoTrajectory traj_startToIJ = routine.trajectory("startToIJ");
    //     AutoTrajectory traj_IJtoHP = routine.trajectory("IJToHP");
    //     AutoTrajectory traj_HPtoIJ = routine.trajectory("HPToIJ");
    //     AutoTrajectory traj_KLtoHP = routine.trajectory("KLtoHP");
    //     AutoTrajectory traj_HPtoKL = routine.trajectory("HPtoKL");

    //     ArmPosition scoringPosition = ArmPosition.L4;

    //     // When the routine begins, reset odometry and start the first trajectory
    //     routine.active().onTrue(
    //             Commands.sequence(
    //                     // traj_startToIJ.resetOdometry(), //rotateBy(180);
    //                     // new InstantCommand(() ->
    //                     // ArmPosition.setPosition(ArmPosition.StartingConfig)),
    //                     new PushForPoints(),
    //                     new InstantCommand(() -> s_Swerve.setHeading(Rotation2d.fromDegrees(0))),
    //                     traj_startToIJ.cmd(),
    //                     new InstantCommand(() -> ArmPosition.setPosition(ArmPosition.L4)),
    //                     new AlignToApril(AlignPosition.RightOffset, true, 0).withTimeout(3),
    //                     new ManipulateCoral(false),
    //                     new ResetArm(),
    //                     traj_IJtoHP.cmd(),
    //                     // new AlignToApril(AlignPosition.CenterOffset, false).withTimeout(3),
    //                     new AutoPickupPiece(0).withTimeout(4),
    //                     // new InstantCommand(() -> switchPipelines(0)),
    //                     new InstantCommand(() -> ArmPosition.setPosition(ArmPosition.StartingConfig)),
    //                     traj_HPtoKL.cmd(),
    //                     new InstantCommand(() -> s_Swerve.drive(new Translation2d(0, 0), 0, true, true)),
    //                     new InstantCommand(() -> ArmPosition.setPosition(ArmPosition.L4)),
    //                     new AlignToApril(AlignPosition.RightOffset, true, 30).withTimeout(4),
    //                     new ManipulateCoral(false),
    //                     new ResetArm(true),
    //                     traj_KLtoHP.cmd(),
    //                     // new AlignToApril(AlignPosition.CenterOffset, false),
    //                     new AutoPickupPiece(0).withTimeout(4),
    //                     new InstantCommand(() -> switchPipelines(0)),
    //                     new InstantCommand(() -> ArmPosition.setPosition(ArmPosition.StartingConfig)),
    //                     traj_HPtoKL.cmd(),
    //                     new InstantCommand(() -> s_Swerve.drive(new Translation2d(0, 0), 0, true, true)),
    //                     new InstantCommand(() -> ArmPosition.setPosition(ArmPosition.L4)),
    //                     new AlignToApril(AlignPosition.LeftOffset, true, 30),
    //                     new ManipulateCoral(false),
    //                     new ResetArm()));

    //     return routine;
    // }

    public AutoRoutine lollipopEFAuto() {
        // System.out.println("this is before the auto routine");
        AutoRoutine routine = autoFactory.newRoutine("threePiece");

        // System.out.println("this is the top of the auto code");

        // Load the routine's trajectories
        AutoTrajectory traj_startToEF = routine.trajectory("startToEF");
        AutoTrajectory traj_EFto3 = routine.trajectory("EFto3");

        // When the routine begins, reset odometry and start the first trajectory
        routine.active().onTrue(
            Commands.sequence(
                // traj_startToIJ.resetOdometry(),
                // new InstantCommand(() ->
                // ArmPosition.setPosition(ArmPosition.StartingConfig)),
                new InstantCommand(() -> s_Swerve.setHeading(Rotation2d.fromDegrees(0))),
                traj_startToEF.cmd().withTimeout(1),
                new InstantCommand(() -> ArmPosition.setPosition(ArmPosition.L4)),
                new AlignToApril(AlignPosition.LeftOffset, true, 0).withTimeout(0.1),
                new AlignToApril(AlignPosition.LeftOffset, true, 0).withTimeout(0.1),
                new AlignToApril(AlignPosition.LeftOffset, true, 0).withTimeout(3),
                new ManipulateCoral(false),
                new InstantCommand(() -> ArmPosition.setPosition(ArmPosition.StartingConfig)),
                traj_EFto3.cmd(),
                new InstantCommand(() -> IntakeSubsystem.getInstance().setIntakeSpeed(0, 0))

        ));
        return routine;
    }

}
