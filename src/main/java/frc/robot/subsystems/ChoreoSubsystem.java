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
import frc.robot.Constants;

import java.sql.Driver;
import java.util.Optional;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.*;



public class ChoreoSubsystem extends SubsystemBase{
    
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
            getFlipped(),
            s_Swerve
        );

        autoFactory.bind("hi", new InstantCommand(() -> System.out.println("this is the bind")));
    }

    private boolean getFlipped() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        System.out.println(alliance);
        return alliance.isPresent() && alliance.get().equals(Alliance.Red);
    }

    private Pose2d getPose() {
        //s_Swerve.setPose(trajectory.getInitialPose());
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

public AutoRoutine pickupAndScoreAuto() {
    AutoRoutine routine = autoFactory.newRoutine("testAuto");

    // Load the routine's trajectories
    AutoTrajectory pickupTraj = routine.trajectory("TestAuto");

      

    // When the routine begins, reset odometry and start the first trajectory 
    routine.active().onTrue(
        Commands.sequence(
            pickupTraj.resetOdometry(),
            new InstantCommand(() -> s_Swerve.setHeading(pickupTraj.getInitialPose().get().getRotation())), //rotateBy(180);
            pickupTraj.cmd()
        )
    );

    // Starting at the event marker named "intake", run the intake 
    pickupTraj.atTime("hi").onTrue(new InstantCommand(() -> System.out.println("this is the atTime")));

    // When the trajectory is done, start the next trajectory
    pickupTraj.done().onTrue(new InstantCommand(() -> System.out.println("this is the end of the trajectory")));

    return routine;
}

}
