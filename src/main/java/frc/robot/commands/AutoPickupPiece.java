package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.AutoRotateUtil;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.ArmPosition;
import frc.robot.autoCommands.ManipulateCoral;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.Swerve;

public class AutoPickupPiece extends Command {
    public LimeLightSubsystem limelightRight;
    public LimeLightSubsystem limelightLeft;
    public LimeLightSubsystem targetLimelight;

    public Swerve swerve;
    public IntakeSubsystem intakeSubsystem;
    public Debouncer debouncer;

    public double xValue;
    public double yValue;
    public double areaValue;

    public Rotation2d yaw;
    private AutoRotateUtil autoRotateUtil;

    public PIDController xTranslationPidController;

    public Pose2d position;

    public boolean hasPickedUpPiece = false;
    private boolean hasSeenPiece = false;

    public int waitFor = 0;
    public int counter = 0;
    public int detectedDelayCount = 0;


    public AutoPickupPiece(int waitFor) {
        swerve = Swerve.getInstance();
        intakeSubsystem = IntakeSubsystem.getInstance();
        autoRotateUtil = new AutoRotateUtil(0);
        this.waitFor = waitFor;

        limelightRight = LimeLightSubsystem.getRightInstance();
        limelightLeft = LimeLightSubsystem.getLeftInstance();

        targetLimelight = limelightRight;

        debouncer = new Debouncer(.5);

        // creating yTranslationPidController and setting the toleance and setpoint
        // yTranslationPidController = new PIDController(.2, 0, 0);
        // yTranslationPidController.setTolerance(1);
        // yTranslationPidController.setSetpoint(0);

        // creating xTranslationPidController and setting the toleance and setpoint
        xTranslationPidController = new PIDController(1.6, 0, 0);
        xTranslationPidController.setTolerance(1);
        xTranslationPidController.setSetpoint(0);

        // puts the value of P,I and D onto the SmartDashboard
        // Will remove later
        // SmartDashboard.putNumber("P", 0);
        // SmartDashboard.putNumber("i", 0);
        // SmartDashboard.putNumber("d", 0);
        // THESE VALUES WILL NEED TO BE MESSED WITH, 0 FOR NOW

        addRequirements(intakeSubsystem);
    }

    private boolean pieceSeen() {
        return targetLimelight.pieceDetected() && targetLimelight.isCoral();
    }

    private double getDistance(LimeLightSubsystem limelight) {
        double dX = 10000;
        double dY = 10000;

        if (limelight.pieceDetected()){
            dX = limelight.getX();
            dY = limelight.getY();
        }

        return Math.hypot(dX, dY * dY);
    }

    @Override
    public void initialize() {
        double leftDistance = getDistance(limelightLeft);
        double rightDistance = getDistance(limelightRight);

        ArmPosition.setPosition(ArmPosition.StartingConfig);
        // LimelightHelpers.Flush();

        if(leftDistance < rightDistance){
            targetLimelight = limelightLeft;
        } else {
            targetLimelight = limelightRight;
        }
    }

    @Override
    public void execute() {
        //kP = SmartDashboard.getNumber("AlignP", 0);
        //kI = SmartDashboard.getNumber("AlignI", 0);
        //kD = SmartDashboard.getNumber("AlignD", 0);

        SmartDashboard.putNumber("Xvalue", xValue);
        SmartDashboard.putNumber("Areavalue", areaValue);

        boolean pieceDetected = intakeSubsystem.hasPiece(); // piece is detected in the robot

        if (pieceDetected) {
            detectedDelayCount++;
        }

        counter++;

        if (debouncer.calculate(pieceSeen()) && !pieceDetected && counter >= waitFor && !hasSeenPiece) {
            xValue = targetLimelight.getX(); // gets the limelight X Coordinate
            yValue = MathUtil.clamp(targetLimelight.getY(), 0, 200); // gets the limelight Y Coordinate
            areaValue = targetLimelight.getArea(); // gets the area percentage from the limelight
            SmartDashboard.putNumber("Limelight Area", areaValue);
            autoRotateUtil.updateTargetAngle(-xValue * 2);

            if (isAligned()) {
                intakeSubsystem.setIntakeSpeed(-1);
                ArmPosition.setPosition(ArmPosition.Ground);
                System.out.println("It should be moving the arm right now");
                hasSeenPiece = true;
                swerve.drive(new Translation2d(0, 0), 0, true, true);
                return;
            }
            // System.out.println("Note in view");
            // Calculates the x and y speed values for the translation movement
            // double ySpeed = yTranslationPidController.calculate(xValue);

            double xSpeed = xTranslationPidController.calculate(yValue);
            double angularSpeed = autoRotateUtil.calculateRotationSpeed() * Constants.Swerve.maxAngularVelocity;

            // moves the swerve subsystem
            Translation2d translation = new Translation2d(xSpeed, 0);
            double rotation = angularSpeed;
            SmartDashboard.putNumber("rotation speed", rotation);

            swerve.drive(translation, rotation, false, true);
        } else {
            if (PivotSubsystem.atPosition() && ArmPosition.Ground.equals(ArmPosition.getPosition()) && isAligned()) {
                swerve.drive(new Translation2d(-0.15, 0).times(Constants.Swerve.maxSpeed), 0, false, true);
            }
        }
    }

    private boolean isAligned() {
        return yValue <= 1 && Math.abs(xValue) <= 3;
    }

    @Override
    public boolean isFinished() {
        // return false;
        return detectedDelayCount >= 5;
    }

    @Override
    public void end(boolean over) {
        swerve.drive(new Translation2d(0, 0), 0, true, true);
        intakeSubsystem.setIntakeSpeed(0);
        detectedDelayCount = 0;
        xTranslationPidController.reset();
        // yTranslationPidController.reset();
        autoRotateUtil.reset();

        ArmPosition.setPosition(ArmPosition.StartingConfig);

        hasSeenPiece = false;
        counter = 0;

        //TODO: go to StartingConfig after intaking :)

        // LimelightHelpers.Flush();
    }
}
