package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
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

public class AutoPopPickup extends Command {
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

    public PIDController inUsePID;
    public PIDController autoXTranslationPID;
    public PIDController rotationPID;


    public Pose2d position;

    public boolean hasPickedUpPiece = false;
    private boolean hasSeenPiece = false;

    public int waitFor = 0;
    public int counter = 0;
    public int detectedDelayCount = 0;




    public AutoPopPickup(int waitFor) {
        swerve = Swerve.getInstance();
        intakeSubsystem = IntakeSubsystem.getInstance();
        autoRotateUtil = new AutoRotateUtil(0);
        this.waitFor = waitFor;

        limelightRight = LimeLightSubsystem.getRightInstance();
        limelightLeft = LimeLightSubsystem.getLeftInstance();

        targetLimelight = limelightRight;

        debouncer = new Debouncer(.05);

        // creating yTranslationPidController and setting the toleance and setpoint
        // yTranslationPidController = new PIDController(.2, 0, 0);
        // yTranslationPidController.setTolerance(1);
        // yTranslationPidController.setSetpoint(0);

        // creating xTranslationPidController and setting the toleance and setpoint
        inUsePID = new PIDController(0.3, 0, 0);
        inUsePID.setTolerance(1);
        inUsePID.setSetpoint(-8);

        rotationPID = new PIDController(.02, 0, 0);

        rotationPID.setTolerance(1.5);
        rotationPID.setSetpoint(0);

        // puts the value of P,I and D onto the SmartDashboard
        // Will remove later
        // SmartDashboard.putNumber("P", 0);
        // SmartDashboard.putNumber("i", 0);
        // SmartDashboard.putNumber("d", 0);
        // THESE VALUES WILL NEED TO BE MESSED WITH, 0 FOR NOW

        addRequirements(intakeSubsystem);
    }

    private boolean pieceSeen() {
        return targetLimelight.isPopsicle();
    }

    private double getDistance(LimeLightSubsystem limelight) {
        double dX = 10000;
        double dY = 10000;

        if (limelight.isPopsicle()){
            dX = limelight.getX();
            dY = limelight.getY();
        }

        return Math.hypot(dX, dY);
    }

    @Override
    public void initialize() {
        limelightLeft.setPipeline(2);
        limelightRight.setPipeline(2);

        double leftDistance = getDistance(limelightLeft);
        double rightDistance = getDistance(limelightRight);

        ArmPosition.setPosition(ArmPosition.GroundP);
        // LimelightHelpers.Flush();

        if(leftDistance < rightDistance){
            targetLimelight = limelightLeft;
        } else {
            targetLimelight = limelightRight;
        }

        System.out.println(targetLimelight.limelightString);

        swerve.drive(new Translation2d(0, 0), 0, true, true);
    }

    @Override
    public void execute() {
        //kP = SmartDashboard.getNumber("AlignP", 0);
        //kI = SmartDashboard.getNumber("AlignI", 0);
        //kD = SmartDashboard.getNumber("AlignD", 0);

        // SmartDashboard.putNumber("Xvalue", xValue);
        // SmartDashboard.putNumber("Areavalue", areaValue);

        boolean pieceDetected = intakeSubsystem.hasPiece(); // piece is detected in the robot

        if (pieceDetected) {
            System.out.println("Has piece :D");
            detectedDelayCount++;
        }

        counter++;

        if (debouncer.calculate(pieceSeen()) && !pieceDetected && counter >= waitFor && !hasSeenPiece) {
            xValue = targetLimelight.getX(); // gets the limelight X Coordinate
            yValue = MathUtil.clamp(targetLimelight.getY(), -20, 200); // gets the limelight Y Coordinate
            areaValue = targetLimelight.getArea(); // gets the area percentage from the limelight
            // SmartDashboard.putNumber("Limelight Area", areaValue);
            
            

            //System.out.println(xValue);

            double angularSpeed = rotationPID.calculate(-xValue);

            if (isYAligned() && isXAligned()) {
                intakeSubsystem.setIntakeSpeed(Constants.coralIntakeSpeed, Constants.algaeIntakeSpeed);
                ArmPosition.setPosition(ArmPosition.Ground);
                System.out.println("It should be moving the arm right now");
                hasSeenPiece = true;
                swerve.drive(new Translation2d(0, 0), 0, true, true);
                return;
            }
            // System.out.println("Note in view");
            // Calculates the x and y speed values for the translation movement
            // double ySpeed = yTranslationPidController.calculate(xValue);

            
            double xSpeed = inUsePID.calculate(-yValue);

            // System.out.println(isYAligned());

            

            // moves the swerve subsystem

            Translation2d translation = new Translation2d(xSpeed, 0);
            
            double rotation = angularSpeed * Constants.Swerve.maxAngularVelocity;
            // SmartDashboard.putNumber("rotation speed", rotation);
            // if (!isYAligned() && !isXAligned()) {
            //     swerve.drive(translation, rotation, false, true);
            // } else if (isYAligned() && !isXAligned()) {
            //     swerve.drive(new Translation2d(0, 0), rotation, false, true);
            // } else if (!isYAligned() && isXAligned()) {
            //     swerve.drive(translation, 0, false, true);
            // } else if (isYAligned() && isXAligned()) {
            //     swerve.drive(new Translation2d(0, 0), 0, false, true);
            // }
            if (!isYAligned()) {
                swerve.drive(translation, rotation, false, true);
            } else if (isYAligned() && !isXAligned()) {
                swerve.drive(new Translation2d(0, 0), rotation, false, true);
            } else if (isYAligned() && isXAligned()) {
                swerve.drive(new Translation2d(0, 0), 0, false, true);
            }
            
        } else {
            if (ArmPosition.Ground.equals(ArmPosition.getPosition()) && isYAligned() && isXAligned()) {
                //System.out.println("is creeping forward");
                swerve.drive(new Translation2d(-0.22, 0).times(Constants.Swerve.maxSpeed), 0, false, true);
            } else {
                swerve.drive(new Translation2d(0, 0).times(Constants.Swerve.maxSpeed), 0, false, true);
            }
        }
    }

    private boolean isYAligned() {
        // System.out.println(yValue);
        return yValue >= -8;
    }

    private boolean isXAligned() {
        return rotationPID.atSetpoint();//Math.abs(xValue) <= 2;
    }

    @Override
    public boolean isFinished() {
        // return false;
        return detectedDelayCount >= 1;
    }

    @Override
    public void end(boolean over) {
        swerve.drive(new Translation2d(0, 0), 0, true, true);
        intakeSubsystem.setIntakeSpeed(0,0);
        detectedDelayCount = 0;
        inUsePID.reset();
        // yTranslationPidController.reset();
        autoRotateUtil.reset();

        ArmPosition.setPosition(ArmPosition.StartingConfig);

        hasSeenPiece = false;
        counter = 0;
        limelightLeft.setPipeline(0);
        limelightRight.setPipeline(0);
    

        //TODO: go to StartingConfig after intaking :)

        // LimelightHelpers.Flush();
    }
}
