package frc.robot;



import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Swerve;

public class AutoRotateUtil {

    private final Swerve s_Swerve = null;

    private final PIDController pidController;

    private double m_angle;

    public AutoRotateUtil(double angle) {

        this.m_angle = angle == 0?360:angle;

        //SmartDashboard.putNumber("Angle", this.m_angle);

        this.pidController = new PIDController(0, 0, 0);

        pidController.setTolerance(0.001);
        pidController.setSetpoint(0);
        // SmartDashboard.putNumber("kP", 0.01);
        // SmartDashboard.putNumber("kI", 0);
        // SmartDashboard.putNumber("kD", 0);
   }

   public void initialize() {
    pidController.reset();
   }

   public void reset() {
    pidController.reset();
   }


   public double calculateRotationSpeed () {
    
    // double kP = SmartDashboard.getNumber("kP", 0.0);
    // double kI = SmartDashboard.getNumber("kI", 0.0);
    // double kD = SmartDashboard.getNumber("kD", 0.0);

    this.pidController.setP(.01);
    this.pidController.setI(0);
    this.pidController.setD(0);

    double headingError = this.m_angle % 360;
    if (headingError > 180) {
        headingError -= 360;
    }
    if (headingError < -180) {
        headingError += 360;
    }
    //SmartDashboard.putNumber("Heading Error Swerve", headingError);
    //double speed = pidController.calculate(headingError, 0);
    double feedForward = 0.5;
    //speed = MathUtil.clamp(speed, -1, 1);
    //SmartDashboard.putNumber("Speed", speed);

    if (Math.abs(headingError) > Constants.feedForwardAngle) {
        return (headingError < 0) ? feedForward : -feedForward;
    } else {
        return MathUtil.clamp(pidController.calculate(headingError, 0), -1, 1);
    }
   }
   /**
    * Updates degrees robot needs to rotate
    */ 
   public void updateTargetAngle(double angle) {
    //System.out.println(angle);
    m_angle = angle;

   }

   public boolean isFinished () {
    return pidController.atSetpoint();
    //double speed = pidController.calculate(s_Swerve.getYawDouble());
    //return speed < 0.1;
   }

   public void end() {
    System.out.println("it ended");
    pidController.reset();
   }

}