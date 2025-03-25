package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmPosition;

import java.sql.Driver;

import org.w3c.dom.css.RGBColor;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;


public class LEDSubsystem extends SubsystemBase {
    private CANdle candle1 = new CANdle(42);
    private RainbowAnimation rainbowAnimation = new RainbowAnimation(1, .8, 31);
    private TwinkleAnimation larsonAnimation = new TwinkleAnimation(65,105,225);
    private StrobeAnimation strobeAnimation = new StrobeAnimation(65, 105, 225, 255, 0.2,31);

    private static LEDSubsystem ledSubsystem;
    private PivotSubsystem pivotSubsystem;
    private ExtensionSubsystem extensionSubsystem;
    private ManipulatorSubsystem manipulatorSubsystem;
    private IntakeSubsystem intakeSubsystem;

    private Debouncer isCoralDebouncerLeft;
    private Debouncer isCoralDebouncerRight;

    private double timer;
    private boolean isStrobing;

    private boolean isRed;

    private int[] fullBlue= {0, 0, 255};

    private int[] fullRed = {255, 0, 0};

    public static LEDSubsystem getInstance() {
        if (ledSubsystem == null) {
            ledSubsystem = new LEDSubsystem();
        }
        return ledSubsystem;
    }

    private LEDSubsystem() {
        isRed = DriverStation.getAlliance().get().equals(Alliance.Red);
        pivotSubsystem = PivotSubsystem.getInstance();
        intakeSubsystem = IntakeSubsystem.getInstance();
        extensionSubsystem = ExtensionSubsystem.getInstance();
        manipulatorSubsystem = ManipulatorSubsystem.getInstance();
        candle1.setLEDs(0, 0, 0);
        candle1.configBrightnessScalar(1);
        candle1.clearAnimation(1);
        candle1.clearAnimation(2);

        isCoralDebouncerLeft = new Debouncer(0.05);
        isCoralDebouncerRight = new Debouncer(0.05);

        timer = 0;
        isStrobing = false;
    }

    public void setStrobe() {
        isStrobing = true;
        timer++;
        candle1.animate(strobeAnimation, 1);
        if (timer > 40) {
            isStrobing = false;
            timer = 0;
        }
    }

    public void setRainbow() {
        candle1.animate(rainbowAnimation, 1);
    }

    public void setBlack() {
        candle1.clearAnimation(1);
        candle1.setLEDs(0, 0, 0);
        timer = 0;
    }

    public void setColor(int r, int g, int b) {
        candle1.clearAnimation(1);
        candle1.setLEDs(r, g, b);
        timer = 0;
    }

    @Override
    public void periodic() {
        // setColor(rgbColor[0], rgbColor[1], rgbColor[2]);
        // candle1.setLEDs(255, 255, 0);

        ArmPosition currentArmPos = ArmPosition.getPosition();
        if (isStrobing) {
            setStrobe();
        } else if (PivotSubsystem.nearSetpoint() && ManipulatorSubsystem.nearSetpoint() && ExtensionSubsystem.nearSetpoint()
        && !currentArmPos.equals(ArmPosition.Ground) && !currentArmPos.equals(ArmPosition.StartingConfig) && !currentArmPos.equals(ArmPosition.Manual) && !currentArmPos.equals(ArmPosition.Travel)
        && !currentArmPos.equals(ArmPosition.GroundP)) {
            setRainbow();
        } else if (intakeSubsystem.hasPiece()) {
            
            if (isRed) {
                setColor(fullRed[0], fullRed[1], fullRed[2]);
            } else {
                setColor(fullBlue[0], fullBlue[1], fullBlue[2]);
            }
        } else if (isCoralDebouncerLeft.calculate(LimeLightSubsystem.getLeftInstance().isCoral()) || isCoralDebouncerRight.calculate(LimeLightSubsystem.getRightInstance().isCoral())) {
            setColor(241, 250, 61); // Team "Safety Green" *eyeroll*
        } else {
            setBlack();
        }

    }
}
