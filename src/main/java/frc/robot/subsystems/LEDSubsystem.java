package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmPosition;

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

    private double timer;
    private boolean isStrobing;

    private int[] rgbColor= {65, 105, 225};

    public static LEDSubsystem getInstance() {
        if (ledSubsystem == null) {
            ledSubsystem = new LEDSubsystem();
        }
        return ledSubsystem;
    }

    private LEDSubsystem() {
        pivotSubsystem = PivotSubsystem.getInstance();
        intakeSubsystem = IntakeSubsystem.getInstance();
        extensionSubsystem = ExtensionSubsystem.getInstance();
        manipulatorSubsystem = ManipulatorSubsystem.getInstance();
        candle1.setLEDs(0, 0, 0);
        candle1.configBrightnessScalar(1);
        candle1.clearAnimation(1);
        candle1.clearAnimation(2);

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
        && !currentArmPos.equals(ArmPosition.HumanP) && !currentArmPos.equals(ArmPosition.StartingConfig) && !currentArmPos.equals(ArmPosition.Manual) && !currentArmPos.equals(ArmPosition.Travel)) {
            setRainbow();
        } else if (intakeSubsystem.hasPiece()) {
            setColor(65, 105, 225);
        } else {
            setBlack();
        }

    }
}
