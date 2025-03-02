package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.w3c.dom.css.RGBColor;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;


public class LEDSubsystem extends SubsystemBase {
    private CANdle candle1 = new CANdle(42);
    private RainbowAnimation rainbowAnimation = new RainbowAnimation(1, .8, 31);
    private TwinkleAnimation larsonAnimation = new TwinkleAnimation(65,105,225);

    private static LEDSubsystem ledSubsystem;
    private PivotSubsystem pivotSubsystem;
    private ExtensionSubsystem extensionSubsystem;
    private ManipulatorSubsystem manipulatorSubsystem;
    private IntakeSubsystem intakeSubsystem;

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
    }

    public void setRainbow() {
        candle1.animate(rainbowAnimation, 1);
    }

    public void setBlack() {
        candle1.setLEDs(0, 0, 0);
    }

    public void setColor(int r, int g, int b) {
        candle1.setLEDs(r, g, b);
    }

    @Override
    public void periodic() {
        // setColor(rgbColor[0], rgbColor[1], rgbColor[2]);
        // candle1.setLEDs(255, 255, 0);
        setRainbow();


    }
}
