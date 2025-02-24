package frc.robot.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ManipulatorSubsystem;

public class ManipulateCoral extends Command{
    

    private ManipulatorSubsystem manipulatorSubsystem;

    private int timer = 0;

    private boolean intaking;

    private double speed = 1;

    public ManipulateCoral(boolean intaking) {

        this.intaking = intaking;

        manipulatorSubsystem = ManipulatorSubsystem.getInstance();

        if (intaking) {
            speed *= -1;
        }

        addRequirements(manipulatorSubsystem);
    }

    @Override
    public void execute() {
        
        manipulatorSubsystem.setIntakeSpeed(speed);
        
        timer ++;
    }

    @Override
    public boolean isFinished() {
        return timer >= 50;
    }

    @Override
    public void end(boolean isFinished) {
        manipulatorSubsystem.setIntakeSpeed(0);
    }


}
