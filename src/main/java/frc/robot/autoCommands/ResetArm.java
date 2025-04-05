package frc.robot.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmPosition;
import frc.robot.subsystems.ExtensionSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class ResetArm extends Command{

    private boolean hasPause = false;
    private int counter = 25;

    private ArmPosition armPosition;

    public ResetArm(boolean hasPause) {
        this.hasPause = hasPause;
    }

    public ResetArm(ArmPosition armPosition) {
        this.armPosition = armPosition;
    }
    
    public ResetArm() {
        armPosition = ArmPosition.StartingConfig;
    }

    @Override
    public void initialize() {
        
        ArmPosition.setPosition(armPosition);

    }

    @Override
    public void execute() {
        if (ExtensionSubsystem.nearSetpointAuto()) {
            counter--;
        }
    }

    @Override
    public boolean isFinished() {
        return (ExtensionSubsystem.nearSetpointAuto() && !hasPause) || (hasPause && counter == 0);
    }


}
