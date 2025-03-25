package frc.robot.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmPosition;
import frc.robot.subsystems.ExtensionSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class ResetArm extends Command{

    public ResetArm() {
    }

    @Override
    public void initialize() {
        
        ArmPosition.setPosition(ArmPosition.StartingConfig);

    }

    @Override
    public boolean isFinished() {
        return ExtensionSubsystem.nearSetpointAuto();
    }


}
