package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Acquisition;

/**
 * Lowers the acquisition then runs it while a toggle is held 
 */
public class ToggleAcq extends CommandBase {
    Acquisition acquisition;

    public ToggleAcq(Acquisition acquisition) {
        this.acquisition = acquisition;
        addRequirements(acquisition);
    }

    @Override
    public void execute() {
        if (acquisition.isLowered()) {
            acquisition.moveAcq(0);
            acquisition.setLowered(true);
        } else {
            acquisition.moveAcq(10);
            acquisition.setLowered(false);
        }
    }
}
