package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.HolonomicDrivetrain;

public class ZeroDrivetrainGyroCommand extends Command {
	private HolonomicDrivetrain mDrivetrain;

	public ZeroDrivetrainGyroCommand(HolonomicDrivetrain drivetrain) {
		mDrivetrain = drivetrain;
	}

	@Override
	public void execute() {
		mDrivetrain.zeroGyro();
	}

	@Override
	protected boolean isFinished() {
		return true;
	}
}
