package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.SwerveDriveModule;

public class SwerveModuleCommand extends Command {

	private SwerveDriveModule mDriveModule;

	public SwerveModuleCommand(SwerveDriveModule driveModule) {
		this.mDriveModule = driveModule;

		requires(driveModule);
	}

	@Override
	protected void execute() {
	}

	@Override
	protected boolean isFinished() {
		return false;
	}
}
