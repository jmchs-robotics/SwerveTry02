package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.HolonomicDrivetrain;

public class HolonomicDriveCommand extends Command {
	private final HolonomicDrivetrain mDrivetrain;

	//this is the default drive command for their drivetrain, inherited from HolonomicDrivetrain.java. SO FAR AWAY :(
	public HolonomicDriveCommand(HolonomicDrivetrain drivetrain) {
		mDrivetrain = drivetrain;

		requires(drivetrain);
	}

	private double deadband(double input) {
		if (Math.abs(input) < 0.05) return 0;
		return input;
	}

	//they use a quadratic drive system -- not linear. Worth noting.
	@Override
	protected void execute() {
		double forward = -Robot.getOI().getPrimaryController().getLeftYValue();
		double strafe = Robot.getOI().getPrimaryController().getLeftXValue();
		double rotation = Robot.getOI().getPrimaryController().getRightXValue();

		forward *= Math.abs(forward);
		strafe *= Math.abs(strafe);
		rotation *= Math.abs(rotation);

		forward = deadband(forward);
		strafe = deadband(strafe);
		rotation = deadband(rotation);

		// SmartDashboard.putNumber("Forward", forward);
		// SmartDashboard.putNumber("Strafe", strafe);
		// SmartDashboard.putNumber("Rotation", rotation);

		// by reading the isFieldOriented() we allow the left bumper to control if driving field oriented or not
		mDrivetrain.holonomicDrive(forward, strafe, rotation, mDrivetrain.isFieldOriented()); // false);
	}

	@Override
	protected void end() {
		mDrivetrain.stopDriveMotors();
	}

	@Override
	protected void interrupted() {
		end();
	}

	@Override
	protected boolean isFinished() {
		return false;
	}
}
