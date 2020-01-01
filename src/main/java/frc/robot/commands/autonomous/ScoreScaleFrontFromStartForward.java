package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.motion.AutonomousPaths;
import frc.robot.motion.Path;
import frc.robot.motion.Trajectory;
import frc.robot.util.Side;

public class ScoreScaleFrontFromStartForward extends CommandGroup {
	/**
	 * Time from the end of the path that the elevator's position is set
	 */

	/**
	 * Time from the end of the path that the cube is launched
	 */
	private static final double LAUNCH_TIME = 0.5;

	public ScoreScaleFrontFromStartForward(Robot robot, Side startSide, Side scaleSide) {
		double launchSpeed;
		Path pathToScale;
		if (startSide == Side.LEFT) {
			if (scaleSide == Side.LEFT) {
				launchSpeed = 0.8;
				pathToScale = AutonomousPaths.LEFT_START_FORWARD_TO_LEFT_SCALE_FRONT;
			} else {
				launchSpeed = 0.9;
				pathToScale = AutonomousPaths.LEFT_START_FORWARD_TO_RIGHT_SCALE_FRONT;
			}
		} else {
			if (scaleSide == Side.LEFT) {
				launchSpeed = 0.9;
				pathToScale = AutonomousPaths.RIGHT_START_FORWARD_TO_LEFT_SCALE_FRONT;
			} else {
				launchSpeed = 0.8;
				pathToScale = AutonomousPaths.RIGHT_START_FORWARD_TO_RIGHT_SCALE_FRONT;
			}
		}

		Trajectory trajectoryToScale = new Trajectory(pathToScale, robot.getDrivetrain().getMaxAcceleration(), robot.getDrivetrain().getMaxVelocity());

		// Wait until x seconds are left in the path to set the elevator position

		// Wait until x seconds are left in the path to launch the cube.
		CommandGroup launchCubeGroup = new CommandGroup();
		launchCubeGroup.addSequential(new WaitCommand(Math.max(0, trajectoryToScale.getDuration() - LAUNCH_TIME)));

		addParallel(launchCubeGroup);
		addSequential(new FollowPathCommand(robot.getDrivetrain(), pathToScale));
	}
}
