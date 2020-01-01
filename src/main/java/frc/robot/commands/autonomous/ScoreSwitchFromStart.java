package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.commands.autonomous.stage1.StartingPosition;
import frc.robot.motion.AutonomousPaths;
import frc.robot.util.Side;

public class ScoreSwitchFromStart extends CommandGroup {
	public ScoreSwitchFromStart(Robot robot, StartingPosition startingPosition, Side switchSide, StartingOrientation orientation) {
		switch (startingPosition) {
			case LEFT:
				// TODO: Switch from side
				break;
			case CENTER:
				if (switchSide == Side.LEFT) {
					if (orientation == StartingOrientation.FORWARDS) {
						addSequential(new FollowPathCommand(robot.getDrivetrain(), AutonomousPaths.CENTER_START_TO_LEFT_SWITCH));
					} else {
						throw new IllegalArgumentException("Cannot start in the center while oriented sideways");
					}
				} else {
					if (orientation == StartingOrientation.FORWARDS) {
						addSequential(new FollowPathCommand(robot.getDrivetrain(), AutonomousPaths.CENTER_START_TO_RIGHT_SWITCH));
					} else {
						throw new IllegalArgumentException("Cannot start in the center while oriented sideways");
					}
				}
				break;
			case RIGHT:
				// TODO: Switch from side
				break;
		}
	}
}