package frc.robot.commands.autonomous.stage2;

import frc.robot.Robot;
import frc.robot.commands.autonomous.DriveForDistanceCommand;
import frc.robot.util.Side;

import edu.wpi.first.wpilibj.command.CommandGroup;

import static frc.robot.commands.autonomous.AutonomousConstants.*;

public class Stage2ScaleCommand extends CommandGroup{
	private final Robot robot;
	public Stage2ScaleCommand(Robot robot, Side side) {
		this.robot = robot;
		addSequential(new DriveForDistanceCommand(robot.getDrivetrain(),
				(side == Side.LEFT ? -1 : 1) * SCORE_SCALE, WALL_TO_SCALE - WALL_TO_PLATFORM_ZONE));
		addSequential(new DriveForDistanceCommand(robot.getDrivetrain(),
				(side == Side.LEFT ? 1 : -1) * SCORE_SCALE, -(WALL_TO_SCALE - WALL_TO_PLATFORM_ZONE)));
	}
}
