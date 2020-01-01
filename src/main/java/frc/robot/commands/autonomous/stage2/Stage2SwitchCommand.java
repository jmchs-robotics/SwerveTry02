package frc.robot.commands.autonomous.stage2;


import frc.robot.Robot;
import frc.robot.commands.autonomous.AutonomousConstants;
import frc.robot.commands.autonomous.DriveForDistanceCommand;
import frc.robot.util.Side;

import edu.wpi.first.wpilibj.command.CommandGroup;

import static frc.robot.commands.autonomous.AutonomousConstants.SWITCH_DEPTH;
import static frc.robot.commands.autonomous.AutonomousConstants.WALL_TO_PLATFORM_ZONE;
import static frc.robot.commands.autonomous.AutonomousConstants.WALL_TO_SWITCH;

public class Stage2SwitchCommand extends CommandGroup{
	private final Robot robot;
	
	public Stage2SwitchCommand(Robot robot, Side side) {
		this.robot = robot;
		addSequential(new DriveForDistanceCommand(robot.getDrivetrain(), 0,
				(WALL_TO_SWITCH + SWITCH_DEPTH / 2) - WALL_TO_PLATFORM_ZONE));


		if(side == Side.LEFT) {
			addSequential(new DriveForDistanceCommand(robot.getDrivetrain(), -AutonomousConstants.SWITCH_SCORE_TO_SWITCH_WALL, 0));
		} else {
			addSequential(new DriveForDistanceCommand(robot.getDrivetrain(), AutonomousConstants.SWITCH_SCORE_TO_SWITCH_WALL, 0));
		}

		
		if(side == Side.LEFT) {
			addSequential(new DriveForDistanceCommand(robot.getDrivetrain(), AutonomousConstants.SWITCH_SCORE_TO_SWITCH_WALL, 0));
		} else {
			addSequential(new DriveForDistanceCommand(robot.getDrivetrain(), -AutonomousConstants.SWITCH_SCORE_TO_SWITCH_WALL, 0));
		}
		addSequential(new DriveForDistanceCommand(robot.getDrivetrain(), 0, WALL_TO_PLATFORM_ZONE - (WALL_TO_SWITCH + SWITCH_DEPTH / 2)));
	}

}
