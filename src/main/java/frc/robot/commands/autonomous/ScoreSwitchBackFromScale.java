package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.*;
import frc.robot.motion.AutonomousPaths;
import frc.robot.motion.Path;
import frc.robot.motion.Trajectory;
import frc.robot.util.Side;

public class ScoreSwitchBackFromScale extends CommandGroup {
	private static final double INTAKE_TIME = 2.5;

	public ScoreSwitchBackFromScale(Robot robot, Side switchSide, Side scaleSide) {
        if (switchSide == Side.LEFT && scaleSide == Side.RIGHT) {
            addSequential(new FollowPathCommand(robot.getDrivetrain(), AutonomousPaths.RIGHT_SCALE_TO_LEFT_SCALE));
        } else if (switchSide == Side.RIGHT && scaleSide == Side.LEFT){
            addSequential(new FollowPathCommand(robot.getDrivetrain(), AutonomousPaths.LEFT_SCALE_TO_RIGHT_SCALE));
        }

	    Path pathToCube;
        if (scaleSide == Side.LEFT) {
            pathToCube = AutonomousPaths.LEFT_SCALE_TO_LEFT_CUBE;
        } else {
            pathToCube = AutonomousPaths.RIGHT_SCALE_TO_RIGHT_CUBE;
        }

        Trajectory trajectoryToCube = new Trajectory(pathToCube, robot.getDrivetrain().getMaxAcceleration(), robot.getDrivetrain().getMaxVelocity());

        CommandGroup intakeGroup = new CommandGroup();
        intakeGroup.addSequential(new WaitCommand(Math.max(0, trajectoryToCube.getDuration() - INTAKE_TIME)));

        addSequential(new VisionLineUpWithCubeCommand(robot, robot.rft_));
        addParallel(intakeGroup);
        addSequential(new FollowPathCommand(robot.getDrivetrain(), pathToCube));
	}
}
