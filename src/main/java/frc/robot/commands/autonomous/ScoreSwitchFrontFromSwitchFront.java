package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.motion.AutonomousPaths;
import frc.robot.motion.Path;
import frc.robot.motion.Trajectory;
import frc.robot.util.Side;

public class ScoreSwitchFrontFromSwitchFront extends CommandGroup {
    private static final double INTAKE_TIME = 2.5;
    private static final double LAUNCH_TIME = 0.5;
   // private static final double ELEVATOR_WAIT = 1;

    public ScoreSwitchFrontFromSwitchFront(Robot robot, Side switchSide) {

        Path pathToCube;
        Path pathToSwitch;
        if (switchSide == Side.LEFT) {
            pathToCube = AutonomousPaths.LEFT_SWITCH_FRONT_SECOND_CUBE_STEP_1;
            pathToSwitch = AutonomousPaths.LEFT_SWITCH_FRONT_SECOND_CUBE_STEP_2;
        } else {
            pathToCube = AutonomousPaths.RIGHT_SWITCH_FRONT_SECOND_CUBE_STEP_1;
            pathToSwitch = AutonomousPaths.RIGHT_SWITCH_FRONT_SECOND_CUBE_STEP_2;
        }

        Trajectory trajectoryToCube = new Trajectory(pathToCube, robot.getDrivetrain().getMaxAcceleration(), robot.getDrivetrain().getMaxVelocity());
        Trajectory trajectoryToSwitch = new Trajectory(pathToSwitch, robot.getDrivetrain().getMaxAcceleration(), robot.getDrivetrain().getMaxVelocity());

        CommandGroup intakeGroup = new CommandGroup();
        intakeGroup.addSequential(new WaitCommand(Math.max(0, trajectoryToCube.getDuration() - INTAKE_TIME / 2)));

        CommandGroup scoreGroup = new CommandGroup();
        scoreGroup.addSequential(new WaitCommand(Math.max(0, trajectoryToSwitch.getDuration() - LAUNCH_TIME)));


        addParallel(intakeGroup);
        addSequential(new FollowPathCommand(robot.getDrivetrain(), pathToCube,
                robot.getDrivetrain().getMaxAcceleration() * 0.6,
                robot.getDrivetrain().getMaxVelocity() * 0.6));
        addSequential(new FollowPathCommand(robot.getDrivetrain(), pathToSwitch,
                robot.getDrivetrain().getMaxAcceleration() * 0.6,
                robot.getDrivetrain().getMaxVelocity() * 0.6));
        addParallel(scoreGroup);
        addSequential(new SetDrivetrainAngleIfNotAngledCommand(robot.getDrivetrain(), 0));
    }
}