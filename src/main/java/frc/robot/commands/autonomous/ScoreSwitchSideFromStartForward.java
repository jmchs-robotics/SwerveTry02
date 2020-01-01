package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.motion.AutonomousPaths;
import frc.robot.motion.Path;
import frc.robot.util.Side;

public class ScoreSwitchSideFromStartForward extends CommandGroup {
    public ScoreSwitchSideFromStartForward(Robot robot, Side startSide, Side switchSide) {
        Path pathToSwitch;
        Path pathToSwitchSide = switchSide == Side.LEFT ? AutonomousPaths.LEFT_SWITCH_TO_LEFT_SWITCH_SIDE
                : AutonomousPaths.RIGHT_SWITCH_TO_RIGHT_SWITCH_SIDE;
        double switchAngle = switchSide == Side.LEFT ? 270 : 90;

        if (startSide == Side.LEFT) {
            if (switchSide == Side.LEFT) {
                pathToSwitch = AutonomousPaths.LEFT_START_TO_LEFT_SWITCH;
            } else {
                pathToSwitch = AutonomousPaths.LEFT_START_TO_RIGHT_SWITCH;
            }
        } else {
            if (switchSide == Side.LEFT) {
                pathToSwitch = AutonomousPaths.RIGHT_START_TO_LEFT_SWITCH;
            } else {
                pathToSwitch = AutonomousPaths.RIGHT_START_TO_RIGHT_SWITCH;
            }
        }

        addSequential(new FollowPathCommand(robot.getDrivetrain(), pathToSwitch));
        addSequential(new SetDrivetrainAngleCommand(robot.getDrivetrain(), switchAngle));
        addSequential(new FollowPathCommand(robot.getDrivetrain(), pathToSwitchSide));
    }
}
