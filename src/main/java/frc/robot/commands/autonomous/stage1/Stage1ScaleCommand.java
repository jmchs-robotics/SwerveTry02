package frc.robot.commands.autonomous.stage1;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.commands.SetFieldOrientedAngleCommand;
import frc.robot.commands.autonomous.DriveForDistanceCommand;
import frc.robot.commands.autonomous.SetDrivetrainAngleCommand;

import static frc.robot.commands.autonomous.AutonomousConstants.*;

public class Stage1ScaleCommand extends CommandGroup {

    private final Robot robot;

    public Stage1ScaleCommand(Robot robot, StartingPosition startPos, char scalePos) {
        this.robot = robot;

        if (startPos == StartingPosition.LEFT)
            addSequential(new SetFieldOrientedAngleCommand(robot.getDrivetrain(), robot.getDrivetrain().getRawGyroAngle() - 90));
        else if (startPos == StartingPosition.RIGHT)
            addSequential(new SetFieldOrientedAngleCommand(robot.getDrivetrain(), robot.getDrivetrain().getRawGyroAngle() + 90));

        if (startPos == StartingPosition.CENTER) {
            addSequential(new DriveForDistanceCommand(robot.getDrivetrain(), SWITCH_LENGTH / 2 + SWITCH_SCORE_TO_SWITCH_WALL + robot.getDrivetrain().getLength() / 2., 0));

            startPos = StartingPosition.RIGHT;
        }

        // TODO: Move elevator to scale position

        // Move to switch scoring position
        switch (startPos) {
            case LEFT:
                if (scalePos == 'L') {
                    driveSideToNearScale(startPos);
                } else {
                    driveSideToFarScale(startPos);
                }
                break;
            case RIGHT:
                if (scalePos == 'L') {
                    driveSideToFarScale(startPos);
                } else {
                    driveSideToNearScale(startPos);
                }
                break;
        }
    }

    private void driveSideToFarScale(StartingPosition startPos) {
        addSequential(new DriveForDistanceCommand(robot.getDrivetrain(),
                0,
                WALL_TO_PLATFORM_ZONE - robot.getDrivetrain().getWidth() / 2));
        /* HACK: Manually added 10 to distance when going to other side of field. */
        addSequential(new DriveForDistanceCommand(robot.getDrivetrain(),
                (startPos == StartingPosition.LEFT ? -1 : 1) * (SWITCH_LENGTH + 2 * SCORE_SWITCH + 10 + robot.getDrivetrain().getLength()),
                0));
        addSequential(new SetDrivetrainAngleCommand(robot.getDrivetrain(),
                (startPos == StartingPosition.LEFT ? 90 : 270)));
        addSequential(new DriveForDistanceCommand(robot.getDrivetrain(),
                0,
                WALL_TO_SCALE - WALL_TO_PLATFORM_ZONE));
        addSequential(new DriveForDistanceCommand(robot.getDrivetrain(),
                (startPos == StartingPosition.LEFT ? 1 : -1) * SCORE_SCALE,
                0));

        addSequential(new DriveForDistanceCommand(robot.getDrivetrain(),
                (startPos == StartingPosition.LEFT ? -1 : 1) * SCORE_SCALE,
                WALL_TO_PLATFORM_ZONE - WALL_TO_SCALE));
    }

    private void driveSideToNearScale(StartingPosition startPos) {
        /* HACK: Move less right when on the left side because the energy chain was hitting the scale when on the
        * practice field. */
        addSequential(new DriveForDistanceCommand(robot.getDrivetrain(),
                (startPos == StartingPosition.LEFT ? -1 : 1) * SCORE_SCALE,
                WALL_TO_SCALE - robot.getDrivetrain().getWidth() / 2));
        addSequential(new DriveForDistanceCommand(robot.getDrivetrain(),
                (startPos == StartingPosition.LEFT ? 1 : -1) * SCORE_SCALE,
                WALL_TO_PLATFORM_ZONE - WALL_TO_SCALE));
    }
}
