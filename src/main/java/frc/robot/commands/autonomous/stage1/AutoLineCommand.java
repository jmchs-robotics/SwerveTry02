package frc.robot.commands.autonomous.stage1;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.commands.SetFieldOrientedAngleCommand;
import frc.robot.commands.autonomous.DriveForDistanceCommand;

import static frc.robot.commands.autonomous.AutonomousConstants.WALL_TO_SWITCH;

public class AutoLineCommand extends CommandGroup {

    public AutoLineCommand(Robot robot, StartingPosition startPos) {
        if (startPos == StartingPosition.LEFT)
            addSequential(new SetFieldOrientedAngleCommand(robot.getDrivetrain(),
                    robot.getDrivetrain().getRawGyroAngle() - 90));
        else if (startPos == StartingPosition.RIGHT)
            addSequential(new SetFieldOrientedAngleCommand(robot.getDrivetrain(),
                    robot.getDrivetrain().getRawGyroAngle() + 90));
        else if (startPos == StartingPosition.CENTER)
            addSequential(new SetFieldOrientedAngleCommand(robot.getDrivetrain(),
                    robot.getDrivetrain().getRawGyroAngle()));

        double driveDistance;
        if (startPos == StartingPosition.CENTER)
            driveDistance = WALL_TO_SWITCH - robot.getDrivetrain().getLength();
        else
            driveDistance = WALL_TO_SWITCH - robot.getDrivetrain().getWidth();

        addSequential(new DriveForDistanceCommand(robot.getDrivetrain(),
                0,
                driveDistance));
    }
}
