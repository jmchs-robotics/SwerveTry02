package frc.robot.commands.autonomous.stage1;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.commands.SetFieldOrientedAngleCommand;
import frc.robot.commands.autonomous.DriveForDistanceCommand;
import frc.robot.util.Side;

import static frc.robot.commands.autonomous.AutonomousConstants.WALL_TO_SWITCH;

public class Stage1CenterSwitchCommand extends CommandGroup {
    public Stage1CenterSwitchCommand(Robot robot, Side switchSide) {
        System.out.println("CENTER SWITCH");
        addSequential(new SetFieldOrientedAngleCommand(robot.getDrivetrain(), robot.getDrivetrain().getRawGyroAngle()));

        addSequential(new DriveForDistanceCommand(robot.getDrivetrain(),
                (switchSide == Side.LEFT ? 1.2 : -1) * 50,
                WALL_TO_SWITCH - robot.getDrivetrain().getWidth() + 3));
    }
}
