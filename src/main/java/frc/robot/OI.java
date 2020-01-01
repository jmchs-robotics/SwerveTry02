package frc.robot;

import frc.robot.commands.SendVisionCommand;
import frc.robot.commands.SetFieldOrientedCommand;
import frc.robot.commands.SetMotorBrakeCommand;
import frc.robot.commands.ZeroDrivetrainGyroCommand;
import frc.robot.input.IGamepad;
import frc.robot.input.XboxGamepad;
import frc.robot.input.DPadButton.Direction;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

    private IGamepad primaryController = new XboxGamepad(0);
    private IGamepad secondaryController = new XboxGamepad(1);

    private Robot mRobot;

    public OI(Robot robot) {
        mRobot = robot;
    }

    public void registerControls() {
        primaryController.getLeftBumperButton().whenPressed(new SetFieldOrientedCommand(mRobot.getDrivetrain(), false));
        primaryController.getLeftBumperButton().whenReleased(new SetFieldOrientedCommand(mRobot.getDrivetrain(), true));
        primaryController.getStartButton().whenPressed(new ZeroDrivetrainGyroCommand(mRobot.getDrivetrain()));
        //If the swerve is in Brake or Coast Mode... Braking when pressed
        primaryController.getRightBumperButton().whenPressed(new SetMotorBrakeCommand(mRobot,true));
        primaryController.getRightBumperButton().whenReleased(new SetMotorBrakeCommand(mRobot,false));

        primaryController.getAButton().whenPressed(new SendVisionCommand(mRobot.sender_, "G"));
        primaryController.getAButton().whenReleased(new SendVisionCommand(mRobot.sender_, "R"));
        // Example of using DPad to run commands:
        // primaryController.getDPadButton(Direction.CENTER).whenActive(new SendVisionCommand(mRobot.sender_, "B"));;
}

    public IGamepad getPrimaryController() {
        return primaryController;
    }

    public IGamepad getSecondaryController() {
        return secondaryController;
    }
}
