package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.commands.AdjustFieldOrientedAngleCommand;
import frc.robot.commands.SetFieldOrientedAngleCommand;
import frc.robot.commands.autonomous.stage1.*;
import frc.robot.util.Side;

import java.util.ArrayList;
import java.util.List;

public class AutonomousChooser {
    public static final int CHOICE_COUNT = 1;
    private final SendableChooser<StartingPosition> startPosChooser = new SendableChooser<>();
    private List<SendableChooser<AutonomousStageChoice>> priorityChoices = new ArrayList<>();

    private final String OPPOSITE_SPECIAL_WAIT_TIME_NAME = "Opposite Special Wait Time";

    public AutonomousChooser() {
        startPosChooser.addOption("Left", StartingPosition.LEFT);
        startPosChooser.setDefaultOption("Center", StartingPosition.CENTER);
        startPosChooser.addOption("Right", StartingPosition.RIGHT);

        SmartDashboard.putData("Start Position", startPosChooser);

        for (int i = 0; i < CHOICE_COUNT; i++) {
            SendableChooser<AutonomousStageChoice> chooser = new SendableChooser<>();

            chooser.addDefault(String.format("None (%d)", i), AutonomousStageChoice.NONE);
            chooser.addObject(String.format("Auto line (%d)", i), AutonomousStageChoice.AUTOLINE);
            chooser.addObject(String.format("Same Side Scale (%d)", i), AutonomousStageChoice.SAME_SIDE_SCALE);
            chooser.addObject(String.format("Same Side Scale Special (%d)", i), AutonomousStageChoice.SAME_SIDE_SCALE_SPECIAL);
            chooser.addObject(String.format("Same Side Switch (%d)", i), AutonomousStageChoice.SAME_SIDE_SWITCH);
            chooser.addObject(String.format("Opposite Side Scale (%d)", i), AutonomousStageChoice.OPPOSITE_SIDE_SCALE);
            chooser.addObject(String.format("Opposite Side Scale Special (%d)", i), AutonomousStageChoice.OPPOSITE_SIDE_SCALE_SPECIAL);
            chooser.addObject(String.format("Opposite Side Switch (%d)", i), AutonomousStageChoice.OPPOSITE_SIDE_SWITCH);

            priorityChoices.add(chooser);

            SmartDashboard.putData("Auto Preference " + i, chooser);

        }

        SmartDashboard.putNumber(OPPOSITE_SPECIAL_WAIT_TIME_NAME, 0);
    }

    private boolean isChoiceGood(AutonomousStageChoice choice, String fieldConf, StartingPosition startPos) {
        Side switchSide = Side.fromChar(fieldConf.charAt(0)),
                scaleSide = Side.fromChar(fieldConf.charAt(1));
        if (startPos == StartingPosition.CENTER) return true;

        switch (choice) {
            case NONE:
                return true;
            case AUTOLINE:
                return true;
            case SAME_SIDE_SCALE:
                return (startPos == StartingPosition.LEFT && scaleSide == Side.LEFT) ||
                        (startPos == StartingPosition.RIGHT && scaleSide == Side.RIGHT);
            case SAME_SIDE_SWITCH:
                return (startPos == StartingPosition.LEFT && switchSide == Side.LEFT) ||
                        (startPos == StartingPosition.RIGHT && switchSide == Side.RIGHT);
            case OPPOSITE_SIDE_SCALE:
                return (startPos == StartingPosition.RIGHT && scaleSide == Side.LEFT) ||
                        (startPos == StartingPosition.LEFT && scaleSide == Side.RIGHT);
            case OPPOSITE_SIDE_SWITCH:
                return (startPos == StartingPosition.RIGHT && switchSide == Side.LEFT) ||
                        (startPos == StartingPosition.LEFT && switchSide == Side.RIGHT);
            default:
                return false;
        }
    }

    private boolean isChoiceGood(AutonomousStageChoice choice, StartingPosition startPos, Side switchSide, Side scaleSide) {
        if (startPos == StartingPosition.CENTER) return true;

        switch (choice) {
            case NONE:
                return true;
            case AUTOLINE:
                return true;
            case SAME_SIDE_SCALE:
            case SAME_SIDE_SCALE_SPECIAL:
                return (startPos == StartingPosition.LEFT && scaleSide == Side.LEFT) ||
                        (startPos == StartingPosition.RIGHT && scaleSide == Side.RIGHT);
            case SAME_SIDE_SWITCH:
                return (startPos == StartingPosition.LEFT && switchSide == Side.LEFT) ||
                        (startPos == StartingPosition.RIGHT && switchSide == Side.RIGHT);
            case OPPOSITE_SIDE_SCALE:
            case OPPOSITE_SIDE_SCALE_SPECIAL:
                return (startPos == StartingPosition.RIGHT && scaleSide == Side.LEFT) ||
                        (startPos == StartingPosition.LEFT && scaleSide == Side.RIGHT);
            case OPPOSITE_SIDE_SWITCH:
                return (startPos == StartingPosition.RIGHT && switchSide == Side.LEFT) ||
                        (startPos == StartingPosition.LEFT && switchSide == Side.RIGHT);
            default:
                return false;
        }
    }

    public Command getCommand(Robot robot) {
       // GrabCubeFromPlatformZoneCommand.resetAvailableCubes();

        String fieldConfiguration = DriverStation.getInstance().getGameSpecificMessage();
        StartingPosition startPos = startPosChooser.getSelected();

        CommandGroup autoGroup = new CommandGroup();
        // simplified 12/1/19 as first try
        autoGroup.addSequential(new AutoLineCommand(robot, startPos));
        return autoGroup;
    }

    public Command getCommand(Robot robot, Side switchSide, Side scaleSide) {
        StartingPosition startPos = startPosChooser.getSelected();


        CommandGroup autoGroup = new CommandGroup();
        autoGroup.addSequential(new SetFieldOrientedAngleCommand(robot.getDrivetrain(), robot.getDrivetrain().getRawGyroAngle()));


        System.out.printf("[INFO] Starting in position: %s%n", startPos);
        // Print out the selected choices because logging
        for (int i = 0; i < CHOICE_COUNT; i++)
            System.out.printf("[INFO]: Selected choice %d: %s%n", i, priorityChoices.get(i).getSelected());

        boolean atScale = false;
        int stageNumber = 1;
        for (int i = 0; i < priorityChoices.size(); i++) {
            AutonomousStageChoice choice = priorityChoices.get(i).getSelected();
            if (!isChoiceGood(choice, startPos, switchSide, scaleSide))
                continue;

            System.out.printf("[INFO]: Stage %d action: %s%n", stageNumber, choice);

            if (stageNumber == 1) {
                Side startSide = null;
                if (startPos == StartingPosition.LEFT)
                    startSide = Side.LEFT;
                else if (startPos == StartingPosition.RIGHT)
                    startSide = Side.RIGHT;

                switch (choice) {
                    case NONE:
                        return autoGroup;
                    case AUTOLINE:
                        autoGroup.addSequential(new DriveForDistanceCommand(robot.getDrivetrain(), 0, 120));
                        return autoGroup;
                    case SAME_SIDE_SCALE:
                    case OPPOSITE_SIDE_SCALE:
                        autoGroup.addSequential(new ScoreScaleFrontFromStartForward(robot, startSide, scaleSide));
                        atScale = true;
                        break;
                    case SAME_SIDE_SWITCH:
                    case OPPOSITE_SIDE_SWITCH:
                        if (startPos == StartingPosition.CENTER)
                            autoGroup.addSequential(new ScoreSwitchFrontFromStartCenter(robot, switchSide));
                        else
                            autoGroup.addSequential(new ScoreSwitchSideFromStartForward(robot, startSide, switchSide));
                        atScale = false;
                        break;
                    case SAME_SIDE_SCALE_SPECIAL:
                    case OPPOSITE_SIDE_SCALE_SPECIAL:
                        double waitTime = SmartDashboard.getNumber(OPPOSITE_SPECIAL_WAIT_TIME_NAME, 0);
                        autoGroup.addSequential(new ScoreScaleSideFromStartSide(robot, startSide, scaleSide, waitTime));
                        return autoGroup;
                }
            } else {
                switch (choice) {
                    case AUTOLINE:
                        // Intentional fallthrough, nothing we can to as an auto already ran.
                    case NONE:
                        // Nothing else to do
                        return autoGroup;
                    case SAME_SIDE_SCALE:
                    case OPPOSITE_SIDE_SCALE:
                        if (atScale) {
                        } else
                            System.err.println("[WARNING]: I don't know how to score in the scale when I'm not at the scale!");
                        atScale = true;
                        break;
                    case SAME_SIDE_SWITCH:
                    case OPPOSITE_SIDE_SWITCH:
                        if (startPos == StartingPosition.CENTER)
                            autoGroup.addSequential(new ScoreSwitchFrontFromSwitchFront(robot, switchSide));
                        else if (atScale)
                            autoGroup.addSequential(new ScoreSwitchBackFromScale(robot, switchSide, scaleSide));
                        else
                            System.err.println("[WARNING]: I don't know how to get to the switch if I'm not at the switch front or the scale!");
                        atScale = false;
                        break;
                }
            }

            stageNumber++;
        }

        return autoGroup;
    }
}
