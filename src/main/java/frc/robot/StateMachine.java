package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.YAMSIntakePivot;

public class StateMachine {
    // TODO: add logging/simulation for states


    public final YAMSIntakePivot yIntakePivot = new YAMSIntakePivot();

    public enum RobotState {
        // Todo: add all states as in button mapping doc
        HANDOFF


    }

    private RobotState currentState = RobotState.HANDOFF;

    public Command setState(RobotState newState) {
        return new InstantCommand(() -> currentState = newState);
    }

    // Functions below:
    // Todo: add command that combines intakeCoral and stowCoral, update states


    public Command stowCoral() {
        return Commands.sequence(setState(RobotState.HANDOFF),
                yIntakePivot.setAngle(YAMSIntakePivot.HANDOFF_ANGLE));
    }







}
