package frc492;

import frclibj.TrcDashboard;
import frclibj.TrcEvent;
import frclibj.TrcStateMachine;
import frclibj.TrcRobot.AutoStrategy;

public class AutoFeedNoodle implements AutoStrategy
{
    private static final String moduleName = "AutoFeedNoodle";
    private Robot robot;
    private double distance;
    private double height;
    private TrcEvent driveEvent;
    private TrcEvent elevatorEvent;
    private TrcStateMachine sm;

    private final int STATE_PICKUP_BIN = TrcStateMachine.STATE_STARTED;
    private final int STATE_MOVE_CHUTE = TrcStateMachine.STATE_STARTED + 1;
    private final int STATE_DONE = TrcStateMachine.STATE_STARTED + 2;

    public AutoFeedNoodle (Robot robot, double distance, double height)
    {
        this.robot = robot;
        this.distance = distance;
        this.height = height;
        driveEvent = new TrcEvent(moduleName + ".driveEvent");
        elevatorEvent = new TrcEvent(moduleName + ".elevatorEvent");
        sm = new TrcStateMachine(moduleName + ".sm");
        sm.start();
    }   //AutoBinSet

    //
    // Implements TrcRobot.AutoStrategy.
    //
    public void autoPeriodic()
    {
        boolean ready = sm.isReady();
        TrcDashboard.textPrintf(1, "%s[%d] = %s",
                moduleName,
                sm.getState(),
                ready? "Ready": "NotReady");

        if (ready)
        {
            int state = sm.getState();
            switch (state)
            {
            case STATE_PICKUP_BIN:
                //
                // Grab the bin and lift to stack bin height.
                //
                robot.lowerGrabber.extend();
                robot.elevator.setHeight(
                        RobotInfo.ELEVATOR_STACK_BIN,
                        elevatorEvent, 1.0);
                sm.addEvent(elevatorEvent);
                sm.waitForEvents(STATE_MOVE_CHUTE);
                break;

            case STATE_MOVE_CHUTE:
                //
                // lift to final height and move forward toward the chute
                //
                robot.elevator.setHeight(
                        height,
                        elevatorEvent, 10.0);
                robot.pidDrive.setTarget(
                        0.0, distance, 0.0, false, driveEvent, 10.0);
                sm.addEvent(driveEvent);
                sm.waitForEvents(STATE_DONE);
                break;

            case STATE_DONE:
            default:
                //
                // stop (in the name of love)
                //
                sm.stop();
            }
        }
    }   //autoPeriodic

}   //class AutoFeedNoodle
