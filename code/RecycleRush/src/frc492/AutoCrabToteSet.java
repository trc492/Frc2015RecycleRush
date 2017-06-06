package frc492;

import frclibj.TrcDashboard;
import frclibj.TrcEvent;
import frclibj.TrcStateMachine;
import frclibj.TrcRobot.AutoStrategy;

public class AutoCrabToteSet implements AutoStrategy
{
    private static final String moduleName = "AutoCrabToteSet";
    private Robot robot;
    private double distance;
    private TrcEvent driveEvent;
    private TrcEvent elevatorEvent;
    private TrcStateMachine sm;

    private final int STATE_PICKUP_TOTE = TrcStateMachine.STATE_STARTED;
    private final int STATE_MOVE_AUTOZONE = TrcStateMachine.STATE_STARTED + 1;
    private final int STATE_DONE = TrcStateMachine.STATE_STARTED + 2;

    public AutoCrabToteSet (Robot robot, double distance)
    {
        this.robot = robot;
        this.distance = distance;
        driveEvent = new TrcEvent(moduleName + ".driveEvent");
        elevatorEvent = new TrcEvent(moduleName + ".elevatorEvent");
        sm = new TrcStateMachine(moduleName + ".sm");
        sm.start();
    }   //AutoCrabToteSet

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
            case STATE_PICKUP_TOTE:
                if (robot.lowerGrabber != null)
                {
                    robot.lowerGrabber.extend();
                }
                robot.elevator.setHeight(
                        RobotInfo.ELEVATOR_LIFT_TOTE,
                        elevatorEvent, 1.0);
                sm.addEvent(elevatorEvent);
                sm.waitForEvents(STATE_MOVE_AUTOZONE);
                break;

            case STATE_MOVE_AUTOZONE:
                //
                // move forward into the auto zone
                //
                robot.elevator.setHeight(RobotInfo.ELEVATOR_LIFT_TOTE);
                robot.pidDrive.setTarget(
                        distance, 0.0, 0.0, false, driveEvent, 10.0);
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
}
