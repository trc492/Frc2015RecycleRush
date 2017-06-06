package frc492;

import frclibj.TrcDashboard;
import frclibj.TrcEvent;
import frclibj.TrcRobot;
import frclibj.TrcStateMachine;

public class AutoRobotSet implements TrcRobot.AutoStrategy
{
    private static final String moduleName = "AutoRobotSet";
    private Robot robot;
    private double distance;
    private TrcEvent driveEvent;
    private TrcStateMachine sm;

    private final int STATE_MOVE_AUTOZONE = TrcStateMachine.STATE_STARTED;
    private final int STATE_DONE = TrcStateMachine.STATE_STARTED + 1;

    public AutoRobotSet(Robot robot, double distance)
    {
        this.robot = robot;
        this.distance = distance;
        driveEvent = new TrcEvent(moduleName + ".driveEvent");
        sm = new TrcStateMachine(moduleName + ".sm");
        sm.start();
    }   //AutoRobotSet

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
            case STATE_MOVE_AUTOZONE:
                //
                // Move forward the given distance.
                //
                robot.pidDrive.setTarget(
                        0.0, distance, 0.0,
                        false,
                        driveEvent,
                        15.0);
                sm.addEvent(driveEvent);
                sm.waitForEvents(STATE_DONE);
                break;

            case STATE_DONE:
            default:
                //
                // We are done.
                //
                sm.stop();
            }
        }
    }   //autoPeriodic

}   //class AutoRobotSet
