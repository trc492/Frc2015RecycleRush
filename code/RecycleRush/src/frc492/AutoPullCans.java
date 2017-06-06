package frc492;

import frclibj.TrcDashboard;
import frclibj.TrcEvent;
import frclibj.TrcStateMachine;
import frclibj.TrcRobot.AutoStrategy;
import frclibj.TrcTimer;

public class AutoPullCans implements AutoStrategy
{
    private static final String moduleName = "AutoPullCans";
    private Robot robot;
    private double distance;
    private double delay;
    private double backDistance;
    private TrcTimer timer;
    private TrcEvent timerEvent;
    private TrcEvent driveEvent;
    private TrcStateMachine sm;

    private final int STATE_GO_BACK = TrcStateMachine.STATE_STARTED;
    private final int STATE_HOOK_CANS = TrcStateMachine.STATE_STARTED + 1;
    private final int STATE_PULL_CANS = TrcStateMachine.STATE_STARTED + 2;
    private final int STATE_DONE = TrcStateMachine.STATE_STARTED + 3;

    public AutoPullCans(
            Robot robot,
            double drivePower,
            double backDistance,
            double distance,
            double delay)
    {
        this.robot = robot;
        this.distance = distance;
        this.delay = delay;
        this.backDistance = backDistance;
        timer = new TrcTimer(moduleName);
        timerEvent = new TrcEvent(moduleName + ".timerEvent");
        driveEvent = new TrcEvent(moduleName + ".driveEvent");
        sm = new TrcStateMachine(moduleName + ".sm");
        robot.leftHook.retract();
        robot.rightHook.retract();
        robot.yPidCtrl.setOutputRange(
                -drivePower, drivePower);
        sm.start();
    }   //AutoPullCans

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
        robot.yPidCtrl.displayPidInfo(2);

        if (ready)
        {
            int state = sm.getState();
            switch (state)
            {
            case STATE_GO_BACK:
                //
                // move forward into the auto zone
                //
                robot.pidDrive.setTarget(
                        0.0, backDistance, 0.0, false, driveEvent, 5.0);
                sm.addEvent(driveEvent);
                sm.waitForEvents(STATE_HOOK_CANS);
                break;

            case STATE_HOOK_CANS:
                //
                // lift elevator to grabbing height
                //
                robot.leftHook.extend();
                robot.rightHook.extend();
                timer.set(delay, timerEvent);
                sm.addEvent(timerEvent);
                sm.waitForEvents(STATE_PULL_CANS);
                break;

            case STATE_PULL_CANS:
                //
                // move forward into the auto zone
                //
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

}   //class AutoPullCans
