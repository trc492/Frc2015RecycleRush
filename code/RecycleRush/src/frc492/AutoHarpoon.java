package frc492;

import frclibj.TrcDashboard;
import frclibj.TrcEvent;
import frclibj.TrcStateMachine;
import frclibj.TrcRobot.AutoStrategy;
import frclibj.TrcTimer;

public class AutoHarpoon implements AutoStrategy
{
    private static final String moduleName = "AutoHarpoon";
    private Robot robot;
    private double backDistance;
    private TrcTimer timer;
    private TrcEvent timerEvent;
    private TrcEvent driveEvent;
    private TrcStateMachine sm;

    private final int STATE_FIRE_HARPOON = TrcStateMachine.STATE_STARTED;
    private final int STATE_PULL_CANS = TrcStateMachine.STATE_STARTED + 1;
    private final int STATE_DONE = TrcStateMachine.STATE_STARTED + 2;

    public AutoHarpoon(
            Robot robot,
            double drivePower,
            double backDistance)
    {
        this.robot = robot;
        this.backDistance = backDistance;
        timer = new TrcTimer(moduleName);
        timerEvent = new TrcEvent(moduleName + ".timerEvent");
        driveEvent = new TrcEvent(moduleName + ".driveEvent");
        sm = new TrcStateMachine(moduleName + ".sm");
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
            case STATE_FIRE_HARPOON:
                robot.harpoon.fire();
                timer.set(0.25, timerEvent);
                sm.addEvent(timerEvent);
                sm.waitForEvents(STATE_PULL_CANS);
                break;

            case STATE_PULL_CANS:
                //
                // move forward into the auto zone
                //
                robot.harpoon.winchBack();
                timer.set(1.75, timerEvent);
                robot.pidDrive.setTarget(
                        0.0, backDistance, 0.0, false, driveEvent, 2.0);
                sm.addEvent(timerEvent);
                sm.waitForEvents(STATE_DONE);
                break;

            case STATE_DONE:
            default:
                //
                // stop (in the name of love)
                //
                robot.harpoon.winchStop();
                sm.stop();
            }
        }
    }   //autoPeriodic

}   //class AutoHarpoon
