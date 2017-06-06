package frc492;

import frclibj.TrcDashboard;
import frclibj.TrcEvent;
import frclibj.TrcStateMachine;
import frclibj.TrcRobot.AutoStrategy;
import frclibj.TrcTimer;

public class AutoStackBin implements AutoStrategy
{
    private static final String moduleName = "AutoStackBin";
    private Robot robot;
    private double distance;
    private TrcTimer timer;
    private TrcEvent timerEvent;
    private TrcEvent driveEvent;
    private TrcEvent elevatorEvent;
    private TrcStateMachine sm;

    private final int STATE_PICKUP_BIN = TrcStateMachine.STATE_STARTED ;
    private final int STATE_MOVE_FORWARD = TrcStateMachine.STATE_STARTED + 1;
    private final int STATE_DROP_DOWN = TrcStateMachine.STATE_STARTED + 2;
    private final int STATE_RELEASE_BIN = TrcStateMachine.STATE_STARTED + 3;
    private final int STATE_BOTTOM_OUT = TrcStateMachine.STATE_STARTED + 4;
    private final int STATE_GRAB_STACK = TrcStateMachine.STATE_STARTED + 5;
    private final int STATE_PICKUP_STACK = TrcStateMachine.STATE_STARTED + 6;
    private final int STATE_TURN_LEFT = TrcStateMachine.STATE_STARTED + 7;
    private final int STATE_MOVE_AUTOZONE = TrcStateMachine.STATE_STARTED + 8;
    private final int STATE_TURN_AROUND = TrcStateMachine.STATE_STARTED + 9;
    private final int STATE_DONE = TrcStateMachine.STATE_STARTED + 10;

    public AutoStackBin (Robot robot, double distance)
    {
        this.robot = robot;
        this.distance = distance;
        timer = new TrcTimer(moduleName);
        timerEvent = new TrcEvent(moduleName + ".timerEvent");
        driveEvent = new TrcEvent(moduleName + ".driveEvent");
        elevatorEvent = new TrcEvent(moduleName + ".elevatorEvent");
        sm = new TrcStateMachine(moduleName + ".sm");
        sm.start();
    }   //AutoPickupBin

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
                if (robot.lowerGrabber != null)
                {
                    robot.lowerGrabber.extend();
                }
                robot.elevator.setHeight(
                        RobotInfo.ELEVATOR_STACK_BIN,
                        elevatorEvent, 1.0);
                sm.addEvent(elevatorEvent);
                sm.waitForEvents(STATE_MOVE_FORWARD);
                break;

            case STATE_MOVE_FORWARD:
                robot.elevator.setHeight(RobotInfo.ELEVATOR_STACK_BIN);
                robot.pidDrive.setTarget(
                        0.0, 32.0, 0.0, false, driveEvent, 3.0);
                sm.addEvent(driveEvent);
                sm.waitForEvents(STATE_DROP_DOWN);
                break;

            case STATE_DROP_DOWN:
                robot.elevator.setHeight(
                        RobotInfo.ELEVATOR_STACK_TOTE, elevatorEvent, 1.0);
                sm.addEvent(elevatorEvent);
                sm.waitForEvents(STATE_RELEASE_BIN);
                break;
                
            case STATE_RELEASE_BIN:
                if (robot.lowerGrabber != null)
                {
                    robot.lowerGrabber.retract();
                }
                robot.elevator.setHeight(
                        RobotInfo.ELEVATOR_GROUND,
                        elevatorEvent, 1.0);
                sm.addEvent(elevatorEvent);
                sm.waitForEvents(STATE_BOTTOM_OUT);
                break;

            case STATE_BOTTOM_OUT:
                robot.elevator.zeroCalibrate(RobotInfo.ELEVATOR_CAL_POWER);
                timer.set(1.0, timerEvent);
                sm.addEvent(timerEvent);
                sm.waitForEvents(STATE_GRAB_STACK);
                break;

            case STATE_GRAB_STACK:
                if (robot.lowerGrabber != null)
                {
                    robot.lowerGrabber.extend();
                }
                timer.set(0.5, timerEvent);
                sm.addEvent(timerEvent);
                sm.waitForEvents(STATE_PICKUP_STACK);
                break;

            case STATE_PICKUP_STACK:
                robot.elevator.setHeight(
                        RobotInfo.ELEVATOR_LIFT_TOTE,
                        elevatorEvent, 1.0);
                sm.addEvent(elevatorEvent);
                sm.waitForEvents(STATE_TURN_LEFT);
                break;

            case STATE_TURN_LEFT:
                robot.elevator.setHeight(RobotInfo.ELEVATOR_LIFT_TOTE);
                robot.pidDrive.setTarget(
                        0.0, 0.0, -90.0, false, driveEvent, 2.5);
                sm.addEvent(driveEvent);
                sm.waitForEvents(STATE_MOVE_AUTOZONE);
                break;

            case STATE_MOVE_AUTOZONE:
                robot.driveBase.setBrakeModeEnabled(false);
                robot.yPidCtrl.setOutputRange(-0.3, 0.3);
                robot.pidDrive.setTarget(
                        0.0, distance - 24.0, 0.0, false, driveEvent, 10.0);
                sm.addEvent(driveEvent);
                sm.waitForEvents(STATE_TURN_AROUND);
                break;

            case STATE_TURN_AROUND:
                robot.pidDrive.setTarget(
                        0.0, 0.0, -135.0, false, driveEvent, 5.0);
                sm.addEvent(driveEvent);
                sm.waitForEvents(STATE_MOVE_AUTOZONE);
                break;

            case STATE_DONE:
            default:
                sm.stop();
            }
        }
    }   //autoPeriodic
}
