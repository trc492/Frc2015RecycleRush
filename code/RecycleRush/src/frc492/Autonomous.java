package frc492;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frclibj.TrcDashboard;
import frclibj.TrcRobot;

public class Autonomous implements TrcRobot.RobotMode
{
 	public enum AutoMode
    {
        AUTOMODE_NONE,
        AUTOMODE_ROBOT_SET,     // Move into AutoZone (4 pt)
        AUTOMODE_CRAB_TOTE_SET, // Pick up tote and crab sideways to AutoZone (6 + 4 pt)
        AUTOMODE_BIN_SET,       // Pick up bin and move to AutoZone (8 + 4 pt)
        AUTOMODE_CRAB_BIN_SET,  // Pick up bin and crab sideways to AutoZone (8 + 4 pt)
        AUTOMODE_STACK_BIN,     // Pick up bin, stack on tote and move into AutoZone (8 + 6 + 4 pt)
        AUTOMODE_FEED_NOODLE,
        AUTOMODE_PULL_CANS,
        AUTOMODE_HARPOON,
        AUTOMODE_TUNE_ROBOT
    }

    public enum TuneMode
    {
        TUNEMODE_MOVE_X,
        TUNEMODE_MOVE_Y,
        TUNEMODE_TURN,
        TUNEMODE_SONAR
    }

    private static final String autoZoneDistanceKey = "AutoZone Distance";
    private static final String autoChuteDistanceKey = "Auto Chute Distance";
    private static final String autoBinHeightKey = "Auto Bin Height";
    private static final String autoPullDistanceKey = "Auto Pull Can Distance";
    private static final String autoPullCanDelayKey = "Auto Pull Can Delay";
    private static final String autoBackDistanceKey = "Auto Back Distance";
    private static final String autoDrivePowerKey = "Auto Drive Power";
    private Robot robot;
    private SendableChooser autoChooser;
    private TrcRobot.AutoStrategy autoStrategy;
    private SendableChooser tuneChooser;
    private double autoZoneDistance = 132.0;
    private double autoChuteDistance = 24.0;
    private double autoBinHeight = 100.0;
    private double autoPullDistance = 60.0;
    private double autoPullCanDelay = 1.0;
    private double autoBackDistance = -60.0;
    private double autoDrivePower = 1.0;

    public Autonomous(Robot robot)
    {
        this.robot = robot;
        autoChooser = new SendableChooser();
        autoChooser.addDefault("No autonomous", AutoMode.AUTOMODE_NONE);
        autoChooser.addObject("Robot Set (4)", AutoMode.AUTOMODE_ROBOT_SET);
        autoChooser.addObject("Crab Tote Set", AutoMode.AUTOMODE_CRAB_TOTE_SET);
        autoChooser.addObject("Bin Set (8+4)", AutoMode.AUTOMODE_BIN_SET);
        autoChooser.addObject("Crab Bin Set",  AutoMode.AUTOMODE_CRAB_BIN_SET);
        if (robot.lowerGrabber != null)
        {
            autoChooser.addObject(
                    "Stack Bin (8+6+4)",
                    AutoMode.AUTOMODE_STACK_BIN);
            autoChooser.addObject("Feed Noodle", AutoMode.AUTOMODE_FEED_NOODLE);
        }
        if (robot.leftHook != null && robot.rightHook != null)
        {
            autoChooser.addObject("Pull Cans", AutoMode.AUTOMODE_PULL_CANS);
        }
        if (robot.harpoon != null)
        {
            autoChooser.addObject("Harpoon Cans", AutoMode.AUTOMODE_HARPOON);
        }
        autoChooser.addObject("Tune robot", AutoMode.AUTOMODE_TUNE_ROBOT);
        TrcDashboard.putData("Autonomous Strategies", autoChooser);

        tuneChooser = new SendableChooser();
        tuneChooser.addDefault("Move X 20 ft", TuneMode.TUNEMODE_MOVE_X);
        tuneChooser.addObject("Move Y 20 ft", TuneMode.TUNEMODE_MOVE_Y);
        tuneChooser.addObject("Turn 360", TuneMode.TUNEMODE_TURN);
        tuneChooser.addObject("Sonar drive 7 in", TuneMode.TUNEMODE_SONAR);
        TrcDashboard.putData("Robot tune modes", tuneChooser);

        autoZoneDistance =
                TrcDashboard.getNumber(autoZoneDistanceKey, autoZoneDistance);
        autoChuteDistance =
                TrcDashboard.getNumber(autoChuteDistanceKey, autoChuteDistance);
        autoBinHeight =
                TrcDashboard.getNumber(autoBinHeightKey, autoBinHeight);
        autoPullDistance =
                TrcDashboard.getNumber(autoPullDistanceKey, autoPullDistance);
        autoPullCanDelay =
                TrcDashboard.getNumber(autoPullCanDelayKey, autoPullCanDelay);
        autoBackDistance =
                TrcDashboard.getNumber(autoBackDistanceKey, autoBackDistance);
        autoDrivePower =
                TrcDashboard.getNumber(autoDrivePowerKey, autoDrivePower);
     }   //Autonomous

    //
    // Implements TrcRobot.RunMode.
    //
    public void start()
    {
        //
        // Turn on brake mode and limit drive power to make autonomous drive
        // more precise.
        //
        robot.driveBase.setBrakeModeEnabled(true);
        robot.xPidCtrl.setOutputRange(
                -RobotInfo.X_RANGE_LIMIT, RobotInfo.X_RANGE_LIMIT);
        robot.yPidCtrl.setOutputRange(
                -RobotInfo.Y_RANGE_LIMIT, RobotInfo.Y_RANGE_LIMIT);
        robot.turnPidCtrl.setOutputRange(
                -RobotInfo.TURN_RANGE_LIMIT, RobotInfo.TURN_RANGE_LIMIT);
        robot.sonarPidCtrl.setOutputRange(
                -RobotInfo.SONAR_RANGE_LIMIT, RobotInfo.SONAR_RANGE_LIMIT);
        robot.elevator.zeroCalibrate(RobotInfo.ELEVATOR_CAL_POWER);

        AutoMode selectedAutoMode = (AutoMode)(autoChooser.getSelected());
        autoZoneDistance = TrcDashboard.getNumber(autoZoneDistanceKey);
        autoPullDistance = TrcDashboard.getNumber(autoPullDistanceKey);
        autoPullCanDelay = TrcDashboard.getNumber(autoPullCanDelayKey);
        autoBackDistance = TrcDashboard.getNumber(autoBackDistanceKey);
        autoDrivePower = TrcDashboard.getNumber(autoDrivePowerKey);
        switch (selectedAutoMode)
        {
        case AUTOMODE_NONE:
            autoStrategy = null;
            TrcDashboard.textPrintf(1, "NoAuto");
            break;

        case AUTOMODE_ROBOT_SET:
            autoStrategy = new AutoRobotSet(robot, autoZoneDistance);
            break;

        case AUTOMODE_CRAB_TOTE_SET:
            autoStrategy = new AutoCrabToteSet(robot, autoZoneDistance);
            break;

        case AUTOMODE_BIN_SET:
            autoStrategy = new AutoBinSet(robot, autoZoneDistance);
            break;

        case AUTOMODE_CRAB_BIN_SET:
            autoStrategy = new AutoCrabBinSet(robot, autoZoneDistance);
            break;

        case AUTOMODE_STACK_BIN:
            if (robot.lowerGrabber != null)
            {
                autoStrategy = new AutoStackBin(robot, autoZoneDistance);
            }
            break;

        case AUTOMODE_FEED_NOODLE:
            if (robot.lowerGrabber != null)
            {
                autoStrategy = new AutoFeedNoodle(
                        robot, autoChuteDistance, autoBinHeight);
            }
            break;

        case AUTOMODE_PULL_CANS:
            if (robot.leftHook != null && robot.rightHook != null)
            {
                autoStrategy = new AutoPullCans(
                        robot, autoDrivePower, autoBackDistance,
                        autoPullDistance, autoPullCanDelay);
            }
            break;

        case AUTOMODE_HARPOON:
            if (robot.harpoon != null)
            {
                autoStrategy = new AutoHarpoon(
                        robot, autoDrivePower, autoBackDistance);
            }
            break;

        case AUTOMODE_TUNE_ROBOT:
            TuneMode selectedTuneMode = (TuneMode)(tuneChooser.getSelected());
            autoStrategy = new AutoTuneRobot(robot, selectedTuneMode);
            break;
        }
    }   //start

    public void stop()
    {
        robot.driveBase.stop();
    }   //stop

    public void periodic()
    {
        if (autoStrategy != null)
        {
            autoStrategy.autoPeriodic();
        }
        robot.updateDashboard();
    }   //periodic

    public void continuous()
    {
    }   //continuous

}   //class Autonomous
