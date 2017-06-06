package frclibj;

import edu.wpi.first.wpilibj.AnalogInput;

public class TrcAnalogInput extends AnalogInput implements TrcTaskMgr.Task
{
    private static final String moduleName = "TrcAnalogInput";
    private static final boolean debugEnabled = false;
    private TrcDbgTrace dbgTrace = null;

    public enum Zone
    {
        ANALOGINPUT_UNKNOWN_ZONE,
        ANALOGINPUT_LOW_ZONE,
        ANALOGINPUT_MID_ZONE,
        ANALOGINPUT_HIGH_ZONE
    }   //enum Zone

    public interface AnalogEventHandler
    {
        public void AnalogEvent(
                TrcAnalogInput analogInput,
                Zone zone,
                double value);
    }   //interface AnalogEventHandler

    public static final int ANALOGINPUTO_FILTER_ENABLED = (1 << 0);
    public static final int ANALOGINPUTO_INVERTED       = (1 << 1);

    private String instanceName;
    private double unitScale;
    private double lowThreshold;
    private double highThreshold;
    private int options;
    private AnalogEventHandler eventHandler;
    private TrcKalmanFilter kalman;
    private Zone prevZone;

    public TrcAnalogInput(
            final String instanceName,
            final int channel,
            double unitScale,
            double lowThreshold,
            double highThreshold,
            int options,
            AnalogEventHandler eventHandler)
    {
        super(channel);
        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(
                    moduleName + "." + instanceName,
                    false,
                    TrcDbgTrace.TraceLevel.API,
                    TrcDbgTrace.MsgLevel.INFO);
        }

        this.instanceName = instanceName;
        this.unitScale = unitScale;
        this.lowThreshold = lowThreshold;
        this.highThreshold = highThreshold;
        this.options = options;
        this.eventHandler = eventHandler;
        if ((options & ANALOGINPUTO_FILTER_ENABLED) != 0)
        {
            kalman = new TrcKalmanFilter();
        }
        else
        {
            kalman = null;
        }
        prevZone = Zone.ANALOGINPUT_UNKNOWN_ZONE;
    }   //TrcAnalogInput

    public void setThresholds(double lowThreshold, double highThreshold)
    {
        final String funcName = "setThreshold";
        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API,
                    "lowThreshold=%f,highThreshold=%f",
                    lowThreshold, highThreshold);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        this.lowThreshold = lowThreshold;
        this.highThreshold = highThreshold;
    }   //setThresholds

    public double getData()
    {
        final String funcName = "getData";
        double data = getVoltage()*unitScale;
        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.FUNC);
            dbgTrace.traceExit(
                    funcName, TrcDbgTrace.TraceLevel.FUNC, "=%f", data);
        }
        return data;
    }   //getValue

    public void setEnabled(boolean enabled)
    {
        final String funcName = "setEnabled";
        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.FUNC,
                    "enabled=%s", Boolean.toString(enabled));
        }

        //throws exception if no event handler.
        if (enabled)
        {
            TrcTaskMgr.registerTask(
                    instanceName,
                    this,
                    TrcTaskMgr.TaskType.PREPERIODIC_TASK);
        }
        else
        {
            TrcTaskMgr.unregisterTask(
                    this,
                    TrcTaskMgr.TaskType.PREPERIODIC_TASK);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC);
        }
    }   //setEnabled

    //
    // Implements TrcTaskMgr.Task
    //
    public void startTask(TrcRobot.RunMode runMode)
    {
    }   //startTask

    public void stopTask(TrcRobot.RunMode runMode)
    {
    }   //stopTask

    public void prePeriodicTask(TrcRobot.RunMode runMode)
    {
        final String funcName = "prePeriodic";
        double data = getVoltage()*unitScale;

        if (kalman != null)
        {
            data = kalman.filter(data);
        }

        Zone zone;
        if (data <= lowThreshold)
        {
            zone = (options & ANALOGINPUTO_INVERTED) != 0?
                    Zone.ANALOGINPUT_HIGH_ZONE: Zone.ANALOGINPUT_LOW_ZONE;
        }
        else if (data <= highThreshold)
        {
            zone = Zone.ANALOGINPUT_MID_ZONE;
        }
        else
        {
            zone = (options & ANALOGINPUTO_INVERTED) != 0?
                   Zone.ANALOGINPUT_LOW_ZONE: Zone.ANALOGINPUT_HIGH_ZONE;
        }

        if (zone != prevZone)
        {
            //
            // We have crossed to another zone, let's notify somebody.
            //
            prevZone = zone;
            if (eventHandler != null)
            {
                eventHandler.AnalogEvent(this, zone, data);
            }
            if (debugEnabled)
            {
                dbgTrace.traceInfo(
                        funcName, "%s entering %s (data=%f)",
                        instanceName, zone.toString(), data);
            }
        }
    }   //prePeriodicTask

    public void postPeriodicTask(TrcRobot.RunMode runMode)
    {
    }   //postPeriodicTask

    public void preContinuousTask(TrcRobot.RunMode runMode)
    {
    }   //preContinuousTask

    public void postContinuousTask(TrcRobot.RunMode runMode)
    {
    }   //postContinuousTask

}   //class TrcAnalogInput
