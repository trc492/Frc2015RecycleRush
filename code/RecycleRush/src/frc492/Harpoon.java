package frc492;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Servo;

public class Harpoon
{
//    private static final String moduleName = "Harpoon";
    private Servo leftServo;
    private Servo rightServo;
    private CANTalon winch;

    public Harpoon()
    {
        leftServo = new Servo(RobotInfo.PWM_LEFT_HARPOON_SERVO);
        rightServo = new Servo(RobotInfo.PWM_RIGHT_HARPOON_SERVO);
        winch = new CANTalon(RobotInfo.CANID_HARPOON_WINCH);
        leftServo.set(RobotInfo.LEFTSERVO_LATCHED);
        rightServo.set(RobotInfo.RIGHTSERVO_LATCHED);
    }

    public void fire()
    {
        leftServo.set(RobotInfo.LEFTSERVO_UNLATCHED);
        rightServo.set(RobotInfo.RIGHTSERVO_UNLATCHED);
    }

    public void winchBack()
    {
        leftServo.set(RobotInfo.LEFTSERVO_LATCHED);
        rightServo.set(RobotInfo.RIGHTSERVO_LATCHED);
        winch.set(1.0);
    }

    public void winchStop()
    {
        winch.set(0.0);
    }

}   //class Harpoon
