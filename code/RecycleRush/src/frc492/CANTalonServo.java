package frc492;

import edu.wpi.first.wpilibj.CANTalon;

public class CANTalonServo extends CANTalon
{
	private double m_kEMF;		// Velocity to voltage
	public void kEMF(double val)	{ m_kEMF = val; }
	public double kEMF()			{return m_kEMF; }
	
	private double m_Drive;	// Requested torque output
	public void Drive(double val)	{ m_Drive = val; }
	public double Drive() 			{return m_Drive; }
	
	private boolean m_Saturated;
	public boolean isSaturated() { return m_Saturated; }

	public CANTalonServo(int deviceNumber) {
		super(deviceNumber);
	}

	// Torque, like drive, is -1.0 to 1.0.  No compensation for voltage 
	// variation (could do this with more math and sampling bus voltage)
	// Assumes incremental encoder attached to Talon.  NB the velocity values
	// returned are counts/.1 sec.  Could compute velocity in terms of the 
	// call rate, I suppose.
	//
	// Math: Apply full power (1.0), measure velocity, then kEMF becomes 1.0/vel 
	// Probably should have separate values for reverse as sometimes the PMDC
	// motors have their coils shifted and thus differing kEMF.
	//
	// To handle varying bus voltage, the math would be (untested)
	// kEMF = 1.0/vBus/Velocity.  Then the required additional drive would be
	// Velocity * kEMF/vBus - thus boosted as vBus sags.
	//
	public void setTorque(double torque)
	{
		double output = m_Drive + (m_kEMF * getEncVelocity());

		m_Saturated = true;
		if (output > 1.0)
			this.set(1.0);
		else if (output < -1.0)
			this.set(-1.0);
		else {
			this.set(output);
			m_Saturated = false;
		}
	}
}
