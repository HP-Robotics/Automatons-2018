package org.usfirst.frc.team2823.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.PIDOutput;

public class LeftIntakeOutput implements PIDOutput {
	
	Robot r;
	public LeftIntakeOutput(Robot robot)  {
		// TODO Auto-generated constructor stub
		r = robot;
	}

	@Override
	public void pidWrite(double output) {
		// TODO Auto-generated method stub
		r.leftElbow.set(ControlMode.PercentOutput, output);
		
	}

}
