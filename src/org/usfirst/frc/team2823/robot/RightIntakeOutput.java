package org.usfirst.frc.team2823.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.PIDOutput;

public class RightIntakeOutput implements PIDOutput {
	
	Robot r;
	public RightIntakeOutput(Robot robot) {
		r = robot;
	}
	
	@Override
	public void pidWrite (double output) {
		r.rightElbow.set(ControlMode.PercentOutput, output);
	}

}
