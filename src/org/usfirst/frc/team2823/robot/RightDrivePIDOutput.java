package org.usfirst.frc.team2823.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.PIDOutput;

public class RightDrivePIDOutput implements PIDOutput {
	
	Robot r;
	
	public RightDrivePIDOutput (Robot robot){
		r = robot;
	}
	
	@Override
	public void pidWrite(double output) {
		
		r.rightMotor3.set(ControlMode.PercentOutput, output);
		r.rightMotor4.set(ControlMode.PercentOutput, output);
		
	}

}
