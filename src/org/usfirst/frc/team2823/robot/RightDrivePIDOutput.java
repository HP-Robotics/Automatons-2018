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
		
		r.motor0.set(ControlMode.Velocity, -output);
		r.motor1.set(ControlMode.Velocity, -output);
		
	}

}
