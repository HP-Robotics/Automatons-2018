package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.PIDOutput;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class LeftDrivePIDOutput implements PIDOutput {
	
	Robot r;
	
	public LeftDrivePIDOutput (Robot robot){
		r = robot;
	}
	
	@Override
	public void pidWrite(double output) {
		// TODO Auto-generated method stub
		r.rightMotor3.set(ControlMode.Velocity, output);
		r.rightMotor4.set(ControlMode.Velocity, output);
		
	}

}
