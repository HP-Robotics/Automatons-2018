package org.usfirst.frc.team2823.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DriverStation;

public class SwitchAuto extends Autonomous {
	
	
	public SwitchAuto(Robot robot) {
		super(robot);
	}
	
	@Override
	public void init() {
		BlueprintStep[] blueprints = new BlueprintStep[] {new BlueprintStep(5.0, this::goStart, this::goPeriodic), 
				new BlueprintStep(1.0, this::unClampStart, this::unClampPeriodic), 
				new BlueprintStep(5.0, this::backStart, this::backPeriodic),
				new BlueprintStep(4.0, this::forwardStart, this::forwardPeriodic),
				new BlueprintStep(1.5, this::waitStart, this::waitPeriodic),
				new BlueprintStep(1.0, this::clampStart, this::clampPeriodic)};
		setBlueprints(blueprints);
		
		start();
	}
	
	public int goStart() {
		String gameData;
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		
		robot.fourbarPIDControl.setSetpoint(50000);
		robot.fourbarPIDControl.enable();
		
		if(gameData.length() > 0)
		{
			if(gameData.charAt(0) == 'L')
			{

				robot.leftControl.configureTrajectory(robot.leftSwitchAutoTraj.getLeftTrajectory(), false);
				robot.rightControl.configureTrajectory(robot.leftSwitchAutoTraj.getRightTrajectory(), false);

			} else {
				robot.leftControl.configureTrajectory(robot.rightSwitchAutoTraj.getLeftTrajectory(), false);
				robot.rightControl.configureTrajectory(robot.rightSwitchAutoTraj.getRightTrajectory(), false);
			}


			robot.leftControl.enable();
			robot.rightControl.enable();
			
		}
		return 0;
	}
	

	
	public int goPeriodic() {
		if(robot.leftControl.isPlanFinished()&&robot.rightControl.isPlanFinished()) {
			
			robot.leftControl.reset();
			robot.rightControl.reset();
				
			nextStage();
		}
		return 0;
	}


	public int backStart() {
		String gameData;
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		
		if(gameData.length() > 0)
		{
			if(gameData.charAt(0) == 'L')
			{
				robot.leftControl.configureTrajectory(robot.leftSwitchBackTraj.getInvertedLeftTrajectory(), false);
				robot.rightControl.configureTrajectory(robot.leftSwitchBackTraj.getInvertedRightTrajectory(), false);
	
			} else {
				robot.leftControl.configureTrajectory(robot.rightSwitchBackTraj.getInvertedLeftTrajectory(), false);
				robot.rightControl.configureTrajectory(robot.rightSwitchBackTraj.getInvertedRightTrajectory(), false);
			}
	
	
			robot.leftControl.enable();
			robot.rightControl.enable();
			
		}
		
		
		
		return 0;
	}
	
	public int backPeriodic() {
		
		if(timer.get() >0.5 && timer.get()<1.0) {
			robot.rIntakeSetpoint = robot.clear;
			robot.rightIntakeControl.setSetpoint(robot.rIntakeSetpoint);
			robot.clamper.set(robot.clampIt);
			robot.fourbarSetpoint = 40000;
			robot.fourbarPIDControl.setSetpoint(robot.fourbarSetpoint);
		}
		if(timer.get()>1.0 && timer.get()<1.5) {
			robot.lIntakeSetpoint = robot.clear;
			robot.leftIntakeControl.setSetpoint(-robot.lIntakeSetpoint);
			robot.fourbarSetpoint = 30000;
			robot.fourbarPIDControl.setSetpoint(robot.fourbarSetpoint);
		} 
		if(robot.fourbarIsSafe(15000)&& (timer.get()>1.7 && timer.get()<2.0)){
			robot.fourbarSetpoint = 15000;
			robot.fourbarPIDControl.setSetpoint(robot.fourbarSetpoint);
		}
		if(robot.fourbarIsSafe(0) && timer.get()>2.0) {
			robot.fourbarSetpoint = 0;
			robot.fourbarPIDControl.setSetpoint(robot.fourbarSetpoint);
		}
		
		
		
		if(robot.leftControl.isPlanFinished()&&robot.rightControl.isPlanFinished()) {
			
			robot.leftControl.reset();
			robot.rightControl.reset();
				
			nextStage();
		}
		return 0;
	}

	
	public int unClampStart() {
		robot.clamper.set(robot.unClampIt);
		return 0;
	}
	
	public int unClampPeriodic() {
		nextStage();
		return 0;
	}
	
	public int forwardStart() {
		robot.rIntakeSetpoint = robot.open;
		robot.rightIntakeControl.setSetpoint(robot.rIntakeSetpoint);
		
		robot.lIntakeSetpoint = robot.open;
		robot.leftIntakeControl.setSetpoint(-robot.lIntakeSetpoint);
		
		robot.leftBelt.set(ControlMode.PercentOutput, -1.0);
		robot.rightBelt.set(ControlMode.PercentOutput, 1.0);
		
		robot.clamper.set(robot.unClampIt);
		
		robot.leftControl.configureTrajectory(robot.switchGrabCubeTraj.getLeftTrajectory(), false);
		robot.rightControl.configureTrajectory(robot.switchGrabCubeTraj.getRightTrajectory(), false);
		
		robot.leftControl.enable();
		robot.rightControl.enable();
		return 0;
	}
	
	public int forwardPeriodic() {
		if(robot.leftControl.isPlanFinished()&&robot.rightControl.isPlanFinished()) {
			
			robot.rIntakeSetpoint = robot.grab;
			robot.rightIntakeControl.setSetpoint(robot.rIntakeSetpoint);
			
			robot.lIntakeSetpoint = robot.grab;
			robot.leftIntakeControl.setSetpoint(-robot.lIntakeSetpoint);
			
			robot.leftControl.reset();
			robot.rightControl.reset();
				
			nextStage();
		}
		return 0;
	}
	
	public int waitStart() {
		return 0;
	}
	
	public int waitPeriodic() {
		return 0;
	}
	
	public int clampStart() {
		robot.clamper.set(robot.clampIt);
		robot.leftBelt.set(ControlMode.PercentOutput, 0.0);
		robot.rightBelt.set(ControlMode.PercentOutput, 0.0);
		return 0;
	}
	
	public int clampPeriodic() {
		nextStage();
		return 0;
	}
	
}

