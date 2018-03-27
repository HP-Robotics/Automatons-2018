package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.DriverStation;

public class ScaleAuto extends Autonomous {
	
	public ScaleAuto(Robot robot) {
		super(robot);
	}
	
	@Override
	public void init() {
		BlueprintStep[] blueprints = new BlueprintStep[] {
		new BlueprintStep(15.0, this::goStart, this::goPeriodic),
		new BlueprintStep(5.0, this::spinnyStart, this::spinnyPeriodic),
		new BlueprintStep(3.0, this::elevatorUpStart, this::elevatorUpPeriodic),
		new BlueprintStep(15.0, this::endMoveStart, this::endMovePeriodic),
		new BlueprintStep(2.0, this::unclampStart, this::unclampPeriodic),
		new BlueprintStep(15.0, this::backUpStart, this::backUpPeriodic),
		new BlueprintStep(15.0, this::bringDownStart, this::bringDownPeriodic),
		};
		setBlueprints(blueprints);
		
		start();
	}
	
	public int goStart() { 
		robot.configureToGear(robot.highGear);
		String gameData;
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		
				
		if(gameData.length() > 0)
		{
			if(gameData.charAt(1) == 'L')
			{
				robot.leftControl.configureTrajectory(robot.leftScaleStartTraj.getLeftTrajectory(), false);
				robot.rightControl.configureTrajectory(robot.leftScaleStartTraj.getRightTrajectory(), false);
				
			}
			else {
				robot.leftControl.configureTrajectory(robot.rightScaleStartTraj.getLeftTrajectory(), false);
				robot.rightControl.configureTrajectory(robot.rightScaleStartTraj.getRightTrajectory(), false);
			}
		}
		robot.leftControl.enable();
		robot.rightControl.enable();
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
	
	public int spinnyStart() {
		String gameData;
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		

		if(gameData.length() > 0)
		{
			if(gameData.charAt(1) == 'L')
			{
				nextStage();
			}
			else {
				//robot.leftControl.configureTrajectory(robot.rightScaleEndTraj.getLeftTrajectory(), false);
				//robot.rightControl.configureTrajectory(robot.rightScaleEndTraj.getRightTrajectory(), false);
				robot.leftControl.configureGoal(-135 * (38.0/160), 50, 50, false);
				robot.rightControl.configureGoal(135* (38.0/160), 50, 50, false);
			}
		}

		robot.leftControl.enable();
		robot.rightControl.enable();
		return 0;
	}
	
	public int spinnyPeriodic() {
		if(robot.leftControl.isPlanFinished()&&robot.rightControl.isPlanFinished()) {
			
			robot.leftControl.reset();
			robot.rightControl.reset();
				
			nextStage();
		}
		return 0;
	}
	
	public int elevatorUpStart() {
		String gameData;
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		robot.configureToGear(robot.lowGear);
		
		/*if(gameData.length() == 0 || gameData.charAt(1)!= 'L')
		{
			stopAll();
			return 0;
		}*/
		
		robot.elevatorPIDControl.setSetpoint(7800);
		robot.elevatorPIDControl.enable();
		robot.fourbarPIDControl.setSetpoint(90000);
		robot.fourbarPIDControl.enable();

		return 0;
	}
	public int elevatorUpPeriodic() {
		return 0;
	}
	
	public int endMoveStart() {
		String gameData;
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		

		if(gameData.length() > 0)
		{
			if(gameData.charAt(1) == 'L')
			{
				robot.leftControl.configureTrajectory(robot.leftScaleEndTraj.getLeftTrajectory(), false);
				robot.rightControl.configureTrajectory(robot.leftScaleEndTraj.getRightTrajectory(), false);
			}
			else {
				//robot.leftControl.configureTrajectory(robot.rightScaleEndTraj.getLeftTrajectory(), false);
				//robot.rightControl.configureTrajectory(robot.rightScaleEndTraj.getRightTrajectory(), false);
				robot.leftControl.configureGoal(40, 50, 50, false);
				robot.rightControl.configureGoal(40, 50, 50, false);
			}
		}

		robot.leftControl.enable();
		robot.rightControl.enable();
		return 0;
	}
	
	public int endMovePeriodic() {
		if(robot.leftControl.isPlanFinished()&&robot.rightControl.isPlanFinished()) {
			
			robot.leftControl.reset();
			robot.rightControl.reset();
				
			nextStage();
		}
		return 0;
	}

	public int unclampStart() {
		String gameData;
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		if(gameData.length()>0) {
			robot.clamper.set(robot.unClampIt);
		} else {
			nextStage();
		}
		return 0;
	}
	public int unclampPeriodic() {
		nextStage();
		return 0;
	}
	
	public int backUpStart() {

		String gameData;
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		

		if(gameData.length() > 0)
		{
			if(gameData.charAt(1) == 'L')
			{
				//robot.leftControl.configureTrajectory(robot.leftScaleBackTraj.getInvertedLeftTrajectory(), false);
				//robot.rightControl.configureTrajectory(robot.leftScaleBackTraj.getInvertedRightTrajectory(), false);

				robot.leftControl.configureGoal(-40, 50, 100);
				robot.rightControl.configureGoal(-40, 50, 100);
			}
			else {
				//robot.leftControl.configureTrajectory(robot.rightScaleBackTraj.getInvertedLeftTrajectory(), false);
				//robot.rightControl.configureTrajectory(robot.rightScaleBackTraj.getInvertedRightTrajectory(), false);

				robot.leftControl.configureGoal(-40, 50, 100);
				robot.rightControl.configureGoal(-40, 50, 100);
			}
		}
		
		robot.leftControl.enable();
		robot.rightControl.enable();

		return 0;
	}

	public int backUpPeriodic() {
		
		if(timer.get() >0 && timer.get()<0.5) {
			robot.rIntakeSetpoint = robot.rClear;
			robot.rightIntakeControl.setSetpoint(robot.rIntakeSetpoint);
			
		}
		if(timer.get()>0.5 && timer.get()<1.0) {
			robot.lIntakeSetpoint = robot.lClear;
			robot.leftIntakeControl.setSetpoint(robot.lIntakeSetpoint);
		} 
		if(robot.leftControl.isPlanFinished()&&robot.rightControl.isPlanFinished()) {
			
			robot.leftControl.reset();
			robot.rightControl.reset();
				
			nextStage();
		}
		return 0;
	}

	public int bringDownStart() {
		robot.clamper.set(robot.clampIt);
		robot.fourbarSetpoint = robot.upperSafeZoneLimit;
		robot.fourbarPIDControl.setSetpoint(robot.fourbarSetpoint);
		robot.elevatorPIDControl.setSetpoint(0);
		robot.elevatorPIDControl.enable();
		return 0;
	}
	
	public int bringDownPeriodic() {
		if(robot.fourbarIsSafe(0) && timer.get()>1.0) {
			robot.fourbarSetpoint = 0;
			robot.fourbarPIDControl.setSetpoint(robot.fourbarSetpoint);
		}
		return 0;
	}
	
}
