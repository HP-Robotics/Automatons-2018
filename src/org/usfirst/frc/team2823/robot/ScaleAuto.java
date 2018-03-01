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
		new BlueprintStep(15.0, this::firstTurnStart, this::firstTurnPeriodic),
		new BlueprintStep(15.0, this::secondGoStart, this::secondGoPeriodic),
		new BlueprintStep(15.0, this::lastTurnStart, this::lastTurnPeriodic),
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
	
	public int firstTurnStart() {
		String gameData;
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		robot.configureToGear(robot.lowGear);
		
		if(gameData.length() > 0)
		{
			if(gameData.charAt(1) == 'L')
			{
				robot.leftControl.configureTrajectory(robot.leftScaleEndTraj.getLeftTrajectory(), false);
				robot.rightControl.configureTrajectory(robot.leftScaleEndTraj.getRightTrajectory(), false);
				
			}
			else {
				robot.leftControl.configureTrajectory(robot.rightScaleFirstTurnTraj.getLeftTrajectory(), false);
				robot.rightControl.configureTrajectory(robot.rightScaleFirstTurnTraj.getRightTrajectory(), false);
			}
		}
		robot.leftControl.enable();
		robot.rightControl.enable();
		return 0;
	}
	
	public int firstTurnPeriodic() {
		if(robot.leftControl.isPlanFinished()&&robot.rightControl.isPlanFinished()) {
			
			robot.leftControl.reset();
			robot.rightControl.reset();
				
			nextStage();
		}
		return 0;
	}
	
	public int secondGoStart() {
		String gameData;
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		
		
		if(gameData.length() > 0)
		{
			if(gameData.charAt(1) == 'L')
			{
				nextStage();
				return 0;
				
			}
			else {
				robot.configureToGear(robot.highGear);
				robot.leftControl.configureTrajectory(robot.rightScaleMidTraj.getLeftTrajectory(), false);
				robot.rightControl.configureTrajectory(robot.rightScaleMidTraj.getRightTrajectory(), false);
				robot.leftControl.enable();
				robot.rightControl.enable();
			}
		}
		
		return 0;
	}
	public int secondGoPeriodic() {
		if(robot.leftControl.isPlanFinished()&&robot.rightControl.isPlanFinished()) {
			
			robot.leftControl.reset();
			robot.rightControl.reset();
				
			nextStage();
		}
		return 0;
	}
	
	public int lastTurnStart() {
		String gameData;
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		
		
		if(gameData.length() > 0)
		{
			if(gameData.charAt(1) == 'L')
			{
				nextStage();
				return 0;
				
			}
			else {
				robot.configureToGear(robot.lowGear);
				
				robot.leftControl.configureTrajectory(robot.rightScaleEndTraj.getLeftTrajectory(), false);
				robot.rightControl.configureTrajectory(robot.rightScaleEndTraj.getRightTrajectory(), false);
				robot.leftControl.enable();
				robot.rightControl.enable();
			}
		}
		
		return 0;
	}
	
	public int lastTurnPeriodic() {
		if(robot.leftControl.isPlanFinished()&&robot.rightControl.isPlanFinished()) {
			
			robot.leftControl.reset();
			robot.rightControl.reset();
				
			nextStage();
		}
		return 0;
	}
}
