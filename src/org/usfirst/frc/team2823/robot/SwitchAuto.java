package org.usfirst.frc.team2823.robot;

import edu.wpi.first.wpilibj.DriverStation;

public class SwitchAuto extends Autonomous {
	
	public SwitchAuto(Robot robot) {
		super(robot);
	}
	
	@Override
	public void init() {
		BlueprintStep[] blueprints = new BlueprintStep[] {new BlueprintStep(5.0, this::goStart, this::goPeriodic), 
				new BlueprintStep(1.0, this::unClampStart, this::unClampPeriodic)/*, 
				new BlueprintStep(5.0, this::backStart, this::goPeriodic)*/};
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
				robot.leftControl.configureTrajectory(robot.leftSwitchAutoTraj.getInvertedLeftTrajectory(), false);
				robot.rightControl.configureTrajectory(robot.leftSwitchAutoTraj.getInvertedRightTrajectory(), false);
	
			} else {
				robot.leftControl.configureTrajectory(robot.rightSwitchAutoTraj.getInvertedLeftTrajectory(), false);
				robot.rightControl.configureTrajectory(robot.rightSwitchAutoTraj.getInvertedRightTrajectory(), false);
			}
	
	
			robot.leftControl.enable();
			robot.rightControl.enable();
			
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
	
}
