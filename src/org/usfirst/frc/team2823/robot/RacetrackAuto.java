package org.usfirst.frc.team2823.robot;

public class RacetrackAuto extends Autonomous {
	
	public RacetrackAuto(Robot robot) {
		super(robot);
	}
	
	@Override
	public void init() {
		BlueprintStep[] blueprints = new BlueprintStep[] {new BlueprintStep(15.0, this::goStart, this::goPeriodic)};
		setBlueprints(blueprints);
		
		start();
	}
	
	public int goStart() {
		robot.configureToGear(robot.lowGear);
		
		robot.leftControl.configureTrajectory(robot.racetrackStartTraj.getLeftTrajectory(), false);
		robot.rightControl.configureTrajectory(robot.racetrackStartTraj.getRightTrajectory(), false);
		
		robot.leftControl.enable();
		robot.rightControl.enable();

		return 0;
	}
	
	public int goPeriodic() {
		if(robot.leftControl.getCurrentDistance()>= robot.racetrackTurnPlan[1][0]) {
			robot.configureToGear(robot.lowGear);
		}
		if(robot.leftControl.isPlanFinished()&&robot.rightControl.isPlanFinished()) {
			
			robot.leftControl.reset();
			robot.rightControl.reset();
				
			nextStage();
		}
		return 0;
	}
	
}
