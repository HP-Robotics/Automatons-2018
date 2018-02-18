package org.usfirst.frc.team2823.robot;

public class DriveForwardAuto extends Autonomous {
	
	public DriveForwardAuto(Robot robot) {
		super(robot);
	}
	
	@Override
	public void init() {
		BlueprintStep[] blueprints = new BlueprintStep[] {new BlueprintStep(15.0, this::goStart, this::goPeriodic)};
		setBlueprints(blueprints);
		
		start();
	}
	
	public int goStart() {
		robot.driveSolenoid.set(robot.highGear);
		
		robot.leftControl.configureTrajectory(robot.driveForwardTraj.getLeftTrajectory(), false);
		robot.rightControl.configureTrajectory(robot.driveForwardTraj.getRightTrajectory(), false);
		
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
}
