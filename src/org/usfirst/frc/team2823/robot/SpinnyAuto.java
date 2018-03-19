package org.usfirst.frc.team2823.robot;

public class SpinnyAuto extends Autonomous {
	
	public SpinnyAuto(Robot robot) {
		super(robot);
	}
	
	@Override
	public void init() {
		BlueprintStep[] blueprints = new BlueprintStep[] {new BlueprintStep(7.0, this::spinStart, this::spinPeriodic)};
		setBlueprints(blueprints);
		
		start();
	}
	
	public int spinStart() {
		robot.leftControl.configureGoal(90, 300, 300, false);
		robot.rightControl.configureGoal(-90, 300, 300, false);
			
		robot.leftControl.enable();
		robot.rightControl.enable();
		
		return 0;
	}
	
	public int spinPeriodic() {
		if(robot.leftControl.isPlanFinished()&&robot.rightControl.isPlanFinished()) {
			
			robot.leftControl.reset();
			robot.rightControl.reset();
			System.out.println(robot.gyro.getAngle());
			
			nextStage();
			
		}
		return 0;
	}
}
