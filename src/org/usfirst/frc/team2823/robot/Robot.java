/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2823.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot {

	Joystick driveStick;
	Joystick operatorStick;
	
	final int xButton = 1;
	final int aButton = 2;
	final int bButton = 3;
	final int yButton = 4;
	final int leftBumper = 5;
	final int rightBumper = 6;
	final int leftTrigger = 7;
	final int rightTrigger = 8;
	
	Button intakeOpenButton;
	Button intakeCubeButton; 
	Button intakeStowButton;
	Button dropCubeButton;
	Button gearHighButton;
	Button gearLowButton;
	
	Button testButton;

	TalonSRX leftMotor1;
	TalonSRX leftMotor2;
	TalonSRX rightMotor3;
	TalonSRX rightMotor4;
	
	VictorSPX leftElbow;
	VictorSPX rightElbow;
	VictorSPX leftBelt;
	VictorSPX rightBelt;
	
	TalonSRX fourbarMotor;
	VictorSPX elevatorMotor;

	DoubleSolenoid driveSolenoid;
	Compressor compressor;
	
	Encoder lDriveEncoder;
	Encoder rDriveEncoder;
	Encoder lIntakeEncoder;
	Encoder rIntakeEncoder;
	Encoder fourbarEncoder;
	Encoder elevatorEncoder;
	
	DriveInchesPIDSource leftInches;
	DriveInchesPIDSource rightInches;
	
	
	SnazzyPIDController leftPIDControl;
	SnazzyPIDController rightPIDControl;
	
	SnazzyPIDController fourbarControl;
	FourbarOutput fourbarOutput;
	
	SnazzyMotionPlanner leftControl;
	SnazzyMotionPlanner rightControl;
	
	LeftDrivePIDOutput lDriveOutput;
	RightDrivePIDOutput rDriveOutput;
	
	TrajectoryPlanner rightSwitchAutoTraj;
	TrajectoryPlanner leftSwitchAutoTraj;
	
	double[][] rightSwitchAutoPlan = {{0,0,0},{60, 20, 0}};
	double[][] leftSwitchAutoPlan = {{0,0,0},{60, -20, 0}};

	boolean calibrate = false;
	boolean pidTune = false;
	
	SendableChooser<Autonomous> autonomousChooser;
	
	final static double ENC_TO_INCH = Math.PI * 6.0 * (24/60) * (1/3) * (1/256);
	final static double INCH_TO_ENC = 1/ENC_TO_INCH;
	
	/*  1 spin of the wheel corresponds to 60/24 spins of the output axle (that's our 3rd stage gearing).
	 *  The output axle is connected to a 36 t 'encoder gear' which spins another 12t 'encoder gear', so the encoder axles spins at 3 times that rate. 
	 *  Then we have 256 ticks per revolution.  So I think that's a total of 60 / 24 * 3 * 256 = 1920  ticks per revolution
	 */
	
	DoubleSolenoid.Value highGear = DoubleSolenoid.Value.kReverse;
	DoubleSolenoid.Value lowGear = DoubleSolenoid.Value.kForward;
	
	double transmissionUpper = 50.0;
	double transmissionLower = 30.0;
	
	double velocity = 0.0;
	double rCurrentDist = 0.0;
	double lCurrentDist = 0.0;
	double currentTime = 0.0;
	double rLastDist = 0.0;
	double lLastDist = 0.0;
	double lastTime = 0.0;
		
	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		autonomousChooser = new SendableChooser<Autonomous>();
		//autonomousChooser.addDefault("Empty: Do Nothing", new EmptyAutonomous(this));
		autonomousChooser.addObject("Switch Auto", new RightSwitchAuto(this));
		autonomousChooser.addObject("Do a Spin", new SpinnyAuto(this));
		SmartDashboard.putData("Autonomous Mode", autonomousChooser);
		
		/** UsbCamera c = CameraServer.getInstance().startAutomaticCapture();
	        c.setResolution(320, 180);
	        c.setFPS(29);
	        **/
		driveStick = new Joystick(0);
		operatorStick = new Joystick(1);

		intakeOpenButton = new Button();
		intakeCubeButton = new Button();
		intakeStowButton = new Button();
		dropCubeButton = new Button();
		gearHighButton = new Button();
		gearLowButton = new Button();
		
		testButton = new Button();
		
		leftMotor1 = new TalonSRX(1);
		leftMotor2 = new TalonSRX(2);
		rightMotor3 = new TalonSRX(3);
		rightMotor4 = new TalonSRX(4);
		
		leftMotor1.configOpenloopRamp(0, 0);
		leftMotor2.configOpenloopRamp(0, 0);
		rightMotor3.configOpenloopRamp(0, 0);
		rightMotor4.configOpenloopRamp(0, 0);
		
		leftElbow = new VictorSPX(11);
		rightElbow = new VictorSPX(12);
		leftBelt = new VictorSPX(21);
		rightBelt = new VictorSPX(22);
		
		fourbarMotor = new TalonSRX(31);
		fourbarOutput = new FourbarOutput();
		elevatorMotor = new VictorSPX(41);
			
		lDriveEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
		lDriveEncoder.setDistancePerPulse(1);
		rDriveEncoder = new Encoder(2, 3, true, Encoder.EncodingType.k4X);
		rDriveEncoder.setDistancePerPulse(1);
		
		lIntakeEncoder = new Encoder(4, 5, false, Encoder.EncodingType.k4X);
		rIntakeEncoder = new Encoder(6, 7, false, Encoder.EncodingType.k4X);
		
		leftInches = new DriveInchesPIDSource(lDriveEncoder);
		rightInches = new DriveInchesPIDSource(rDriveEncoder);
		
		fourbarEncoder = new Encoder(8,9, false, Encoder.EncodingType.k4X);
		elevatorEncoder = new Encoder( 10, 11, false, Encoder.EncodingType.k4X);

		driveSolenoid = new DoubleSolenoid(0, 1);
		compressor = new Compressor(0);

		driveSolenoid.set(lowGear);

		lDriveEncoder.reset();
		rDriveEncoder.reset();
		lIntakeEncoder.reset();
		rIntakeEncoder.reset();
		fourbarEncoder.reset();
		elevatorEncoder.reset();
	
		lDriveOutput = new LeftDrivePIDOutput(this);
		rDriveOutput = new RightDrivePIDOutput(this);
		
		rightSwitchAutoTraj = new TrajectoryPlanner(rightSwitchAutoPlan, 90, 3000, 6000);
		rightSwitchAutoTraj.generate();
		
		leftSwitchAutoTraj = new TrajectoryPlanner(leftSwitchAutoPlan, 90, 3000, 6000);
		rightSwitchAutoTraj.generate();
		
		leftControl = new SnazzyMotionPlanner(0.1, 0.001, 0.3, 0, 0.00143, 0.0102,  leftInches, lDriveOutput, 0.005, "Left.csv");
		rightControl= new SnazzyMotionPlanner(0.1, 0.001, 0.3, 0, 0.00143, 0.0102,  rightInches, rDriveOutput, 0.005,"Right.csv");
		
		leftPIDControl = new SnazzyPIDController(0.04, 0.001, 0.8, 0, leftInches, lDriveOutput, 0.005, "Left.csv");
		rightPIDControl= new SnazzyPIDController(0.04, 0.001, 0.8, 0, rightInches, rDriveOutput, 0.005,"Right.csv");
		
		fourbarControl = new SnazzyPIDController(0.1, 0.0, 0.0, 0.0, fourbarEncoder, fourbarOutput, 0.005, "Fourbar.csv");
		
		SmartDashboard.putNumber("P", 0.01);
		SmartDashboard.putNumber("I", 0.0);
		SmartDashboard.putNumber("D", 0.0);
		SmartDashboard.putNumber("Setpoint", 0);
		SmartDashboard.putNumber("L Encoder", 0);
		SmartDashboard.putNumber("R Encoder", 0);
		SmartDashboard.putNumber("L Elbow", 0);
		SmartDashboard.putNumber("R Elbow", 0);
		SmartDashboard.putString("Driving Gear", "Low");
	}

	@Override
	public void autonomousInit() {
		
		lDriveEncoder.reset();
		rDriveEncoder.reset();
		lIntakeEncoder.reset();
		rIntakeEncoder.reset();
		fourbarEncoder.reset();
		elevatorEncoder.reset();
		
		leftControl.reset();
		rightControl.reset();
		
		driveSolenoid.set(lowGear);
		
		((Autonomous) autonomousChooser.getSelected()).init();
		
		
	}

	@Override
	public void autonomousPeriodic() {
		((Autonomous) autonomousChooser.getSelected()).periodic();
		
		SmartDashboard.putNumber("L Encoder", lDriveEncoder.get());
		SmartDashboard.putNumber("R Encoder", rDriveEncoder.get());
	}

	@Override
	public void teleopPeriodic() {
		
		calibrate = false;
		pidTune = false;
		
		if(!calibrate && pidTune) {
			
			currentTime = Timer.getFPGATimestamp();
			rCurrentDist = rightInches.pidGet();
			lCurrentDist = leftInches.pidGet();
			
			velocity  = ((lCurrentDist-lLastDist)/(currentTime-lastTime)+(rCurrentDist-rLastDist)/(currentTime-lastTime))/2;
			
			gearLowButton.update(driveStick.getRawButton(leftTrigger));
			gearHighButton.update(driveStick.getRawButton(leftBumper));
			intakeOpenButton.update(driveStick.getRawButton(rightBumper));
			intakeCubeButton.update(driveStick.getRawButton(rightTrigger));
			intakeStowButton.update(driveStick.getRawButton(xButton));
			dropCubeButton.update(driveStick.getRawButton(bButton));
			testButton.update(driveStick.getRawButton(1));
			
			/*if(velocity >= 50.0 && driveSolenoid.get() == lowGear) {
				driveSolenoid.set(highGear);
				SmartDashboard.putString("Driving Gear", "High");
			} 
			
			if(velocity <= 30.0 && driveSolenoid.get() == highGear) {
				driveSolenoid.set(lowGear);
				SmartDashboard.putString("Driving Gear", "Low");
			}*/
	
			if (gearLowButton.changed()) {
				driveSolenoid.set(lowGear);
				SmartDashboard.putString("Driving Gear", "Low");
	
			}
			if (gearHighButton.changed()) {
				driveSolenoid.set(highGear);
				SmartDashboard.putString("Driving Gear", "High");
				
			}
			
			SmartDashboard.putNumber("L Encoder", leftInches.pidGet());
			SmartDashboard.putNumber("R Encoder", rightInches.pidGet());
			SmartDashboard.putNumber("L Elbow", rDriveEncoder.get());
			SmartDashboard.putNumber("R Elbow", rDriveEncoder.get());
			
			leftMotor1.set(ControlMode.PercentOutput, Math.pow(driveStick.getRawAxis(1), 1) );
			leftMotor2.set(ControlMode.PercentOutput, Math.pow(driveStick.getRawAxis(1), 1) );
			rightMotor3.set(ControlMode.PercentOutput, -Math.pow(driveStick.getRawAxis(3), 1));
			rightMotor4.set(ControlMode.PercentOutput, -Math.pow(driveStick.getRawAxis(3), 1));
			
			elevatorMotor.set(ControlMode.PercentOutput, operatorStick.getRawAxis(3)*.4);
			
			rLastDist = rCurrentDist;
			lLastDist = lCurrentDist;
			lastTime = currentTime;
		
		} else if(calibrate && !pidTune) {
			calibrateNow();
		}else {
			pidTune();
		}
		
	}


	@Override
	public void testInit() {
		leftControl.disable();
		rightControl.disable();
	}

	@Override
	public void testPeriodic() {
		
	}
	public void disabledInit() {
		
	}
	
	public void calibrateNow() {
		testButton.update(driveStick.getRawButton(1));
		
		if(testButton.on()){
			if(testButton.changed()) {
				lDriveEncoder.reset();
				rDriveEncoder.reset();
				
				leftControl.startCalibration();
				rightControl.startCalibration();
				
				leftControl.enable();
				rightControl.enable();

			}
			
		}else if (testButton.changed()&& !testButton.on()){
			leftControl.disable();
			rightControl.disable();
			
			
		}
		SmartDashboard.putNumber("L Encoder", leftInches.pidGet());
		SmartDashboard.putNumber("R Encoder", rightInches.pidGet());
	}
	
	public void pidTune() {
		testButton.update(driveStick.getRawButton(1));
		
		leftPIDControl.setPID(SmartDashboard.getNumber("P", 0), SmartDashboard.getNumber("I", 0), SmartDashboard.getNumber("D", 0));
		rightPIDControl.setPID(SmartDashboard.getNumber("P", 0), SmartDashboard.getNumber("I", 0), SmartDashboard.getNumber("D", 0));
		
			if(testButton.on()){
				if(testButton.changed()) {
					lDriveEncoder.reset();
					rDriveEncoder.reset();
					
					leftPIDControl.setSetpoint(SmartDashboard.getNumber("Setpoint", 0));
					leftPIDControl.enable();
					
					rightPIDControl.setSetpoint(SmartDashboard.getNumber("Setpoint", 0));
					rightPIDControl.enable();
	
				}
				
			}else if (testButton.changed()&& !testButton.on()){
				leftPIDControl.disable();
				rightPIDControl.disable();
				
			}
		SmartDashboard.putNumber("L Encoder", leftInches.pidGet());
		SmartDashboard.putNumber("R Encoder", rightInches.pidGet());
	}
}
