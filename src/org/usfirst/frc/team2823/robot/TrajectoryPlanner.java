package org.usfirst.frc.team2823.robot;

import java.io.File;
import java.util.Arrays;

import edu.wpi.first.wpilibj.Timer;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;

public class TrajectoryPlanner {
	SnazzyLog log = new SnazzyLog();
	private Trajectory m_left;
	private Trajectory m_right;
	File rightFile;
	File leftFile;
	double[][] arrayPoints;
	private Trajectory m_trajectory; 
	private double m_maxA;
	private double m_maxV;
	private double m_maxJ;
	private double wheelbase = 25.125+3.05;  // 2.9 pretty good
	private String m_name;
	
	public TrajectoryPlanner(double[][] ap, double max_v, double max_a, double max_j, String name) {
		arrayPoints = ap;
		m_maxA = max_a;
		m_maxV = max_v;
		m_maxJ = max_j;
		m_name = name;
		
	}
	
    public void generate() {
    	Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.005, m_maxV, m_maxA, m_maxJ);
    	Waypoint[] points = new Waypoint[arrayPoints.length] ;

    	for (int i=0; i<arrayPoints.length;i++) {
    		points[i]=new Waypoint(arrayPoints[i][0], arrayPoints[i][1],Pathfinder.d2r(arrayPoints[i][2]));
    	}
    	File myFile = new File(getFileName());

    	if(myFile.exists()) {
    	m_trajectory = Pathfinder.readFromCSV(myFile);
    	}else{
    		m_trajectory = Pathfinder.generate(points, config);
    		Pathfinder.writeToCSV(myFile, m_trajectory);
    	}
    	System.out.println(m_trajectory.length());
    	
        regenerate();
    }
    
    public void regenerate() {
		TankModifier modifier = new TankModifier(m_trajectory).modify(wheelbase);
        // Do something with the new Trajectories...
        m_left = modifier.getLeftTrajectory();
        m_right = modifier.getRightTrajectory();        
        
        for (int i = 0; i < m_trajectory.length(); i++) {
            Trajectory.Segment seg = m_trajectory.get(i);
            
            log.open(m_name+"Trajectory.csv","Timestamp,X,Y,Position,Velocity,Accel,Jerk,Heading\n");
            log.write(seg.dt + "," + seg.x + "," + seg.y + "," + seg.position + "," + seg.velocity + "," + 
                    seg.acceleration + "," + seg.jerk + "," + seg.heading + "\n");
        }
        log.close();
        for (int i = 0; i < m_trajectory.length(); i++) {
            Trajectory.Segment seg = m_left.get(i);
            
            log.open(m_name+"LeftTrajectory.csv","Timestamp,X,Y,Position,Velocity,Accel,Jerk,Heading\n");
            log.write(seg.dt + "," + seg.x + "," + seg.y + "," + seg.position + "," + seg.velocity + "," + 
                    seg.acceleration + "," + seg.jerk + "," + seg.heading + "\n");
        }
        log.close();
        for (int i = 0; i < m_trajectory.length(); i++) {
            Trajectory.Segment seg = m_right.get(i);
            
            log.open(m_name+"RightTrajectory.csv","Timestamp,X,Y,Position,Velocity,Accel,Jerk,Heading\n");
            log.write(seg.dt + "," + seg.x + "," + seg.y + "," + seg.position + "," + seg.velocity + "," + 
                    seg.acceleration + "," + seg.jerk + "," + seg.heading + "\n");
        }
        log.close();
	}
    
    public Trajectory getLeftTrajectory() {
    	return m_left;
    }
    
    public Trajectory getInvertTrajectory(Trajectory t) {
    	Trajectory inverted = new Trajectory(t.length());
    	for(int i = 0; i<t.length();i++) {
    		inverted.segments[i] = t.segments[i].copy();
    		inverted.segments[i].x = t.segments[i].x *-1;
    		inverted.segments[i].y = t.segments[i].y *-1;
    		inverted.segments[i].acceleration = t.segments[i].acceleration*-1;
    		inverted.segments[i].jerk = t.segments[i].jerk*-1;
    		inverted.segments[i].position = t.segments[i].position*-1;
    		inverted.segments[i].velocity = t.segments[i].velocity*-1;
    		//TODO invert heading?
    	}
    	return inverted;
    }
    
    public Trajectory getInvertedLeftTrajectory() {
    	return getInvertTrajectory(m_right);
    }
    
    public Trajectory getInvertedRightTrajectory() {
    	return getInvertTrajectory(m_left);
    }
    
    public Trajectory getRightTrajectory() {
    	return m_right;
    }
    
    public String getFileName() {
    	String name = new String("/home/lvuser/"+m_name+"Trajectory.");
    	for (int i=0;i<arrayPoints.length;i++) {
    		name += Double.toString(arrayPoints[i][0])+"."+Double.toString(arrayPoints[i][1])+"."+Double.toString(arrayPoints[i][2]);
    	}
    	name += Double.toString(m_maxV)+Double.toString(m_maxA)+Double.toString(m_maxJ)+".csv";
    	return name;
    }
    
   public void  frankenstein( TrajectoryPlanner traj2, double v) {
    	Trajectory newTraj;
    	rename("Frankenstein"+m_name);
    	int chopOne = 0;
    	int chopTwo = traj2.m_trajectory.segments.length;
    	
    	for (int i = m_trajectory.segments.length-1; i>= 0 && m_trajectory.segments[i].velocity< v ; i--) {
    		chopOne = i;
    		
    	}
    	for (int i = 0;i <traj2.m_trajectory.segments.length && traj2.m_trajectory.segments[i].velocity < v; i++) {
    		chopTwo = i;
    		
    	}
    	
    	newTraj = new Trajectory(chopOne + (traj2.m_trajectory.segments.length-chopTwo));
    	double maxv1 = 0;
    	for (int i = 0; i< chopOne ; i++) {
    		newTraj.segments[i] = m_trajectory.segments[i].copy();
    		if(m_trajectory.segments[i].velocity>maxv1) {
    			maxv1 = m_trajectory.segments[i].velocity;
    		}
    	}
    	double maxv2 = 0;
    	for (int i = chopTwo;i <traj2.m_trajectory.segments.length; i++) {
    		
    		newTraj.segments[chopOne+i-chopTwo] = traj2.m_trajectory.segments[i].copy();
    		
    		newTraj.segments[chopOne+i-chopTwo].position += m_trajectory.segments[chopOne].position-traj2.m_trajectory.segments[chopTwo].position;
    		
    		if(traj2.m_trajectory.segments[i].velocity>maxv2) {
    			maxv2 = traj2.m_trajectory.segments[i].velocity;
    		}
    		
    	}
    	
    	System.out.println("chopOne " + chopOne + " "+ m_trajectory.segments[chopOne].position+ "; chopTwo " + chopTwo + " "+traj2.m_trajectory.segments[chopTwo].position);
    	System.out.println("Max V1: "+ maxv1+ "Max V2: "+ maxv2);
    	System.out.println(getFileName());
    	System.out.println(traj2.getFileName());
    	m_trajectory = newTraj;

    	regenerate();
    	flattenkAkV();
    }
   
   public void rename(String new_name) {
	   m_name = new_name;
   }
   
   public void flattenkAkV() {
	   for (int i = 0; i< m_trajectory.segments.length; i++) {
		   m_left.segments[i].acceleration = m_trajectory.segments[i].acceleration;
		   m_left.segments[i].velocity = m_trajectory.segments[i].velocity;
		   m_right.segments[i].acceleration = m_trajectory.segments[i].acceleration;
		   m_right.segments[i].velocity = m_trajectory.segments[i].velocity;
	   }
   }
}