package org.usfirst.frc.team2823.robot;

import java.io.File;
import java.util.Arrays;

import edu.wpi.first.wpilibj.Timer;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
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
	
	public TrajectoryPlanner(double[][] ap, double max_v, double max_a, double max_j) {
		arrayPoints = ap;
		m_maxA = max_a;
		m_maxV = max_v;
		m_maxJ = max_j;
		
	}
	
    public void generate() {
    	Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.005, m_maxV, m_maxA, m_maxJ);
    	Waypoint[] points = new Waypoint[arrayPoints.length] ;
    	// new Waypoint(-4, -1, Pathfinder.d2r(-45)),
    	for (int i=0; i<arrayPoints.length;i++) {
    		points[i]=new Waypoint(arrayPoints[i][0], arrayPoints[i][1],arrayPoints[i][2]);
    	}
    	File myFile = new File(getFileName());

    	if(myFile.exists()) {
    	m_trajectory = Pathfinder.readFromCSV(myFile);
    	}else{
    		m_trajectory = Pathfinder.generate(points, config);
    		Pathfinder.writeToCSV(myFile, m_trajectory);
    	}

        // Wheelbase Width = 0.5m
        TankModifier modifier = new TankModifier(m_trajectory).modify(35);
        // Do something with the new Trajectories...
        m_left = modifier.getLeftTrajectory();
        m_right = modifier.getRightTrajectory();
        
        
        //File myFile = new File("/home/lvuser/myfile.csv");
        //Pathfinder.writeToCSV(myFile, trajectory);
        
        for (int i = 0; i < m_trajectory.length(); i++) {
            Trajectory.Segment seg = m_trajectory.get(i);
            
            log.open("Trajectory.csv","Timestamp,X,Y,Position,Velocity,Accel,Jerk,Heading\n");
            log.write(seg.dt + "," + seg.x + "," + seg.y + "," + seg.position + "," + seg.velocity + "," + 
                    seg.acceleration + "," + seg.jerk + "," + seg.heading + "\n");
        }
        log.close();
        for (int i = 0; i < m_trajectory.length(); i++) {
            Trajectory.Segment seg = m_left.get(i);
            
            log.open("LeftTrajectory.csv","Timestamp,X,Y,Position,Velocity,Accel,Jerk,Heading\n");
            log.write(seg.dt + "," + seg.x + "," + seg.y + "," + seg.position + "," + seg.velocity + "," + 
                    seg.acceleration + "," + seg.jerk + "," + seg.heading + "\n");
        }
        log.close();
        for (int i = 0; i < m_trajectory.length(); i++) {
            Trajectory.Segment seg = m_right.get(i);
            
            log.open("RightTrajectory.csv","Timestamp,X,Y,Position,Velocity,Accel,Jerk,Heading\n");
            log.write(seg.dt + "," + seg.x + "," + seg.y + "," + seg.position + "," + seg.velocity + "," + 
                    seg.acceleration + "," + seg.jerk + "," + seg.heading + "\n");
        }
        log.close();
    }
    
    public Trajectory getLeftTrajectory() {
    	return m_left;
    }
    
    public Trajectory getRightTrajectory() {
    	return m_right;
    }
    
    public String getFileName() {
    	String name = new String("/home/lvuser/Trajectory.");
    	for (int i=0;i<arrayPoints.length;i++) {
    		name += Double.toString(arrayPoints[i][0])+"."+Double.toString(arrayPoints[i][1])+"."+Double.toString(arrayPoints[i][2]);
    	}
    	name += Double.toString(m_maxV)+Double.toString(m_maxA)+Double.toString(m_maxJ)+".csv";
    	return name;
    }

}