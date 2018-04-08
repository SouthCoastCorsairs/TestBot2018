package org.usfirst.frc.team5846.robot.util.Paths;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Waypoint;

public class TestPath {
	
	public static Waypoint[] path = new Waypoint[] {
			new Waypoint(0.0, 0.0, Pathfinder.d2r(0.0)),
			new Waypoint(1.0, 1.0, Pathfinder.d2r(90.0)),
	};

}
