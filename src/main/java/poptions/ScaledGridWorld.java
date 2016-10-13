package poptions;

import burlap.domain.singleagent.gridworld.GridWorldDomain;
import burlap.domain.singleagent.gridworld.GridWorldVisualizer;
import burlap.domain.singleagent.gridworld.state.GridAgent;
import burlap.domain.singleagent.gridworld.state.GridLocation;
import burlap.domain.singleagent.gridworld.state.GridWorldState;
import burlap.mdp.singleagent.SADomain;
import burlap.shell.EnvironmentShell;
import burlap.shell.visual.VisualExplorer;
import burlap.visualizer.Visualizer;

public class ScaledGridWorld extends GridWorldDomain {

	public int door1X;
	public int door1Y;
	public int door2X;
	public int door2Y;
	public int door3X;
	public int door3Y;
	public int door4X;
	public int door4Y;
	
	public ScaledGridWorld(int width, int height){
		super(width, height);
	}
	
	@Override
	public void setMapToFourRooms() {
		setMapToFourRooms(-1, -1, -1, -1);
	}
	
	public void setMapToFourRooms(int d1x, int d2y, int d3x, int d4y){
		this.makeEmptyMap();
		
		int halfWidth = (width)/2;
		int halfHeight = (height)/2;
		
		door1X = d1x == -1 ? 1 : d1x;
		door1Y = halfHeight;
		door2X = halfWidth;
		door2Y = d2y == -1 ? 1 : d2y;
		door3X = d3x == -1 ? 1 + halfWidth + halfWidth/2 : d3x;
		door3Y = halfHeight;
		door4X = halfWidth;
		door4Y = d4y == -1 ? 1 + halfHeight + halfHeight/2 : d4y;
		
		horizontalWall(0, door1X-1, halfHeight);
		horizontalWall(door1X+1, halfWidth-1, halfHeight);
		horizontalWall(halfWidth+1, door3X-1, halfHeight);
		horizontalWall(door3X+1, width-1, halfHeight);
		
		verticalWall(0, door2Y-1, halfWidth);
		verticalWall(door2Y+1, door4Y-1, halfWidth);
		verticalWall(door4Y+1, height-1, halfWidth);
	}
	
}
