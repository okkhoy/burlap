package poption.domain;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

import javax.swing.JFrame;

import burlap.behavior.singleagent.auxiliary.EpisodeSequenceVisualizer;
import burlap.behavior.singleagent.auxiliary.StateReachability;
import burlap.domain.singleagent.gridworld.GridWorldDomain;
import burlap.domain.singleagent.gridworld.GridWorldVisualizer;
import burlap.domain.singleagent.gridworld.GridWorldDomain.GridWorldModel;
import burlap.domain.singleagent.gridworld.GridWorldDomain.WallToPF;
import burlap.domain.singleagent.gridworld.state.GridAgent;
import burlap.domain.singleagent.gridworld.state.GridLocation;
import burlap.domain.singleagent.gridworld.state.GridWorldState;
import burlap.mdp.auxiliary.common.NullTermination;
import burlap.mdp.core.Domain;
import burlap.mdp.core.TerminalFunction;
import burlap.mdp.core.action.UniversalActionType;
import burlap.mdp.core.oo.OODomain;
import burlap.mdp.core.oo.propositional.PropositionalFunction;
import burlap.mdp.core.oo.state.OOState;
import burlap.mdp.core.state.State;
import burlap.mdp.singleagent.SADomain;
import burlap.mdp.singleagent.common.UniformCostRF;
import burlap.mdp.singleagent.model.FactoredModel;
import burlap.mdp.singleagent.model.RewardFunction;
import burlap.mdp.singleagent.oo.OOSADomain;
import burlap.shell.visual.VisualExplorer;
import burlap.statehashing.simple.SimpleHashableStateFactory;
import burlap.visualizer.Visualizer;

public class DoorWorldDomain extends ScaledGridWorld {

	public static final String					CLASS_DOOR = "door";
	public static final String					PF_AGENT_IN_DOOR = "agentInDoor";
	public static final String					PF_AGENT_IN_CORNER = "agentInCorner";

	private PropositionalFunction pfWallToNorth = new WallToPF(PF_WALL_NORTH, new String[]{CLASS_AGENT}, 0);
	private PropositionalFunction pfWallToSouth = new WallToPF(PF_WALL_SOUTH, new String[]{CLASS_AGENT}, 1);
	private PropositionalFunction pfWallToEast = new WallToPF(PF_WALL_EAST, new String[]{CLASS_AGENT}, 2);
	private PropositionalFunction pfWallToWest = new WallToPF(PF_WALL_WEST, new String[]{CLASS_AGENT}, 3);
	
	public DoorWorldDomain(int width, int height) {
		super(width, height);
	}

	@Override
	public OOSADomain generateDomain() {

		OOSADomain domain = new OOSADomain();

		int [][] cmap = this.getMap();

		domain.addStateClass(CLASS_AGENT, GridAgent.class);
		domain.addStateClass(CLASS_LOCATION, GridLocation.class);
//		domain.addStateClass(CLASS_DOOR, GridDoor.class);

		GridWorldModel smodel = new GridWorldModel(cmap, getTransitionDynamics());
		RewardFunction rf = this.rf;
		TerminalFunction tf = this.tf;

		if(rf == null){
			rf = new UniformCostRF();
		}
		if(tf == null){
			tf = new NullTermination();
		}

		FactoredModel model = new FactoredModel(smodel, rf, tf);
		domain.setModel(model);

		domain.addActionTypes(
				new UniversalActionType(ACTION_NORTH),
				new UniversalActionType(ACTION_SOUTH),
				new UniversalActionType(ACTION_EAST),
				new UniversalActionType(ACTION_WEST));

		
		OODomain.Helper.addPfsToDomain(domain, this.generatePfs());
		
		return domain;
	}

	@Override
	public List<PropositionalFunction> generatePfs(){
		ArrayList<PropositionalFunction> pfs = new ArrayList<PropositionalFunction>(super.generatePfs());
		pfs.add(new InDoorPF(PF_AGENT_IN_DOOR, new String[]{CLASS_AGENT}));
		pfs.add(new InCornerPF(PF_AGENT_IN_CORNER, new String[]{CLASS_AGENT}));
		return pfs;
	}

	public void setMapOneDoorRandom(Random rng) {
		this.makeEmptyMap();
		
		door1X = 1 + rng.nextInt(width-2);
		door1Y = 1 + rng.nextInt(height-2);
		int direction = rng.nextInt(2);
		if (direction == 0) {
			horizontalWall(0, width-1, door1Y);
		} else {
			verticalWall(0, height-1, door1X);
		}
		
		this.map[door1X][door1Y] = 0;
	}
	
	public void setMapOneRoomDoorRandom(Random rng) {
		this.makeEmptyMap();
		
		int doorMin = 2;
		int direction = rng.nextInt(4);
		if (direction == 0) {
			door1X = doorMin + rng.nextInt(width-2-doorMin);
			door1Y = 1;
		} else if (direction == 1) {
			door1X = doorMin + rng.nextInt(width-2-doorMin);
			door1Y = height - 2;
		} else if (direction == 2) {
			door1X = 1;
			door1Y = doorMin + rng.nextInt(height-2-doorMin);
		} else if (direction == 3) {
			door1X = width - 2;
			door1Y = doorMin + rng.nextInt(height-2-doorMin);
		}
		
		horizontalWall(1, width-2, 1);
		horizontalWall(1, width-2, height-2);
		verticalWall(1, height-2, 1);
		verticalWall(1, height-2, width-2);
		
		this.map[door1X][door1Y] = 0;
	}
	
	public void setMapFourDoorsRandom(Random rng) {
		int halfWidth = (width)/2;
		int halfHeight = (height)/2;
		int d1x = rng.nextInt(halfWidth - 1);
		int d2y = rng.nextInt(halfHeight - 1);
		int d3x = halfWidth + 1 + rng.nextInt(halfWidth);
		int d4y = halfHeight + 1 + rng.nextInt(halfHeight);
		setMapToFourRooms(d1x, d2y, d3x, d4y);
	}

	
	public class InDoorPF extends PropositionalFunction{

		public InDoorPF(String name, String[] parameterClasses) {
			super(name, parameterClasses);
		}

		@Override
		public boolean isTrue(OOState st, String... params) {
			boolean n = pfWallToNorth.isTrue(st, params);
			boolean s = pfWallToSouth.isTrue(st, params);
			boolean e = pfWallToEast.isTrue(st, params);
			boolean w = pfWallToWest.isTrue(st, params);
			if( ( n &&  s && !e && !w) ||
				(!n && !s &&  e &&  w)) {
				return true;
			}
			return false;
		}
	}
	
	public class InCornerPF extends PropositionalFunction{

		public InCornerPF (String name, String[] parameterClasses) {
			super(name, parameterClasses);
		}

		@Override
		public boolean isTrue(OOState st, String... params) {
			boolean n = pfWallToNorth.isTrue(st, params);
			boolean s = pfWallToSouth.isTrue(st, params);
			boolean e = pfWallToEast.isTrue(st, params);
			boolean w = pfWallToWest.isTrue(st, params);
			if( (( n && !s) && (e || w)) ||
				(( s && !n) && (e || w)) ||
				(( e && !w) && (n || s)) ||
				(( w && !e) && (n || s))) {
				return true;
			}
			return false;
		}
	}


	public OOState getOneAgentOneLocationFourDoorsState(int ax, int ay, int lx, int ly){
		GridAgent agent = new GridAgent(ax, ay, CLASS_AGENT);
		GridLocation loc0 = new GridLocation(lx, ly, CLASS_LOCATION);
//		GridDoor door1 = new GridDoor(door1X, door1Y, CLASS_DOOR);
//		GridDoor door2 = new GridDoor(door2X, door2Y, CLASS_DOOR);
//		GridDoor door3 = new GridDoor(door3X, door3Y, CLASS_DOOR);
//		GridDoor door4 = new GridDoor(door4X, door4Y, CLASS_DOOR);
//		OOState s = new DoorWorldState(agent, loc0, door1, door2, door3, door4);
//		List<GridDoor> doors = new ArrayList<GridDoor>();
//		doors.add(door1);
//		doors.add(door2);
//		doors.add(door3);
//		doors.add(door4);
		OOState s = new DoorWorldState(agent, loc0);
		return s;
	}

	public OOState getOneAgentOneLocationOneDoorState(int ax, int ay, int lx, int ly){
		GridAgent agent = new GridAgent(ax, ay, CLASS_AGENT);
		GridLocation loc0 = new GridLocation(lx, ly, CLASS_LOCATION);
//		GridDoor door1 = new GridDoor(lx, ly, CLASS_DOOR);
//		OOState s = new DoorWorldState(agent, loc0, door1);
		OOState s = new DoorWorldState(agent, loc0);
		return s;
	}
	
	public static void main(String[] args) {
		DoorWorldDomain dw = new DoorWorldDomain(5, 5);
		dw.setMapFourDoorsRandom(new Random());
		OOState s = dw.getOneAgentOneLocationFourDoorsState(0,0,9,9);
		OOSADomain d = dw.generateDomain();
//
//		Visualizer v = GridWorldVisualizer.getVisualizer(dw.getMap());
//		VisualExplorer exp = new VisualExplorer(domain, v, state);
//
//		//set control keys to use w-s-a-d
//		exp.addKeyAction("w", GridWorldDomain.ACTION_NORTH, "");
//		exp.addKeyAction("s", GridWorldDomain.ACTION_SOUTH, "");
//		exp.addKeyAction("a", GridWorldDomain.ACTION_WEST, "");
//		exp.addKeyAction("d", GridWorldDomain.ACTION_EAST, "");
//		exp.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
//		exp.initGUI();
		
//		GridWorldDomain gwdg = new GridWorldDomain(11, 11);
//		gwdg.setMapToFourRooms();
//		SADomain d = gwdg.generateDomain();
//		GridWorldState s = new GridWorldState(new GridAgent(0, 0), new GridLocation(10, 10, "loc0"));

		
		StateReachability.getReachableStates(s, d, new SimpleHashableStateFactory());
	}

	
}
