package poptions;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

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
import burlap.mdp.singleagent.common.UniformCostRF;
import burlap.mdp.singleagent.model.FactoredModel;
import burlap.mdp.singleagent.model.RewardFunction;
import burlap.mdp.singleagent.oo.OOSADomain;

public class DoorWorldDomain extends ScaledGridWorld {

	public static final String					CLASS_DOOR = "door";

	public static final String					PF_AGENT_IN_DOOR = "agentInDoor";
	public static final String					PF_AGENT_IN_CORNER = "agentInCorner";
	
	public DoorWorldDomain(int width, int height) {
		super(width, height);
	}

	@Override
	public OOSADomain generateDomain() {
		return super.generateDomain().addStateClass(CLASS_DOOR,  GridDoor.class);
	}

	public List<PropositionalFunction> generatePfs(OOSADomain domain){
		ArrayList<PropositionalFunction> pfs = new ArrayList<PropositionalFunction>(super.generatePfs());
		pfs.add(new InDoorPF(PF_AGENT_IN_DOOR, domain, new String[]{CLASS_AGENT}));
		pfs.add(new InCornerPF(PF_AGENT_IN_CORNER, domain, new String[]{CLASS_AGENT}));
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

		private OOSADomain domain;
		
		public InDoorPF(String name, OOSADomain domain, String[] parameterClasses) {
			super(name, parameterClasses);
			this.domain = domain;
		}

		@Override
		public boolean isTrue(OOState st, String... params) {
			
			boolean n = domain.propFunction(PF_WALL_NORTH).isTrue(st, params);
			boolean s = domain.propFunction(PF_WALL_SOUTH).isTrue(st, params);
			boolean e = domain.propFunction(PF_WALL_EAST).isTrue(st, params);
			boolean w = domain.propFunction(PF_WALL_WEST).isTrue(st, params);
			if( ( n &&  s && !e && !w) ||
				(!n && !s &&  e &&  w)) {
				return true;
			}
			
			return false;
		}
	}
	
	public class InCornerPF extends PropositionalFunction{

		private OOSADomain domain;
		
		public InCornerPF (String name, OOSADomain domain, String[] parameterClasses) {
			super(name, parameterClasses);
			this.domain = domain;
		}

		@Override
		public boolean isTrue(OOState st, String... params) {

			boolean n = domain.propFunction(PF_WALL_NORTH).isTrue(st, params);
			boolean s = domain.propFunction(PF_WALL_SOUTH).isTrue(st, params);
			boolean e = domain.propFunction(PF_WALL_EAST).isTrue(st, params);
			boolean w = domain.propFunction(PF_WALL_WEST).isTrue(st, params);
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
		
		GridAgent agent = new GridAgent(ax, ay, CLASS_AGENT+"0");
		GridLocation loc0 = new GridLocation(lx, ly, CLASS_LOCATION, true);
		GridDoor door1 = new GridDoor(door1X, door1Y, CLASS_DOOR, true);
		GridDoor door2 = new GridDoor(door2X, door2Y, CLASS_DOOR, true);
		GridDoor door3 = new GridDoor(door3X, door3Y, CLASS_DOOR, true);
		GridDoor door4 = new GridDoor(door4X, door4Y, CLASS_DOOR, true);
		OOState s = new DoorWorldState(agent, loc0, door1, door2, door3, door4);
		System.out.println("1 agent, 1 loc, 4 doors");
		
		
//		State s = new MutableState();
//		
//		s.addObject(new ObjectInstance(d.getObjectClass(CLASSLOCATION), CLASSLOCATION+0));
//		s.addObject(new ObjectInstance(d.getObjectClass(CLASSAGENT), CLASSAGENT+0));
//
//		s.addObject(new ObjectInstance(d.getObjectClass(CLASSDOOR), CLASSDOOR+1));
//		s.addObject(new ObjectInstance(d.getObjectClass(CLASSDOOR), CLASSDOOR+2));
//		s.addObject(new ObjectInstance(d.getObjectClass(CLASSDOOR), CLASSDOOR+3));
//		s.addObject(new ObjectInstance(d.getObjectClass(CLASSDOOR), CLASSDOOR+4));
		
		return s;
	}

	
}
