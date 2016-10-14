package poption.domain.cleanup.state;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import burlap.debugtools.RandomFactory;
import burlap.mdp.auxiliary.StateGenerator;
import burlap.mdp.core.oo.propositional.PropositionalFunction;
import burlap.mdp.core.oo.state.ObjectInstance;
import burlap.mdp.core.state.State;
import poption.domain.cleanup.CleanupGoalDescription;
import poption.domain.cleanup.CleanupWorld;

public class CleanupRandomStateGenerator implements StateGenerator {

	public int numBlocks = 2;
	
	@Override
	public State generateState() {

		Random rng = RandomFactory.getMapped(0);

		int numRooms = 4;
		int numDoors = 4;
		
		int width = 13;
		int height = 13;
		
		int y1 = 3;
		int y2 = 7;
		int y3 = 12;

		int x1 = 4;
		int x2 = 8;
		int x3 = 12;

		int ax = 7;
		int ay = 1;
		String agentDirection = CleanupWorld.ACTION_NORTH;

		CleanupWorldState s = new CleanupWorldState(width, height, ax, ay, agentDirection, numBlocks, numRooms, numDoors);
		
//		s.addObject(new CleanupBlock("block0", bx, by, bShape, bColor));
		
		s.addObject(new CleanupRoom("room0", x1, x2, 0, y2, CleanupWorld.COLORS[rng.nextInt(CleanupWorld.COLORS.length)], CleanupWorld.SHAPE_ROOM));
		s.addObject(new CleanupRoom("room1", 0, x1, y1, y2, CleanupWorld.COLORS[rng.nextInt(CleanupWorld.COLORS.length)], CleanupWorld.SHAPE_ROOM));
		s.addObject(new CleanupRoom("room2", 0, x3, y2, y3, CleanupWorld.COLORS[rng.nextInt(CleanupWorld.COLORS.length)], CleanupWorld.SHAPE_ROOM));
		s.addObject(new CleanupRoom("room3", x2, x3, 0, y2, CleanupWorld.COLORS[rng.nextInt(CleanupWorld.COLORS.length)], CleanupWorld.SHAPE_ROOM));
		
		s.addObject(new CleanupDoor("door0", x2, x2, 1, 1, CleanupWorld.LOCKABLE_STATES[0]));
		s.addObject(new CleanupDoor("door1", x1, x1, 5, 5, CleanupWorld.LOCKABLE_STATES[0]));
		s.addObject(new CleanupDoor("door2", 2, 2, y2, y2, CleanupWorld.LOCKABLE_STATES[0]));
		s.addObject(new CleanupDoor("door3", 10, 10, y2, y2, CleanupWorld.LOCKABLE_STATES[0]));
		

		int i = 0;
		while (i < numBlocks) {
			String id = "block"+i;
			CleanupRoom room = (CleanupRoom) s.objectsOfClass(CleanupWorld.CLASS_ROOM).get(rng.nextInt(numRooms));
			int rLeft = ((Integer) room.get(CleanupWorld.ATT_LEFT))+1;
			int rRight = ((Integer) room.get(CleanupWorld.ATT_RIGHT))-1;
			int rTop = ((Integer) room.get(CleanupWorld.ATT_TOP))-1;
			int rBottom = ((Integer) room.get(CleanupWorld.ATT_BOTTOM))+1;
			int bX = rng.nextInt(rRight - rLeft) + rLeft;
			int bY = rng.nextInt(rTop - rBottom) + rBottom;
			if (s.isOpen(room, bX, bY)) {
				String shape = CleanupWorld.SHAPES[rng.nextInt(CleanupWorld.SHAPES.length)];
				String color = CleanupWorld.COLORS[rng.nextInt(CleanupWorld.COLORS.length)];
				System.out.println("block"+i+": "+ shape + " " + color + " (" + bX + ", " + bY + ") in the " + room.get(CleanupWorld.ATT_COLOR) + " room");
				s.addObject(new CleanupBlock(id, bX, bY, shape, color));
				i = i + 1;
			}
		}
//		CleanupWorld.setBlock(s, 0, 5, 4, "chair", "blue");
//		CleanupWorld.setBlock(s, 1, 6, 10, "basket", "red");
//		CleanupWorld.setBlock(s, 2, 2, 10, "bag", "magenta");
		
		return s;
		

	}
	
	public static boolean regionContainsPoint(ObjectInstance o, int x, int y, boolean countBoundary){
		int top = (Integer) o.get(CleanupWorld.ATT_TOP);
		int left = (Integer) o.get(CleanupWorld.ATT_LEFT);
		int bottom = (Integer) o.get(CleanupWorld.ATT_BOTTOM);
		int right = (Integer) o.get(CleanupWorld.ATT_RIGHT);

		if(countBoundary){
			if(y >= bottom && y <= top && x >= left && x <= right){
				return true;
			}
		}
		else{
			if(y > bottom && y < top && x > left && x < right){
				return true;
			}
		}

		return false;
	}

	
	public static CleanupGoalDescription[] getRandomGoalDescription(Random rng, CleanupWorldState s, int numGoals, PropositionalFunction pf) {
		CleanupGoalDescription[] goals = new CleanupGoalDescription[numGoals];
		if (pf.getName().equals(CleanupWorld.PF_BLOCK_IN_ROOM)) {
			List<ObjectInstance> blocks = s.objectsOfClass(CleanupWorld.CLASS_BLOCK);
			List<ObjectInstance> rooms = s.objectsOfClass(CleanupWorld.CLASS_ROOM);
			List<Integer> blockIdxs = new ArrayList<Integer>();
			for (int i = 0; i < blocks.size(); i++) { blockIdxs.add(i); }
			for (int i = 0; i < numGoals; i++) {
				ObjectInstance block = blocks.get(blockIdxs.get(i));
				ObjectInstance room = rooms.get(rng.nextInt(rooms.size()));
				while (regionContainsPoint(room, (Integer) block.get(CleanupWorld.ATT_X), (Integer) block.get(CleanupWorld.ATT_Y), true)){
					// disallow the room the block is already in
					room = rooms.get(rng.nextInt(rooms.size()));
				}
				goals[i] = new CleanupGoalDescription(new String[]{block.name(), room.name()}, pf);
				System.out.println(goals[i] + ": "
						+ block.get(CleanupWorld.ATT_COLOR) + " "
						+ block.get(CleanupWorld.ATT_SHAPE) + " to "
						+ room.get(CleanupWorld.ATT_COLOR) + " room");
			}
		} else if (pf.getName().equals(CleanupWorld.PF_AGENT_IN_DOOR)) {
			List<ObjectInstance> agents = s.objectsOfClass(CleanupWorld.CLASS_AGENT);
			List<ObjectInstance> doors = s.objectsOfClass(CleanupWorld.CLASS_DOOR);
			for (int i = 0; i < numGoals; i++) {
				ObjectInstance door = doors.get(rng.nextInt(doors.size()));
				ObjectInstance agent = agents.get(0);
				goals[i] = new CleanupGoalDescription(new String[]{agent.name(), door.name()}, pf);
				System.out.println(goals[i] + ": agent (x:"
						+ agent.get(CleanupWorld.ATT_X) + ", y:"
						+ agent.get(CleanupWorld.ATT_Y) + ") to door (x:"
						+ door.get(CleanupWorld.ATT_TOP) + ", y:"
						+ door.get(CleanupWorld.ATT_LEFT) + ")");
			}
		} else if (pf.getName().equals(CleanupWorld.PF_BLOCK_IN_DOOR)) {
			List<ObjectInstance> blocks = s.objectsOfClass(CleanupWorld.CLASS_BLOCK);
			List<ObjectInstance> doors = s.objectsOfClass(CleanupWorld.CLASS_DOOR);
			for (int i = 0; i < numGoals; i++) {
				ObjectInstance door = doors.get(rng.nextInt(doors.size()));
				ObjectInstance block = blocks.get(0);
				goals[i] = new CleanupGoalDescription(new String[]{block.name(), door.name()}, pf);
				System.out.println(goals[i] + ": block (x:"
						+ block.get(CleanupWorld.ATT_X) + ", y:"
						+ block.get(CleanupWorld.ATT_Y) + ") to door (x:"
						+ door.get(CleanupWorld.ATT_TOP) + ", y:"
						+ door.get(CleanupWorld.ATT_LEFT) + ")");
			}
		} else {
			throw new RuntimeException("Randomization of goal not implemented for given propositional function.");
		}
		return goals;
	}


	
}
