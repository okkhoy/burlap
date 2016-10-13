package poptions;

import burlap.domain.singleagent.gridworld.state.GridLocation;
import burlap.mdp.core.state.annotations.DeepCopyState;

@DeepCopyState
public class GridDoor extends GridLocation {
	
	public GridDoor(int x, int y, int type, String name, boolean useGlobalID) {
		super(x, y, type, name, useGlobalID);
	}

	public GridDoor(int x, int y, String name, boolean useGlobalID) {
		super(x, y, name, useGlobalID);
	}

	@Override
	public String className() {
		return DoorWorldDomain.CLASS_DOOR;
	}

	@Override
	public GridDoor copy() {
		return new GridDoor(x, y, type, name, false);
	}
	
}