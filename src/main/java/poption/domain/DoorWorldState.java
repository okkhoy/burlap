package poption.domain;

import static burlap.domain.singleagent.gridworld.GridWorldDomain.VAR_TYPE;
import static burlap.domain.singleagent.gridworld.GridWorldDomain.VAR_X;
import static burlap.domain.singleagent.gridworld.GridWorldDomain.VAR_Y;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import burlap.domain.singleagent.gridworld.state.GridAgent;
import burlap.domain.singleagent.gridworld.state.GridLocation;
import burlap.domain.singleagent.gridworld.state.GridWorldState;
import burlap.mdp.core.oo.state.MutableOOState;
import burlap.mdp.core.oo.state.OOStateUtilities;
import burlap.mdp.core.oo.state.OOVariableKey;
import burlap.mdp.core.oo.state.ObjectInstance;
import burlap.mdp.core.state.MutableState;
import burlap.mdp.core.state.State;
import burlap.mdp.core.state.StateUtilities;
import burlap.mdp.core.state.annotations.ShallowCopyState;

@ShallowCopyState
public class DoorWorldState extends GridWorldState {

	protected List<GridDoor> doors = new ArrayList<GridDoor>();
	
	public DoorWorldState(GridAgent agent, GridLocation loc) {
		super(agent, loc);
	}

	public DoorWorldState(GridAgent agent, GridLocation loc0, GridDoor door1, GridDoor door2,
			GridDoor door3, GridDoor door4) {
		super(agent, loc0);
		doors.add(door1);
		doors.add(door2);
		doors.add(door3);
		doors.add(door4);
	}

	public DoorWorldState(GridAgent agent, GridLocation loc0, GridDoor door1) {
		super(agent, loc0);
		doors.add(door1);
	}
	
	
	public DoorWorldState(GridAgent agent, GridLocation loc, List<GridDoor> doors) {
		super(agent, loc);
		this.doors = doors;
	}

	public DoorWorldState(GridAgent agent, List<GridLocation> locations, List<GridDoor> doors) {
		super(agent, locations);
		this.doors = doors;
	}

	public DoorWorldState() {
		super();
	}

	@Override
	public List<ObjectInstance> objectsOfClass(String oclass) {
		if(oclass.equals(DoorWorldDomain.CLASS_DOOR)){
			return new ArrayList<ObjectInstance>(doors);
		}
		return super.objectsOfClass(oclass);
	}

	@Override
	public ObjectInstance object(String oname) {
		if(oname.equals(agent.name())){
			return agent;
		}
		int ind = this.locationInd(oname);
		if(ind != -1){
			return locations.get(ind);
		}
		ind = this.doorInd(oname);
		if(ind != -1) {
			return doors.get(ind);
		}
		throw new RuntimeException("Unknown object of name " + oname);
	}

	@Override
	public List<ObjectInstance> objects() {
		List<ObjectInstance> obs = super.objects();
		obs.addAll(doors);
		return obs;
	}

	protected int doorInd(String oname){
		return this.indexOfObjectNameInList(oname, doors);
	}
	
	public List<GridDoor> touchDoors(){
		this.doors = new ArrayList<GridDoor>(doors);
		return doors;
	}

	public List<GridDoor> deepTouchDoors(){
		List<GridDoor> nDoors = new ArrayList<GridDoor>(doors.size());
		for(GridDoor door : doors){
			nDoors.add(door.copy());
		}
		doors = nDoors;
		return doors;
	}

	public GridDoor touchDoor(int ind){
		GridDoor n = doors.get(ind).copy();
		touchDoors().remove(ind);
		doors.add(ind, n);
		return n;
	}


	@Override
	public MutableOOState addObject(ObjectInstance o) {
		if(!(o instanceof GridLocation) && !(o instanceof GridDoor)){
			throw new RuntimeException("Can only add GridLocation objects to a GridWorldState.");
		} else if(o instanceof GridLocation) {
			GridLocation loc = (GridLocation)o;
			//copy on write
			touchLocations().add(loc);
		} else if(o instanceof GridDoor) {
			GridDoor door = (GridDoor)o;
			//copy on write
			touchDoors().add(door);
		}
		return this;
	}
	
	@Override
	public boolean equals(Object other) {
		if (this == other) return true;
		if (other == null) return false;
		if (getClass() != other.getClass()) return false;
		DoorWorldState o = (DoorWorldState) other;
		if (o.agent != null && this.agent != null) {
			if (!o.agent.get(VAR_X).equals(this.agent.get(VAR_X))
			 || !o.agent.get(VAR_Y).equals(this.agent.get(VAR_Y))) {
				return false;
			}
		}
		if (!o.locations.equals(this.locations)) {
			for (int i = 0; i < locations.size(); i++) {
				if (!locations.get(i).get(VAR_X).equals(o.locations.get(i).get(VAR_X))
				 || !locations.get(i).get(VAR_Y).equals(o.locations.get(i).get(VAR_Y))) {
					return false;
				}
			}
		}
		if (!o.doors.equals(this.doors)) return false;
		return true;
	}

	@Override
	public State copy() {
		return new DoorWorldState(agent, locations, doors);
	}
}
