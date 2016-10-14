package poption.domain.cleanup.state;

import java.util.Arrays;
import java.util.List;

import burlap.mdp.core.oo.state.ObjectInstance;
import burlap.mdp.core.oo.state.generic.GenericOOState;
import burlap.mdp.core.state.MutableState;
import burlap.mdp.core.state.State;
import burlap.mdp.core.state.StateUtilities;
import burlap.mdp.core.state.UnknownKeyException;
import burlap.mdp.core.state.annotations.DeepCopyState;
import poption.domain.cleanup.CleanupWorld;

@DeepCopyState
public class CleanupDoor extends MutableObject implements MutableObjectInstance {

	public String name;
	
	public CleanupDoor() {
		
	}
	
	private final static List<Object> keys = Arrays.<Object>asList(
			CleanupWorld.ATT_X,
			CleanupWorld.ATT_Y,
			CleanupWorld.ATT_LEFT,
			CleanupWorld.ATT_RIGHT,
			CleanupWorld.ATT_BOTTOM,
			CleanupWorld.ATT_TOP,
			CleanupWorld.ATT_LOCKED,
			CleanupWorld.ATT_SHAPE
	);

	public CleanupDoor(int x, int y) {
		this(CleanupWorld.CLASS_DOOR, x, x, y, y, CleanupWorld.LOCKABLE_STATES[0], CleanupWorld.SHAPE_DOOR);
	}
	
	public CleanupDoor(String name, int left, int right, int bottom, int top, String locked) {
		this(name, (Object)left, (Object)right, (Object)bottom, (Object)top, (Object)locked, CleanupWorld.SHAPE_DOOR);
	}
	
	public CleanupDoor(String name, int left, int right, int bottom, int top, String locked, String shape) {
		this(name, (Object)left, (Object)right, (Object)bottom, (Object)top, (Object)locked, (Object) shape);
	}
	
	public CleanupDoor(String name, Object left, Object right, Object bottom, Object top, Object locked, Object shape) {
		this.set(CleanupWorld.ATT_LEFT, left);
		this.set(CleanupWorld.ATT_X, left);
		this.set(CleanupWorld.ATT_RIGHT, right);
		this.set(CleanupWorld.ATT_BOTTOM, bottom);
		this.set(CleanupWorld.ATT_Y, bottom);
		this.set(CleanupWorld.ATT_TOP, top);
		this.set(CleanupWorld.ATT_LOCKED, locked);
		this.set(CleanupWorld.ATT_SHAPE, shape);
		this.name = name;
	}

	@Override
	public String className() {
		return CleanupWorld.CLASS_DOOR;
	}

	@Override
	public String name() {
		return name;
	}

	@Override
	public List<Object> variableKeys() {
		return keys;
	}

	@Override
	public CleanupDoor copy() {
		return (CleanupDoor) copyWithName(name);
	}

	@Override
	public ObjectInstance copyWithName(String objectName) {
		return new CleanupDoor(objectName,
				get(CleanupWorld.ATT_LEFT),
				get(CleanupWorld.ATT_RIGHT),
				get(CleanupWorld.ATT_BOTTOM),
				get(CleanupWorld.ATT_TOP),
				get(CleanupWorld.ATT_LOCKED),
				get(CleanupWorld.ATT_SHAPE)
		);
	}

	@Override
	public boolean equals(Object other) {
		if (this == other) return true;
		if (other == null) return false;
		if (getClass() != other.getClass()) return false;
		CleanupDoor o = (CleanupDoor) other;
		for (Object key : keys) {
			if (!get(key).equals(o.get(key))) {
				return false;
			}
		}
		return name.equals(o.name);
	}
}
