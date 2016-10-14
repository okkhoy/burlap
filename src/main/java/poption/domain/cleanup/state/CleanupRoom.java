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
public class CleanupRoom extends MutableObject implements MutableObjectInstance {

	public String name;
	
	public CleanupRoom() {
		
	}
	
	private final static List<Object> keys = Arrays.<Object>asList(
			CleanupWorld.ATT_LEFT,
			CleanupWorld.ATT_RIGHT,
			CleanupWorld.ATT_BOTTOM,
			CleanupWorld.ATT_TOP,
			CleanupWorld.ATT_COLOR,
			CleanupWorld.ATT_SHAPE
	);

	public CleanupRoom(String name, int left, int right, int bottom, int top, String color) {
		this(name, (Object)left, (Object)right, (Object)bottom, (Object)top, (Object)color, CleanupWorld.SHAPE_ROOM);
	}
	
	public CleanupRoom(String name, int left, int right, int bottom, int top, String color, String shape) {
		this(name, (Object)left, (Object)right, (Object)bottom, (Object)top, (Object)color, (Object)shape);
	}
	
	public CleanupRoom(String name, Object left, Object right, Object bottom, Object top, Object color, Object shape) {
		this.set(CleanupWorld.ATT_LEFT, left);
		this.set(CleanupWorld.ATT_RIGHT, right);
		this.set(CleanupWorld.ATT_BOTTOM, bottom);
		this.set(CleanupWorld.ATT_TOP, top);
		this.set(CleanupWorld.ATT_COLOR, color);
		this.set(CleanupWorld.ATT_SHAPE, shape);
		this.name = name;
	}


	@Override
	public String className() {
		return CleanupWorld.CLASS_ROOM;
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
	public CleanupRoom copy() {
		return (CleanupRoom) copyWithName(name);
	}

	@Override
	public ObjectInstance copyWithName(String objectName) {
		return new CleanupRoom(objectName,
				get(CleanupWorld.ATT_LEFT),
				get(CleanupWorld.ATT_RIGHT), 
				get(CleanupWorld.ATT_BOTTOM),
				get(CleanupWorld.ATT_TOP),
				get(CleanupWorld.ATT_COLOR),
				get(CleanupWorld.ATT_SHAPE)
		);
	}

	@Override
	public boolean equals(Object other) {
		if (this == other) return true;
		if (other == null) return false;
		if (getClass() != other.getClass()) return false;
		CleanupRoom o = (CleanupRoom) other;
		for (Object key : keys) {
			if (!get(key).equals(o.get(key))) {
				return false;
			}
		}
		return name.equals(o.name);
	}
	
}
