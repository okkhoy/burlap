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
public class CleanupAgent extends MutableObject {

	public String name;
	
	public CleanupAgent() {
		
	}
	
	private final static List<Object> keys = Arrays.<Object>asList(
			CleanupWorld.ATT_X,
			CleanupWorld.ATT_Y,
			CleanupWorld.ATT_DIR,
			CleanupWorld.ATT_SHAPE
	);

	public CleanupAgent(int x, int y, String direction) {
		this(CleanupWorld.CLASS_AGENT, x, y, direction, CleanupWorld.SHAPE_AGENT);
	}
	
	public CleanupAgent(String name, int x, int y, String direction, String shape) {
		this(name, (Object)x, (Object)y, (Object)direction, (Object)shape);
	}
	
	public CleanupAgent(String name, Object x, Object y, Object direction, Object shape) {
		this.set(CleanupWorld.ATT_X, x);
		this.set(CleanupWorld.ATT_Y, y);
		this.set(CleanupWorld.ATT_DIR, direction);
		this.set(CleanupWorld.ATT_SHAPE, shape);
		this.name = name;
	}

	@Override
	public String className() {
		return CleanupWorld.CLASS_AGENT;
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
	public CleanupAgent copy() {
		return new CleanupAgent(name,
				get(CleanupWorld.ATT_X),
				get(CleanupWorld.ATT_Y),
				get(CleanupWorld.ATT_DIR),
				get(CleanupWorld.ATT_SHAPE)
		);
	}

	@Override
	public ObjectInstance copyWithName(String objectName) {
		return new CleanupAgent(objectName, get(CleanupWorld.ATT_X), get(CleanupWorld.ATT_Y), get(CleanupWorld.ATT_DIR), get(CleanupWorld.ATT_SHAPE));
	}

	@Override
	public boolean equals(Object other) {
		if (this == other) return true;
		if (other == null) return false;
		if (getClass() != other.getClass()) return false;
		CleanupAgent o = (CleanupAgent) other;
		for (Object key : keys) {
			if (!get(key).equals(o.get(key))) {
				return false;
			}
		}
		return name.equals(o.name);
	}
	
}
