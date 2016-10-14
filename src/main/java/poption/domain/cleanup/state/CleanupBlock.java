package poption.domain.cleanup.state;

import java.util.Arrays;
import java.util.List;

import burlap.mdp.core.oo.state.OOStateUtilities;
import burlap.mdp.core.oo.state.ObjectInstance;
import burlap.mdp.core.oo.state.generic.GenericOOState;
import burlap.mdp.core.state.MutableState;
import burlap.mdp.core.state.State;
import burlap.mdp.core.state.StateUtilities;
import burlap.mdp.core.state.UnknownKeyException;
import burlap.mdp.core.state.annotations.DeepCopyState;
import poption.domain.cleanup.CleanupWorld;

@DeepCopyState
public class CleanupBlock extends MutableObject implements MutableObjectInstance {

	public String name;
	
	public CleanupBlock() {
		
	}
	
	private final static List<Object> keys = Arrays.<Object>asList(
			CleanupWorld.ATT_X,
			CleanupWorld.ATT_Y,
			CleanupWorld.ATT_SHAPE,
			CleanupWorld.ATT_COLOR
	);


	public CleanupBlock(int x, int y) {
		this(CleanupWorld.CLASS_BLOCK, (Object)x, (Object)y, (Object)"chair", (Object)"yellow");
	}
	
	public CleanupBlock(String name, Object x, Object y, Object shape, Object color) {
		this.set(CleanupWorld.ATT_X, x);
		this.set(CleanupWorld.ATT_Y, y);
		this.set(CleanupWorld.ATT_SHAPE, shape);
		this.set(CleanupWorld.ATT_COLOR, color);
		this.name = name;
	}

	@Override
	public String className() {
		return CleanupWorld.CLASS_BLOCK;
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
	public CleanupBlock copy() {
		return new CleanupBlock(name, get(CleanupWorld.ATT_X), get(CleanupWorld.ATT_Y), get(CleanupWorld.ATT_SHAPE), get(CleanupWorld.ATT_COLOR));
	}
	
	@Override
	public ObjectInstance copyWithName(String objectName) {
		return new CleanupBlock(objectName, get(CleanupWorld.ATT_X), get(CleanupWorld.ATT_Y), get(CleanupWorld.ATT_SHAPE), get(CleanupWorld.ATT_COLOR));
	}
		
	@Override
	public boolean equals(Object other) {
		if (this == other) return true;
		if (other == null) return false;
		if (getClass() != other.getClass()) return false;
		CleanupBlock block = (CleanupBlock) other;
		for (Object key : keys) {
			if (!get(key).equals(block.get(key))) {
				return false;
			}
		}
		return name.equals(block.name);
	}
	
}
