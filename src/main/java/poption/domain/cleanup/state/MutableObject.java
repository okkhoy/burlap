package poption.domain.cleanup.state;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import burlap.mdp.core.state.MutableState;


public abstract class MutableObject implements MutableObjectInstance {

	public Map<String, Object> values = new HashMap<String, Object>();
	
	@Override
	public Object get(Object variableKey) {
		return this.values.get(variableKey);
	}

	@Override
	public MutableState set(Object variableKey, Object value) {
		this.values.put(variableKey.toString(), value);
		return this;
	}
	
	@Override
	public String toString() {
		StringBuilder buf = new StringBuilder();
		buf.append(className()).append(":").append(name());
		buf.append(" {\n");
		List<Object> keys = this.variableKeys();
		for(Object key : keys){
			Object value = this.get(key);
			buf.append("\t").append(key.toString()).append(": {");
			if (value == null) {
				buf.append("unset");
			} else {
				buf.append(value.toString());
			}
			buf.append("}\n");
		}
		buf.append("}");
		return buf.toString();
	}
	
//	@Override
//	public boolean equals(Object other) {
//		if (this == other) return true;
//		if (other == null) return false;
//		if (getClass() != other.getClass()) return false;
//		MutableObject obj = (MutableObject) other;
//		if (obj.values == null && this.values == null) {
//			return true;
//		} else if ((obj.values == null && this.values != null)
//				|| (obj.values != null && this.values == null)) {
//			return false;
//		} 
//		// check key equality
//		if (!this.values.keySet().equals(obj.values.keySet())) {
//			return false;
//		}
//		// check equality of values
//		for (Object key : values.keySet()) {
//			Object thisValue = this.values.get(key);
//			Object otherValue = obj.values.get(key);
//			if (thisValue == null && otherValue == null) {
//				continue;
//			} else if ((otherValue == null && thisValue != null)
//					|| (otherValue != null && thisValue == null)) {
//				return false;
//			} else if (!thisValue.equals(otherValue)) {
//				return false;
//			}
//		}
//		return true;
//	}
	
}
