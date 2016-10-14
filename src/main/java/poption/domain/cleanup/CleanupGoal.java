package poption.domain.cleanup;

import burlap.mdp.auxiliary.stateconditiontest.StateConditionTest;
import burlap.mdp.core.oo.propositional.GroundedProp;
import burlap.mdp.core.state.State;
import poption.domain.cleanup.state.CleanupWorldState;

public class CleanupGoal implements StateConditionTest {

	public CleanupGoalDescription[] goals = {};

	public CleanupGoal(CleanupGoalDescription[] goals) {
		this.goals = goals;
	}

	public CleanupGoal() {

	}
	
	public void setGoals(CleanupGoalDescription[] goals) {
		this.goals = goals;
	}

	@Override
	public boolean satisfies(State s) {
		for(int i = 0; i < goals.length; i++){
			GroundedProp gp = new GroundedProp(goals[i].pf, goals[i].objects);
			if(!gp.isTrue((CleanupWorldState)s)){
				return false;
			}
		}
		return true;
	}
	
	public String toString() {
		String out = "";
		for (CleanupGoalDescription desc : goals) {
			out += desc.toString();
		}
		return out;
	}
	
}