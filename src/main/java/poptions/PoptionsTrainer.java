package poptions;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.ArrayUtils;
import org.apache.commons.lang3.StringUtils;

import burlap.behavior.functionapproximation.dense.ConcatenatedObjectFeatures;
import burlap.behavior.functionapproximation.dense.NumericVariableFeatures;
import burlap.behavior.functionapproximation.dense.PFFeatures;
import burlap.behavior.policy.Policy;
import burlap.behavior.singleagent.Episode;
import burlap.behavior.singleagent.auxiliary.StateReachability;
import burlap.behavior.singleagent.options.Option;
import burlap.behavior.singleagent.planning.Planner;
import burlap.mdp.auxiliary.stateconditiontest.StateConditionTest;
import burlap.mdp.core.oo.propositional.PropositionalFunction;
import burlap.mdp.core.oo.state.OOState;
import burlap.mdp.core.state.State;
import burlap.mdp.singleagent.SADomain;
import burlap.mdp.singleagent.oo.OOSADomain;
import burlap.statehashing.HashableStateFactory;
import burlap.statehashing.masked.MaskedHashableStateFactory;
import burlap.statehashing.simple.SimpleHashableStateFactory;

/**
 * This class provides the basis for a class that trains parameterized options.
 * 
 * 
 * 
 * @author John Winder
 *
 */
public abstract class PoptionsTrainer {
	
	protected String name = "poptions";
	protected OOSADomain domain;
	protected OOState initialState;
	protected StateConditionTest goalCondition;
	protected MaskedHashableStateFactory mhsf;
	protected Episode result;
	protected int maxUniqueStates = 10000;
	
	
	public List<State> states;
	
	protected int propFuncIndex = -1;
	
	public String getName() {
		return name;
	}
	
	public abstract void initialize(long seed, boolean testing);

	public abstract void setStateHashingMasks();

	public abstract boolean train(String outputPath);
	
	public abstract String getFeatureVectorHeader();
	
	public abstract String getVisualizeFilePath();

	public abstract List<double[]> getTrainingFeatureVectors();
	
	public abstract List<double[]> getTestFeatureVectors();
	
	public abstract Policy createOptionPolicy(StateConditionTest goal, HashableStateFactory hf);
	
	public abstract Planner getPlanner(StateConditionTest goal, HashableStateFactory hf);
	
	public abstract void evaluate(List<Option> options);
	
	public abstract void visualize(String outputPath);
	
	public int getFirstPropositionalFunctionIndex() {
		return propFuncIndex;
	}
	
	protected String[] objectClassNamesToArray(List<String> objectClassNames) {
		if (objectClassNames == null) {
			List<Class<?>> objectClasses = domain.stateClasses();
			objectClassNames = new ArrayList<String>(objectClasses.size());
			for (int i = 0; i < objectClasses.size(); i++) {
				objectClassNames.add(objectClasses.get(i).getName());
			}
		}
		return objectClassNames.toArray(new String[objectClassNames.size()]);
	}
	
	public String getFeatureVectorNames(List<String> objectClassNames, List<PropositionalFunction> propFunctions) {
		String[] classNames = objectClassNamesToArray(objectClassNames);
		if (propFunctions == null) {
			propFunctions = domain.propFunctions();
		}
		
		ConcatenatedObjectFeatures objFVGen = new ConcatenatedObjectFeatures();
		for (String name : classNames) {
			objFVGen.addObjectVectorization(name, new NumericVariableFeatures());
		}
		PFFeatures pfFVGen = new PFFeatures(propFunctions);
		String objectAttributeNames = StringUtils.join(objFVGen.featureNames(initialState), ",");
		String propFunctionNames = StringUtils.join(pfFVGen.featureNames(initialState), ",");
		String goal = "isGoal";
		return objectAttributeNames + "," + propFunctionNames + "," + goal;
	}

	public List<double[]> getTrainingFeatureVectors(List<String> objectClassNames, List<PropositionalFunction> propFunctions) {
		states = result.stateSequence;
		return getFeatureVectors(objectClassNames, propFunctions, states);
	}
	
	public List<double[]> getTestFeatureVectors(List<String> objectClassNames, List<PropositionalFunction> propFunctions) {
		SimpleHashableStateFactory sh = new SimpleHashableStateFactory(false);
		System.out.println("Using bounded state reachability, max " + maxUniqueStates);
		states = StateReachability.getReachableStates(initialState, (SADomain)domain, sh, maxUniqueStates);
		return getFeatureVectors(objectClassNames, propFunctions, states);
	};
	
	public List<double[]> getFeatureVectors(List<String> objectClassNames, List<PropositionalFunction> propFunctions, List<State> states) {
		String[] classNames = objectClassNamesToArray(objectClassNames);
		if (propFunctions == null) {
			propFunctions = domain.propFunctions();
		}

		ConcatenatedObjectFeatures cofvGen = new ConcatenatedObjectFeatures();
		for (String name : classNames) {
			cofvGen.addObjectVectorization(name, new NumericVariableFeatures());
		}
		PFFeatures pffvGen = new PFFeatures(propFunctions);
		List<double[]> fvs = new ArrayList<double[]>();
		for (int i = 0; i < states.size(); i++) {
			State s = states.get(i);
			double[] combined = {};
			double[] cofv = cofvGen.features(s);
			propFuncIndex = cofv.length+1;
			double[] pffv = pffvGen.features(s);
			combined = ArrayUtils.addAll(combined, cofv);
			combined = ArrayUtils.addAll(combined, pffv);
			double[] goal = {goalCondition.satisfies(s) ? 1.0 : 0.0};
			combined = ArrayUtils.addAll(combined, goal);
			fvs.add(combined);
		}
		return fvs;
	}

	public MaskedHashableStateFactory getMaskedHashableStateFactory() {
		return mhsf;
	}
	
}
