package poption;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import javax.swing.JFrame;

import burlap.behavior.policy.GreedyQPolicy;
import burlap.behavior.policy.Policy;
import burlap.behavior.policy.PolicyUtils;
import burlap.behavior.singleagent.Episode;
import burlap.behavior.singleagent.auxiliary.EpisodeSequenceVisualizer;
import burlap.behavior.singleagent.auxiliary.StateReachability;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.ValueFunctionVisualizerGUI;
import burlap.behavior.singleagent.learning.tdmethods.QLearning;
import burlap.behavior.singleagent.options.Option;
import burlap.behavior.singleagent.options.OptionType;
import burlap.behavior.singleagent.options.SubgoalOption;
import burlap.behavior.singleagent.planning.Planner;
import burlap.behavior.singleagent.planning.deterministic.DDPlannerPolicy;
import burlap.behavior.singleagent.planning.deterministic.DeterministicPlanner;
import burlap.behavior.singleagent.planning.stochastic.valueiteration.ValueIteration;
import burlap.behavior.valuefunction.ValueFunction;
import burlap.debugtools.RandomFactory;
import burlap.domain.singleagent.gridworld.GridWorldDomain;
import burlap.domain.singleagent.gridworld.GridWorldVisualizer;
import burlap.mdp.core.TerminalFunction;
import burlap.mdp.core.action.Action;
import burlap.mdp.core.action.ActionType;
import burlap.mdp.core.oo.propositional.PropositionalFunction;
import burlap.mdp.core.state.State;
import burlap.mdp.singleagent.common.GoalBasedRF;
import burlap.mdp.singleagent.common.UniformCostRF;
import burlap.mdp.singleagent.common.VisualActionObserver;
import burlap.mdp.singleagent.environment.Environment;
import burlap.mdp.singleagent.environment.SimulatedEnvironment;
import burlap.mdp.singleagent.environment.extensions.EnvironmentServer;
import burlap.mdp.singleagent.model.RewardFunction;
import burlap.mdp.singleagent.oo.OOSADomain;
import burlap.shell.visual.VisualExplorer;
import burlap.statehashing.HashableStateFactory;
import burlap.mdp.auxiliary.common.NullTermination;
import burlap.mdp.auxiliary.common.SinglePFTF;
import burlap.mdp.auxiliary.stateconditiontest.StateConditionTest;
import burlap.mdp.auxiliary.stateconditiontest.TFGoalCondition;
import burlap.statehashing.masked.MaskedHashableStateFactory;
import burlap.statehashing.simple.SimpleHashableStateFactory;
import burlap.visualizer.Visualizer;
import poption.domain.doorworld.DoorWorldDomain;
import weka.classifiers.Classifier;
import weka.classifiers.trees.J48;

public class DoorWorldTrainer extends PoptionsTrainer {
	
	private Random rng;
	private DoorWorldDomain domainGenerator;
	private Environment env;
	private RewardFunction rf;
	private TerminalFunction tf;
	private HashableStateFactory hashingFactory;
	private int randomFactoryMapIndex = 4246;

	private ValueIteration planner;
	
	private int minWidth;
	private int maxWidth;
	private int minHeight;
	private int maxHeight;
	private List<String> classesInFeatureVector;
	private List<PropositionalFunction> propFunctions;
	
	public DoorWorldTrainer(int minWidth, int maxWidth, int minHeight, int maxHeight) {
		super();
		this.name = "door";
		this.minWidth = minWidth;
		this.maxWidth = maxWidth;
		this.minHeight = minHeight;
		this.maxHeight = maxHeight;
		this.classesInFeatureVector = Arrays.asList(DoorWorldDomain.CLASS_AGENT);
		propFunctions = new ArrayList<PropositionalFunction>();
		mhsf = new MaskedHashableStateFactory(false);
	}
	
	public DoorWorldTrainer(int minWidth, int maxWidth, int minHeight, int maxHeight, long seed) {
		this(minWidth, maxWidth, minHeight, maxHeight);
	}
	
	@Override
	public void initialize(long seed, boolean testing) {
		
		System.out.println("Using seed: " + seed);
		
		RandomFactory.seedMapped(randomFactoryMapIndex, seed);
		rng = RandomFactory.getMapped(randomFactoryMapIndex);

		int width = maxWidth-1;
		int height = maxHeight-1;
		if (!testing) {
			width = minWidth + rng.nextInt(maxWidth - minWidth);
			height = minHeight + rng.nextInt(maxHeight - minHeight);
		}
		
		//debug!
		if (testing) {
			width = 11;//25;
			height = 11;//25;
		}

		domainGenerator = new DoorWorldDomain(width, height);
		if(testing) {
			domainGenerator.setMapFourDoorsRandom(rng);
		} else {
			domainGenerator.setMapOneDoorRandom(rng);
		}
		domainGenerator.setProbSucceedTransitionDynamics(0.95);
		rf = new UniformCostRF();
		PropositionalFunction atLocationPF = domainGenerator.generateDomain().propFunction(GridWorldDomain.PF_AT_LOCATION);
		tf = new SinglePFTF(atLocationPF);
		domainGenerator.setRf(rf);
		domainGenerator.setTf(tf);
		domain = domainGenerator.generateDomain();
		goalCondition = new TFGoalCondition(tf);

		int ax = 0;
		int ay = 0;
		int lx = domainGenerator.getWidth()-1;
		int ly = domainGenerator.getHeight()-1;
		if (testing) {
			initialState = domainGenerator.getOneAgentOneLocationFourDoorsState(ax, ay, lx, ly);
			System.out.println("testing doorworld");
		} else {
			lx = domainGenerator.door1X;
			ly = domainGenerator.door1Y;
			initialState = domainGenerator.getOneAgentOneLocationOneDoorState(ax, ay, lx, ly);
			System.out.println("training doorworld");
		}
		
		hashingFactory = new SimpleHashableStateFactory(true);
		env = new SimulatedEnvironment(domain, initialState);
		
		propFunctions = new ArrayList<PropositionalFunction>();
		propFunctions.add(domain.propFunction(DoorWorldDomain.PF_WALL_NORTH));
		propFunctions.add(domain.propFunction(DoorWorldDomain.PF_WALL_SOUTH));
		propFunctions.add(domain.propFunction(DoorWorldDomain.PF_WALL_EAST));
		propFunctions.add(domain.propFunction(DoorWorldDomain.PF_WALL_WEST));
		propFunctions.add(domain.propFunction(DoorWorldDomain.PF_AGENT_IN_CORNER));
		propFunctions.add(domain.propFunction(DoorWorldDomain.PF_AGENT_IN_DOOR));
	}
	
	@Override
	public List<double[]> getTrainingFeatureVectors(List<String> objectClassNames, List<PropositionalFunction> propFunctions) {
		states = StateReachability.getReachableStates(this.initialState, this.domain, this.hashingFactory);
		return getFeatureVectors(objectClassNames, propFunctions, states);
	}


	public void visualize(String outputPath) {
		Visualizer v = GridWorldVisualizer.getVisualizer(domainGenerator.getMap());
		EpisodeSequenceVisualizer esv = new EpisodeSequenceVisualizer(v, domain, outputPath);
		esv.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
	}
	
	@Override
	public boolean train(String outputPath) {
		
		double gamma = 0.99;
		double maxDelta = 0.001;
		int maxIterations = 1000;
		Planner planner = new ValueIteration(domain, gamma, hashingFactory, maxDelta, maxIterations);
		long startTime = System.nanoTime();
		Policy p = planner.planFromState(initialState);
		result = PolicyUtils.rollout(p, initialState, domain.getModel());
		long endTime = System.nanoTime();
		long duration = (endTime - startTime);
		System.out.println("total time: " + duration);
		
		result.write(outputPath + "dw_vi");

		System.out.println("total actions: " + result.actionSequence.size());

		return true;
	}

	@Override
	public String getFeatureVectorHeader() {
		return getFeatureVectorNames(classesInFeatureVector, propFunctions);
	}

	@Override
	public List<double[]> getTrainingFeatureVectors() {
		return getTrainingFeatureVectors(classesInFeatureVector, propFunctions);
	}

	@Override
	public List<double[]> getTestFeatureVectors() {
		return getTestFeatureVectors(classesInFeatureVector, propFunctions);
	}
	
	@Override
	public List<State> getReachableStates() {
		if (planner == null) {
			System.out.println("init vi");
			planner = initializeVI(domain, hashingFactory);
		}
		planner.performReachabilityFrom(initialState);
		return planner.getAllStates();
	}

	@Override
	public Policy createOptionPolicy(StateConditionTest goal, HashableStateFactory hf) {
		domainGenerator.setRf(new GoalBasedRF(goal));
		domainGenerator.setTf(tf);
		OOSADomain optionDomain = domainGenerator.generateDomain();
		ValueIteration vi = initializeVI(optionDomain, hf);
		Policy optionPolicy = vi.planFromState(initialState);
		return optionPolicy;
	}

	@Override
	public Planner getPlanner(StateConditionTest goal, HashableStateFactory hf) {
		// get the planner for current goal (e.g., option/subgoal)
		domainGenerator.setRf(new GoalBasedRF(goal));
		domainGenerator.setTf(tf);
		OOSADomain planningDomain = domainGenerator.generateDomain();
		if (planner == null) {
			System.out.println("init vi planner");
			planner = initializeVI(domain, hashingFactory);
		}
		planner.setDomain(planningDomain);
		planner.setHashingFactory(hf);
		planner.resetSolverKeepStates();
		return planner;
	}
	
	public ValueIteration initializeVI(OOSADomain planningDomain, HashableStateFactory hf) {
		double gamma = 0.90;
		double learningRate = 0.1;
		int maxIterations = 1000;
		ValueIteration vi = new ValueIteration(planningDomain, gamma, hf, learningRate, maxIterations);
		return vi;
	}

	public void visualizeValueFunction(ValueFunction valueFunction, Policy policy) {
//		List<State> allStates = getReachableStates();//StateReachability.getReachableStates(initialState, domain, new SimpleHashableStateFactory(false));
		List<State> allStates = StateReachability.getReachableStates(initialState, domain, hashingFactory);
		ValueFunctionVisualizerGUI gui = GridWorldDomain.getGridWorldValueFunctionVisualization(
				allStates,
				domainGenerator.getWidth(),
				domainGenerator.getHeight(),
				valueFunction, policy);
		gui.initGUI();
		
	}
	
	@Override
	public void evaluate(List<Option> options) {
		boolean visualize = false;
		boolean visOptionPolicies = true;
		double gamma = 0.99;
		double qInit = -99;
		double learningRate = 0.9;
		int numEpisodes = 200;
		int writeEvery = 1;
		int maxEpisodeSize = 1000;
		evaluateLearning(options, visualize, visOptionPolicies, gamma, qInit, learningRate, numEpisodes, writeEvery, maxEpisodeSize);
	}
	
	public void evaluateLearning(List<Option> options, boolean visualize, boolean visOptionPolicies, double gamma, double qInit, double learningRate, int numEpisodes, 
			int writeEvery, int maxEpisodeSize) {
		
		QLearning ql = new QLearning(domain, gamma, hashingFactory, qInit, learningRate, maxEpisodeSize);
		System.out.println("Giving the agent " + options.size() + " options");
		for (Option option : options) {
			ql.addActionType(new OptionType(option));
		}

		// visualize option policies
		
		if (visOptionPolicies) {
			List<ActionType> acts = ((QLearning)ql).getActionTypes();
			for(ActionType act : acts) {
				if(act.typeName().contains("option")) {
					OptionType optionType = (OptionType)act;
					SubgoalOption opt = (SubgoalOption)optionType.associatedAction(null);
					Policy policy = opt.getPolicy();
					ValueFunction vf = (ValueFunction) (((GreedyQPolicy)policy).getQPlanner());
					visualizeValueFunction(vf, policy);
				}
			}
		}
		
		
		if (visualize) {
			VisualActionObserver observer = new VisualActionObserver(domain, GridWorldVisualizer.getVisualizer(domainGenerator.getMap()));
			observer.initGUI();
			env = new EnvironmentServer(env, observer);
		}

		System.out.println("Begin learning...");
		for(int i = 0; i < numEpisodes; i++) {
			Episode ea = ql.runLearningEpisode(env, maxEpisodeSize);
			if (i % writeEvery == 0) {
				ea.write(getVisualizeFilePath() + "ql_" + i);
				System.out.println(i + ": " + ea.maxTimeStep());
			}
			env.resetEnvironment();
		}
		
	}
	
	@Override
	public void setStateHashingMasks() {
		// 
	}

	@Override
	public String getVisualizeFilePath() {
		return "./output_dw/";
	}
	
	@Override
	public Visualizer getVisualizer() {
		return GridWorldVisualizer.getVisualizer(domainGenerator.getMap());
	}

	public static void main(String[] args) {
		
		DoorWorldTrainer trainer = new DoorWorldTrainer(5, 14, 5, 14);
		Classifier model = new J48();
		String outputPath = "output/";
		String outputPrefix = "driver_";
		
		Random rng = new Random();
		
		PoptionsDriver driver = new PoptionsDriver(trainer, model, outputPath, outputPrefix);
		int numTrials = 100;
		driver.run(rng, numTrials);
			
	}

}
