package poptions;

import java.util.ArrayList;
import java.util.Arrays;
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
import burlap.behavior.singleagent.planning.Planner;
import burlap.behavior.singleagent.planning.deterministic.DDPlannerPolicy;
import burlap.behavior.singleagent.planning.deterministic.DeterministicPlanner;
import burlap.behavior.singleagent.planning.stochastic.valueiteration.ValueIteration;
import burlap.behavior.valuefunction.ValueFunction;
import burlap.domain.singleagent.gridworld.GridWorldDomain;
import burlap.domain.singleagent.gridworld.GridWorldVisualizer;
import burlap.mdp.core.TerminalFunction;
import burlap.mdp.core.oo.propositional.PropositionalFunction;
import burlap.mdp.core.state.State;
import burlap.mdp.singleagent.common.GoalBasedRF;
import burlap.mdp.singleagent.common.UniformCostRF;
import burlap.mdp.singleagent.common.VisualActionObserver;
import burlap.mdp.singleagent.environment.Environment;
import burlap.mdp.singleagent.environment.SimulatedEnvironment;
import burlap.mdp.singleagent.environment.extensions.EnvironmentServer;
import burlap.mdp.singleagent.model.RewardFunction;
import burlap.statehashing.HashableStateFactory;
import burlap.mdp.auxiliary.common.SinglePFTF;
import burlap.mdp.auxiliary.stateconditiontest.StateConditionTest;
import burlap.mdp.auxiliary.stateconditiontest.TFGoalCondition;
import burlap.statehashing.masked.MaskedHashableStateFactory;
import burlap.statehashing.simple.SimpleHashableStateFactory;
import burlap.visualizer.Visualizer;
import weka.classifiers.Classifier;
import weka.classifiers.trees.J48;

public class DoorWorldTrainer extends PoptionsTrainer {
	
	private Random rng;
	public DoorWorldDomain domainGenerator;
	public Environment env;
	private RewardFunction rf;
	private TerminalFunction tf;
	private HashableStateFactory hashingFactory;

	private int minWidth;
	private int maxWidth;
	private int minHeight;
	private int maxHeight;
	private List<String> classesInFeatureVector;
	private List<PropositionalFunction> propFunctions;
	public DoorWorldTrainer(int minWidth, int maxWidth, int minHeight, int maxHeight) {
		super();
		this.minWidth = minWidth;
		this.maxWidth = maxWidth;
		this.minHeight = minHeight;
		this.maxHeight = maxHeight;
		this.name = "door";
		this.classesInFeatureVector = Arrays.asList(DoorWorldDomain.CLASS_AGENT);
		propFunctions = new ArrayList<PropositionalFunction>();
		mhsf = new MaskedHashableStateFactory(false);
	}
	
	public DoorWorldTrainer(int minWidth, int maxWidth, int minHeight, int maxHeight, long seed) {
		this(minWidth, maxWidth, minHeight, maxHeight);
		initialize(seed, false);
	}
	
	@Override
	public void initialize(long seed, boolean testing) {
		
		System.out.println("Using seed: " + seed);
		
		rng = new Random(seed);
		
		int width = maxWidth-1;
		int height = maxHeight-1;
		if (!testing) {
			width = minWidth + rng.nextInt(maxWidth - minWidth);
			height = minHeight + rng.nextInt(maxHeight - minHeight);
		}
		
		//debug!
		if (testing) {
			width = 25;
			height = 25;
		}

		domainGenerator = new DoorWorldDomain(width, height);
		if(testing) {
			domainGenerator.setMapFourDoorsRandom(rng);
		} else {
			domainGenerator.setMapOneDoorRandom(rng);
		}
		// make it stochastic
		// MUST do this BEFORE generating domain
		domainGenerator.setProbSucceedTransitionDynamics(0.95);
		rf = new UniformCostRF();
		tf = new SinglePFTF(domain.propFunction(GridWorldDomain.PF_AT_LOCATION));
		domainGenerator.setRf(rf);
		domainGenerator.setTf(tf);
		domain = domainGenerator.generateDomain();
		goalCondition = new TFGoalCondition(tf);

		if (testing) {
			initialState = DoorWorldDomain.getOneAgentOneLocationFourDoorsState(domain);
			DoorWorldDomain.setAgent(initialState, 0, 0);
			domainGenerator.setFourDoors(initialState);
			System.out.println("testing");
			DoorWorldDomain.setLocation(initialState, 0, domainGenerator.getWidth()-1, domainGenerator.getHeight()-1);
			System.out.println("placing goal at " + (domainGenerator.getWidth()-1) + " " + (domainGenerator.getHeight()-1));
		} else {
			initialState = DoorWorldDomain.getOneAgentOneDoorState(domain);
			DoorWorldDomain.setAgent(initialState, domainGenerator.door1X, domainGenerator.door1Y);
			domainGenerator.setOneDoor(initialState);
			System.out.println("not testing");
			// IMPORTANT: this sets the 'poption' to train on learning to move to the door
			DoorWorldDomain.setLocation(initialState, 0, domainGenerator.door1X, domainGenerator.door1Y);
		}
		
		hashingFactory = new SimpleHashableStateFactory(false);
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
/*
	public void simpleValueFunctionVis(ValueFunction valueFunction, Policy p) {
		List<State> allStates = StateReachability.getReachableStates(initialState, (SADomain)domain, hashingFactory);
		ValueFunctionVisualizerGUI gui = GridWorldDomain.getGridWorldValueFunctionVisualization(allStates, valueFunction, p);
		gui.initGUI();
	}
	
	public Policy ValueIterationExample(String outputPath) {
		Planner planner = new ValueIteration(domain, rf, tf, 0.99, hashingFactory, 0.001, 100);
		Policy p = planner.planFromState(initialState);
		p.evaluateBehavior(initialState, rf, tf).writeToFile(outputPath + "vi");
//		visualize(outputPath);
		simpleValueFunctionVis((ValueFunction)planner, p);
		return p;
	}
	*/
	

	@Override
	public boolean train(String outputPath) {
		
		double gamma = 0.99;
		double maxDelta = 0.001;
		int maxIterations = 100;
		Planner planner = new ValueIteration(domain, gamma, hashingFactory, maxDelta, maxIterations);
		long startTime = System.nanoTime();
		Policy p = planner.planFromState(initialState);
		result = PolicyUtils.rollout(p, initialState, domain.getModel());
		long endTime = System.nanoTime();
		long duration = (endTime - startTime);
		System.out.println("total time: " + duration);
		
		result.writeToFile(outputPath + "dw_vi");

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

	public void simpleValueFunctionVis(ValueFunction valueFunction, Policy p) {
		List<State> allStates = StateReachability.getReachableStates(initialState, domain, new SimpleHashableStateFactory(false));
		ValueFunctionVisualizerGUI gui = GridWorldDomain.getGridWorldValueFunctionVisualization(
				allStates, domainGenerator.getWidth(), domainGenerator.getHeight(), valueFunction, p);
		gui.initGUI();
	}
	
	@Override
	public void evaluate(List<Option> options) {
		evaluateLearning(options, false, true, 0.99, -99, 0.9, 1001, 1, 1000);
	}
	
	public void evaluateLearning(List<Option> options, boolean visualize, boolean visOptionPolicies, double gamma, double qInit, double learningRate, int numEpisodes, 
			int writeEvery, int maxEpisodeSize) {
		
		QLearning ql = new QLearning(domain, gamma, new SimpleHashableStateFactory(false), qInit, learningRate, maxEpisodeSize);
		System.out.println("Giving the agent " + options.size() + " options");
		for (Option option : options) {
			ql.addNonDomainReferencedAction(option);
		}

		// visualize option policies
		if (visOptionPolicies) {
			for(Action act : ((QLearning)ql).getActions()) {
				if(act.getName().contains("option")) {
					DeterministicTerminationOption opt = (DeterministicTerminationOption)act;
					simpleValueFunctionVis((ValueFunction)((GreedyQPolicy)opt.getPolicy()).getQplanner(), opt.getPolicy());
				}
			}
		}
		
		if (visualize) {
			VisualActionObserver observer = new VisualActionObserver(domain, GridWorldVisualizer.getVisualizer(domainGenerator.getMap()));
			observer.initGUI();
			env = new EnvironmentServer(env, observer);
		}

		System.out.println("Begin learning...");
//		int totalSteps = 0;
		for(int i = 0; i < numEpisodes; i++){
			Episode ea = ql.runLearningEpisode(env, maxEpisodeSize);
			if (i % writeEvery == 0) {
				ea.writeToFile(getVisualizeFilePath() + "ql_" + i);
				System.out.println(i + ": " + ea.maxTimeStep());
			}
//			totalSteps += ea.maxTimeStep();
			env.resetEnvironment();
		}
//		System.out.println("Total steps: " + totalSteps);

//		EpisodeAnalysis.writeEpisodesToDisk(episodes, getVisualizeFilePath(), "ql_options");

		//visualize the learning episodes
//		Visualizer v = GridWorldVisualizer.getVisualizer((this).domainGenerator.getMap());
		//EpisodeSequenceVisualizer evis = new EpisodeSequenceVisualizer(v, domain, episodes);
//		EpisodeSequenceVisualizer evis = new EpisodeSequenceVisualizer(v, domain, "output_dw");
//		evis.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

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
	public Policy createOptionPolicy(StateConditionTest goal, HashableStateFactory hf) {
//		RewardFunction rf = new UniformCostRF();
//		TerminalFunction tf = new GoalConditionTF(goal);
		Planner planner = getPlanner(goal, hf);
		Policy optionPolicy = planner.planFromState(initialState);
//		Policy optionPolicy = new DDPlannerPolicy((DeterministicPlanner)planner);
//		optionPolicy.evaluateBehavior(initialState, rf, tf);
		return optionPolicy;
	}

	@Override
	public Planner getPlanner(StateConditionTest goal, HashableStateFactory hf) {
		Planner planner = new ValueIteration(domain, new GoalBasedRF(goal), tf, 0.99, hf, 0.9, 1000);
		return planner;
	}

	public static void main(String[] args) {
		
		DoorWorldTrainer trainer = new DoorWorldTrainer(5, 14, 5, 14);
		Classifier model = new J48();
		String outputPath = "output/";
		String outputPrefix = "dw";
		
		Random rng = new Random();
		
		PoptionsDriver driver = new PoptionsDriver(trainer, model, outputPath, outputPrefix);
		int numTrials = 100;
		driver.run(rng, numTrials);
			
	}

}
