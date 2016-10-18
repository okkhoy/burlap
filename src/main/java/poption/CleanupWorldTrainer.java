package poption;

import java.util.Arrays;
import java.util.List;
import java.util.Random;

import javax.swing.JFrame;

import burlap.behavior.policy.Policy;
import burlap.behavior.policy.PolicyUtils;
import burlap.behavior.singleagent.Episode;
import burlap.behavior.singleagent.auxiliary.EpisodeSequenceVisualizer;
import burlap.behavior.singleagent.auxiliary.StateReachability;
import burlap.behavior.singleagent.learning.tdmethods.QLearning;
import burlap.behavior.singleagent.options.Option;
import burlap.behavior.singleagent.planning.Planner;
import burlap.behavior.singleagent.planning.deterministic.DDPlannerPolicy;
import burlap.behavior.singleagent.planning.deterministic.DeterministicPlanner;
import burlap.behavior.singleagent.planning.deterministic.uninformed.bfs.BFS;
import burlap.behavior.singleagent.planning.stochastic.rtdp.BoundedRTDP;
import burlap.behavior.valuefunction.ConstantValueFunction;
import burlap.behavior.valuefunction.ValueFunction;
import burlap.debugtools.RandomFactory;
import burlap.mdp.auxiliary.common.GoalConditionTF;
import burlap.mdp.auxiliary.stateconditiontest.StateConditionTest;
import burlap.mdp.core.TerminalFunction;
import burlap.mdp.core.oo.state.OOState;
import burlap.mdp.core.state.State;
import burlap.mdp.singleagent.common.GoalBasedRF;
import burlap.mdp.singleagent.environment.SimulatedEnvironment;
import burlap.mdp.singleagent.model.RewardFunction;
import burlap.mdp.singleagent.oo.OOSADomain;
import burlap.oomdp.auxiliary.common.NullTermination;
import burlap.oomdp.singleagent.SADomain;
import burlap.statehashing.HashableStateFactory;
import burlap.statehashing.masked.MaskedHashableStateFactory;
import burlap.statehashing.simple.SimpleHashableStateFactory;
import burlap.visualizer.Visualizer;
import poption.domain.cleanup.*;
import poption.domain.cleanup.state.*;
import poptions.BoundedStateReachability;

public class CleanupWorldTrainer extends PoptionsTrainer {

	public static final int BOUND_REACHABLE_STATES = 100000;
	
	private int width;
	private int height;
	private Random rng;
	private CleanupWorld cw;
	private RewardFunction rf;
	private TerminalFunction tf;
	private String goalPF;
	private List<String> classesInFeatureVector;
	private int numBlocks = 1;
	private int numGoals = 1;
	private int randomFactoryMapIndex = 4245;
	
	private Planner planner;
	private CleanupGoalDescription[] goalDescriptions;
	
	public CleanupWorldTrainer(int numBlocks, int numGoals) {
		super();
		this.width = 13;
		this.height = 13;
		this.name = "cleanup";
		this.numBlocks = numBlocks;
		this.numGoals = numGoals;
		classesInFeatureVector = Arrays.asList(CleanupWorld.CLASS_AGENT, CleanupWorld.CLASS_BLOCK, CleanupWorld.CLASS_ROOM, CleanupWorld.CLASS_DOOR);
	}
	
	public CleanupWorldTrainer(int numBlocks, int numGoals, long seed, String goalPF) {
		this(numBlocks, numGoals);
		this.goalPF = goalPF;
		setStateHashingMasks();
	}

	public CleanupWorldTrainer(int numBlocks, int numGoals, long seed) {
		this(numBlocks, numGoals, seed, CleanupWorld.PF_BLOCK_IN_ROOM);
	}
	
	@Override
	public void setStateHashingMasks(){
		// as a trainer, we know what can be ignored by the agent for hashing inside p. option
		// this should not be used during evaluation since agent is not supposed to know what to ignore
		mhsf = new MaskedHashableStateFactory(false);
		if (goalPF.equals(CleanupWorld.PF_AGENT_IN_DOOR)) {
			mhsf.addObjectClassMasks(CleanupWorld.CLASS_DOOR, CleanupWorld.CLASS_ROOM, CleanupWorld.CLASS_BLOCK);
			mhsf.addVariableMasks(CleanupWorld.ATT_DIR);
		} else if (goalPF.equals(CleanupWorld.PF_BLOCK_IN_ROOM)){
			mhsf.addObjectClassMasks(CleanupWorld.CLASS_DOOR);
			mhsf.addVariableMasks(CleanupWorld.ATT_DIR);
		} else if (goalPF.equals(CleanupWorld.PF_BLOCK_IN_DOOR)){
			mhsf.addVariableMasks(CleanupWorld.ATT_DIR, CleanupWorld.ATT_LOCKED);
		}
	}
	
	@Override
	public void initialize(long seed, boolean testing) {
		
		// do __NOT__ call setStateHashingMasks
		
		System.out.println("Using seed: " + seed);
		System.out.println(numBlocks + " blocks, " + numGoals + " goals");
		
		RandomFactory.seedMapped(randomFactoryMapIndex, seed);
		rng = RandomFactory.getMapped(randomFactoryMapIndex);
		
		if (testing) {
			// evaluation goal task is to move a block to a room
			goalPF = CleanupWorld.PF_BLOCK_IN_ROOM;
		}
		
		cw = new CleanupWorld();
		cw.includeDirectionAttribute(true);
		cw.includeLockableDoors(false);
//		double lockProb = 0.0;
//		cw.setLockProbability(lockProb);
		
		CleanupRandomStateGenerator randomCleanup = new CleanupRandomStateGenerator();
		randomCleanup.setWidth(width);
		randomCleanup.setHeight(height);
		initialState = (OOState) randomCleanup.generateState(); //cw.getRandomState(domain, rng, numBlocks);
		CleanupGoal goal = new CleanupGoal();
		goalCondition = (StateConditionTest) goal;
		rf = new CleanupWorldRF(goalCondition, 1.0, 0.0);
		tf = new GoalConditionTF(goalCondition);
		cw.setRf(rf);
		cw.setTf(tf);
		domain = (OOSADomain) cw.generateDomain();
		
		goalDescriptions = CleanupRandomStateGenerator.getRandomGoalDescription(rng, (CleanupWorldState) initialState, numGoals, domain.propFunction(goalPF));
		goal.setGoals(goalDescriptions);
		
		System.out.println("Goal is: " + goalDescriptions[0]);
		
	}

	@Override
	public boolean train(String outputPath) {
		int maxSteps = 1000;
		double gamma = 0.99;
		int maxRolloutDepth = 50;
		double maxDiff = 0.01;
		int maxRollouts = 500;
		return train(outputPath, maxSteps, gamma, maxRolloutDepth, maxDiff, maxRollouts);
	}	

	public boolean train(String outputPath, int maxSteps, double gamma, int maxRolloutDepth, double maxDiff, int maxRollouts) {
		System.out.println("maxSteps: " + maxSteps);

		int maxTrajectoryLength = 10000;
		double lockProb = 0.0;
		RewardFunction heuristicRF = new CleanupWorldRF(goalCondition, 1.0, 0.0);
		ValueFunction heuristic = CleanupWorld.getGroundHeuristic(initialState, heuristicRF, lockProb);
		BoundedRTDP brtd = new BoundedRTDP(domain, gamma,
				new SimpleHashableStateFactory(false),
				new ConstantValueFunction(0.),
				heuristic, maxDiff, maxRollouts);
		brtd.setMaxRolloutDepth(maxRolloutDepth);
		brtd.toggleDebugPrinting(true);

		long startTime = System.nanoTime();
		Policy p = brtd.planFromState(initialState);
//		result = p.evaluateBehavior(env, maxSteps);
		SimulatedEnvironment env = new SimulatedEnvironment(domain, initialState);
		result = PolicyUtils.rollout(p, env, maxTrajectoryLength);
		long endTime = System.nanoTime();
		long duration = (endTime - startTime);
		System.out.println("total time: " + duration);
		result.write(outputPath + "cwr_brtdp");
		System.out.println("total actions: " + result.actionSequence.size());
		System.out.println("number Bellman: " + brtd.getNumberOfBellmanUpdates());
		System.out.println("maxTimeStep: " + result.maxTimeStep()+ " " + !((result.maxTimeStep() + 1 >= maxSteps)) );

		return !(result.maxTimeStep()+1 >= maxSteps);
	}
	
	@Override
	public Planner getPlanner(StateConditionTest goal, HashableStateFactory hf){
		cw.setRf(new GoalBasedRF(goal));
		cw.setTf(tf);
		OOSADomain planningDomain = (OOSADomain) cw.generateDomain();
//		if (planner == null) {
//			
//		}
		planner = initializeBFS(planningDomain, goal, hf);
		return planner;
	}
	

	@Override
	public List<State> getReachableStates() {
		List<State> allStates = StateReachability.getReachableStates(initialState, domain, mhsf, 100000);
		return allStates;
	}

	@Override
	public Visualizer getVisualizer() {
		return CleanupVisualizer.getVisualizer(width, height);
	}

	
	@Override
	public Policy createOptionPolicy(StateConditionTest goal, HashableStateFactory hf) {
		cw.setRf(new CleanupWorldRF(goal, 1., 0.));
		cw.setTf(new GoalConditionTF(goal));
		OOSADomain optionDomain = (OOSADomain) cw.generateDomain();
//		BoundedRTDP brtdp = initializeBRTDP(optionDomain, hf);
		BFS bfs = initializeBFS(optionDomain, goal, hf);
//		Policy optionPolicy = new DDPlannerPolicy((DeterministicPlanner)planner);
		Policy optionPolicy = bfs.planFromState(initialState);
		return optionPolicy;
	}
	
//	public BoundedRTDP initializeBRTDP(OOSADomain planningDomain, HashableStateFactory hf) {
//	
//}
	
	public BFS initializeBFS(OOSADomain planningDomain, StateConditionTest goal, HashableStateFactory hf) {
		BFS bfs = new BFS(domain, goal, hf);
		bfs.toggleDebugPrinting(false);
		return bfs;
	}
	
	@Override
	public String getFeatureVectorHeader() {
		return getFeatureVectorNames(classesInFeatureVector, null);
	}

	@Override
	public List<double[]> getTrainingFeatureVectors() {
		return getTrainingFeatureVectors(classesInFeatureVector, null);
	}

	@Override
	public List<double[]> getTestFeatureVectors() {
		return getTestFeatureVectors(classesInFeatureVector, null);
	}
	
	@Override
	public void visualize(String outputPath) {
		Visualizer v = getVisualizer();
		EpisodeSequenceVisualizer esv = new EpisodeSequenceVisualizer(v, domain, outputPath);
		esv.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		esv.initGUI();
	}
	
	@Override
	public String getVisualizeFilePath() {
		return "./output_eval/";
	}
	
	@Override
	public void evaluate(List<Option> options) {
//		evaluatePlanning(options, false, 100, 0.99, 50, 0.01, 100);
//		evaluateLearning(options, false, 0.99, 1.0, 0.9, 1001, 10, 1000);
	}

	/*
	public void evaluateLearning(List<Option> options, boolean visualize, double discountFactor,
			double initialQValues, double learningRate, int numEpisodes, 
			int writeEvery, int maxEpisodeSize) {
		Environment env = new FixedDoorCleanupEnv(domain, rf, tf, initialState);
		QLearning agent = new QLearning(domain, discountFactor, new SimpleHashableStateFactory(false), initialQValues, learningRate, maxEpisodeSize);
		if (visualize) {
			// live visualization
			int delay = 150;
			VisualActionObserver observer = new VisualActionObserver(domain, 
					getVisualizer());
			observer.setFrameDelay(delay);
			observer.initGUI();
			observer.setFrameDelay(delay);
			env = new EnvironmentServer(env, observer);
		}
		System.out.println("giving the agent " + options.size() + " options");
		for (Option o : options) {
			agent.addNonDomainReferencedAction(o);
        }
		for (int i = 0; i < numEpisodes; i++) {
			EpisodeAnalysis ea = agent.runLearningEpisode(env, maxEpisodeSize);
			if (i % writeEvery == 0) {
				ea.writeToFile(getVisualizeFilePath() + "ql_" + i);
				System.out.println(i + ": " + ea.maxTimeStep());
			}
			env.resetEnvironment();
		}
	}
	
	/*
	public void evaluatePlanning(List<Option> options, boolean visualize, int maxSteps, double gamma, int maxRolloutDepth, double maxDiff, int maxRollouts) {
		Environment env = new FixedDoorCleanupEnv(domain, rf, tf, initialState);

		System.out.println("maxSteps: " + maxSteps);
		System.out.println(goalDescriptions[0]);
		RewardFunction heuristicRF = new CleanupWorldRF(goalCondition, 1., 0.);
		ValueFunctionInitialization heuristic = CleanupRandomDomainDriver.getL0Heuristic(initialState, heuristicRF);
		BoundedRTDP brtd = new BoundedRTDP(domain, rf, tf, gamma, new SimpleHashableStateFactory(false),
				new ValueFunctionInitialization.ConstantValueFunctionInitialization(0.), heuristic, maxDiff, maxRollouts);
		brtd.setMaxRolloutDepth(maxRolloutDepth);
		brtd.toggleDebugPrinting(true);
		System.out.println("giving the agent " + options.size() + " options");
		for (Option o : options) {
			brtd.addNonDomainReferencedAction(o);
		}
		VisualActionObserver observer = null;
		if (visualize) {
			int delay = 150;
			observer = new VisualActionObserver(domain, 
					getVisualizer());
			observer.setFrameDelay(delay);
			observer.initGUI();
			observer.setFrameDelay(delay);
			((SADomain)domain).addActionObserverForAllAction(observer);
		}
		long startTime = System.nanoTime();
		Policy P = brtd.planFromState(initialState);
		if (visualize) {
			env = new EnvironmentServer(env, observer);
		}
		result = P.evaluateBehavior(env, maxSteps);
		long endTime = System.nanoTime();
		long duration = (endTime - startTime);
		System.out.println("total time: " + duration);
		result.write("./output_eval/cwr_brtdp");
	}
	*/
	
	public static void main(String[] args) {
//		String outputPath = "output_cw_main/";
//		Random rng = RandomFactory.seedMapped(0,11324);//42342343);
//		long seed = rng.nextLong();
//		System.out.println("Using seed " + seed);
//		CleanupWorldTrainer trainer = new CleanupWorldTrainer(1,1,seed,CleanupWorld.PF_AGENT_IN_DOOR);
//		
//		trainer.visualize(outputPath);
//		
//		long startTime = System.nanoTime();
//		trainer.train(outputPath + "main1_");
//		long endTime = System.nanoTime();
//		long duration = (endTime - startTime); 
//		System.out.println("Duration: " + duration);
	}
	
}
