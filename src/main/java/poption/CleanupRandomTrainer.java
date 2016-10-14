package poption;

import java.util.Arrays;
import java.util.List;
import java.util.Random;

import javax.swing.JFrame;

import burlap.behavior.policy.Policy;
import burlap.behavior.singleagent.auxiliary.EpisodeSequenceVisualizer;
import burlap.behavior.singleagent.learning.tdmethods.QLearning;
import burlap.behavior.singleagent.options.Option;
import burlap.behavior.singleagent.planning.Planner;
import burlap.behavior.singleagent.planning.deterministic.DDPlannerPolicy;
import burlap.behavior.singleagent.planning.deterministic.DeterministicPlanner;
import burlap.behavior.singleagent.planning.deterministic.uninformed.bfs.BFS;
import burlap.behavior.singleagent.planning.stochastic.rtdp.BoundedRTDP;
import burlap.debugtools.RandomFactory;
import burlap.mdp.auxiliary.common.GoalConditionTF;
import burlap.mdp.auxiliary.stateconditiontest.StateConditionTest;
import burlap.mdp.core.TerminalFunction;
import burlap.mdp.core.oo.state.OOState;
import burlap.mdp.singleagent.common.GoalBasedRF;
import burlap.mdp.singleagent.model.RewardFunction;
import burlap.mdp.singleagent.oo.OOSADomain;
import burlap.statehashing.masked.MaskedHashableStateFactory;
import poption.domain.cleanup.CleanupGoal;
import poption.domain.cleanup.CleanupGoalDescription;
import poption.domain.cleanup.CleanupWorld;
import poption.domain.cleanup.state.CleanupRandomStateGenerator;
import poption.domain.cleanup.state.CleanupWorldState;

public class CleanupRandomTrainer extends PoptionsTrainer {

	private Random rng;
	private CleanupWorld cw;
	private RewardFunction rf;
	private TerminalFunction tf;
	private Planner planner;
	private String goalPF;
	private List<String> classesInFeatureVector;
	private int numBlocks = 1;
	private int numGoals = 1;
	private int randomFactoryMapIndex = 4245;
	
	private CleanupGoalDescription[] goalDescriptions;
	
	public CleanupRandomTrainer(int numBlocks, int numGoals) {
		super();
		this.name = "cleanup";
		this.numBlocks = numBlocks;
		this.numGoals = numGoals;
		classesInFeatureVector = Arrays.asList(CleanupWorld.CLASS_AGENT, CleanupWorld.CLASS_BLOCK, CleanupWorld.CLASS_ROOM);
	}
	
	public CleanupRandomTrainer(int numBlocks, int numGoals, long seed) {
		this(numBlocks, numGoals, seed, CleanupWorld.PF_BLOCK_IN_ROOM);
	}
	
	public CleanupRandomTrainer(int numBlocks, int numGoals, long seed, String goalPF) {
		this(numBlocks, numGoals);
		this.goalPF = goalPF;
		setStateHashingMasks();
		initialize(seed, false);
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
		double lockProb = 0.0;
		
		if (testing) {
			goalPF = CleanupWorld.PF_BLOCK_IN_ROOM;
		}
		
		// do __NOT__ call setStateHashingMasks
		
		System.out.println("Using seed: " + seed);
		System.out.println(numBlocks + " blocks, " + numGoals + " goals");
		
		RandomFactory.seedMapped(randomFactoryMapIndex, seed);
		rng = RandomFactory.getMapped(randomFactoryMapIndex);

		cw = new CleanupWorld();
		cw.includeDirectionAttribute(true);
		cw.includeLockableDoors(true);
		cw.setLockProbability(lockProb);
		domain = (OOSADomain) cw.generateDomain();
		
		CleanupRandomStateGenerator randomCleanup = new CleanupRandomStateGenerator();
		
		initialState = (OOState) randomCleanup.generateState(); //cw.getRandomState(domain, rng, numBlocks);
		
		goalDescriptions = CleanupRandomStateGenerator.getRandomGoalDescription(rng, (CleanupWorldState) initialState, numGoals, domain.propFunction(goalPF));
		System.out.println(goalDescriptions[0]);
		
		goalCondition = (StateConditionTest) new CleanupGoal(goalDescriptions);
		rf = new GoalBasedRF(goalCondition, 1., 0.);
		tf = new GoalConditionTF(goalCondition);
		
	}

	@Override
	public boolean train(String outputPath) {
		return train(outputPath, 1000, 0.99, 50, 0.01, 500);
	}		
	public boolean train(String outputPath, int maxSteps, double gamma, int maxRolloutDepth, double maxDiff, int maxRollouts) {
		System.out.println("maxSteps: " + maxSteps);

		FixedDoorCleanupEnv env = new FixedDoorCleanupEnv(domain, rf, tf, initialState);
		RewardFunction heuristicRF = new CleanupWorldRF(goalCondition, 1., 0.);
		ValueFunctionInitialization heuristic = CleanupRandomDomainDriver.getL0Heuristic(initialState, heuristicRF);
		BoundedRTDP brtd = new BoundedRTDP(domain, rf, tf, gamma, new SimpleHashableStateFactory(false),
				new ValueFunctionInitialization.ConstantValueFunctionInitialization(0.), heuristic, maxDiff, maxRollouts);
		brtd.setMaxRolloutDepth(maxRolloutDepth);
		brtd.toggleDebugPrinting(true);

		//visualize execution
//		VisualActionObserver observer = new VisualActionObserver(domain, 
//				CleanupVisualizer.getVisualizer("data/resources/robotImages/"));
//		observer.initGUI();
//		((SADomain)domain).addActionObserverForAllAction(observer);
		
		long startTime = System.nanoTime();
		Policy P = brtd.planFromState(initialState);
		result = P.evaluateBehavior(env, maxSteps);
		long endTime = System.nanoTime();
		long duration = (endTime - startTime);
		System.out.println("total time: " + duration);
		result.writeToFile(outputPath + "cwr_brtdp");
		System.out.println("total actions: " + result.actionSequence.size());
		System.out.println("number Bellman: " + brtd.getNumberOfBellmanUpdates());
		System.out.println("maxTimeStep: " + result.maxTimeStep()+ " " + !((result.maxTimeStep() + 1 >= maxSteps)) );

		return !(result.maxTimeStep()+1 >= maxSteps);
	}
	
	@Override
	public Planner getPlanner(StateConditionTest goal, HashableStateFactory hf){
		DeterministicPlanner dp = new BFS(domain, goal, hf);
		dp.toggleDebugPrinting(false);
		planner = dp;
		return planner;
	}
	
	@Override
	public Policy createOptionPolicy(StateConditionTest goal, HashableStateFactory hf) {
		RewardFunction rf = new CleanupWorldRF(goal, 1., 0.);
		TerminalFunction tf = new GoalConditionTF(goal);
		Planner planner = getPlanner(goal, hf);
		Policy optionPolicy = new DDPlannerPolicy((DeterministicPlanner)planner);
		optionPolicy.evaluateBehavior(initialState, rf, tf);
		return optionPolicy;
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
		Visualizer v = CleanupVisualizer.getVisualizer("data/resources/robotImages/");
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
		evaluateLearning(options, false, 0.99, 1.0, 0.9, 1001, 10, 1000);
	}

	public void evaluateLearning(List<Option> options, boolean visualize, double discountFactor,
			double initialQValues, double learningRate, int numEpisodes, 
			int writeEvery, int maxEpisodeSize) {
		Environment env = new FixedDoorCleanupEnv(domain, rf, tf, initialState);
		QLearning agent = new QLearning(domain, discountFactor, new SimpleHashableStateFactory(false), initialQValues, learningRate, maxEpisodeSize);
		if (visualize) {
			// live visualization
			int delay = 150;
			VisualActionObserver observer = new VisualActionObserver(domain, 
					CleanupVisualizer.getVisualizer("data/resources/robotImages/"));
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
					CleanupVisualizer.getVisualizer("data/resources/robotImages/"));
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
		result.writeToFile("./output_eval/cwr_brtdp");
	}
	
	public static void main(String[] args) {
		String outputPath = "output_cw_main/";
		Random rng = RandomFactory.seedMapped(0,11324);//42342343);
		long seed = rng.nextLong();
		System.out.println("Using seed " + seed);
		CleanupRandomTrainer trainer = new CleanupRandomTrainer(1,1,seed,CleanupWorld.PF_AGENT_IN_DOOR);
		
		trainer.visualize(outputPath);
		
		long startTime = System.nanoTime();
		trainer.train(outputPath + "main1_");
		long endTime = System.nanoTime();
		long duration = (endTime - startTime); 
		System.out.println("Duration: " + duration);
	}
}
