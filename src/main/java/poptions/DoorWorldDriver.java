package poptions;
import java.util.List;
import java.util.Random;

import javax.swing.JFrame;

import burlap.behavior.functionapproximation.dense.ConcatenatedObjectFeatures;
import burlap.behavior.functionapproximation.dense.NumericVariableFeatures;
import burlap.behavior.functionapproximation.dense.PFFeatures;
import burlap.behavior.policy.GreedyQPolicy;
import burlap.behavior.policy.Policy;
import burlap.behavior.policy.PolicyUtils;
import burlap.behavior.singleagent.Episode;
import burlap.behavior.singleagent.auxiliary.EpisodeSequenceVisualizer;
import burlap.behavior.singleagent.auxiliary.StateReachability;
import burlap.behavior.singleagent.auxiliary.performance.LearningAlgorithmExperimenter;
import burlap.behavior.singleagent.auxiliary.performance.PerformanceMetric;
import burlap.behavior.singleagent.auxiliary.performance.TrialMode;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.ValueFunctionVisualizerGUI;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.ArrowActionGlyph;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.LandmarkColorBlendInterpolation;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.PolicyGlyphPainter2D;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.StateValuePainter2D;
import burlap.behavior.singleagent.learning.LearningAgent;
import burlap.behavior.singleagent.learning.LearningAgentFactory;
import burlap.behavior.singleagent.learning.tdmethods.QLearning;
import burlap.behavior.singleagent.learning.tdmethods.SarsaLam;
import burlap.behavior.singleagent.planning.Planner;
import burlap.behavior.singleagent.planning.deterministic.DeterministicPlanner;
import burlap.behavior.singleagent.planning.deterministic.informed.Heuristic;
import burlap.behavior.singleagent.planning.deterministic.informed.astar.AStar;
import burlap.behavior.singleagent.planning.deterministic.uninformed.bfs.BFS;
import burlap.behavior.singleagent.planning.deterministic.uninformed.dfs.DFS;
import burlap.behavior.singleagent.planning.stochastic.valueiteration.ValueIteration;
import burlap.behavior.valuefunction.QFunction;
import burlap.behavior.valuefunction.ValueFunction;
import burlap.domain.singleagent.gridworld.GridWorldDomain;
import burlap.domain.singleagent.gridworld.GridWorldTerminalFunction;
import burlap.domain.singleagent.gridworld.GridWorldVisualizer;
import burlap.domain.singleagent.gridworld.state.GridAgent;
import burlap.domain.singleagent.gridworld.state.GridLocation;
import burlap.domain.singleagent.gridworld.state.GridWorldState;
import burlap.mdp.auxiliary.stateconditiontest.StateConditionTest;
import burlap.mdp.auxiliary.stateconditiontest.TFGoalCondition;
import burlap.mdp.core.Domain;
import burlap.mdp.core.TerminalFunction;
import burlap.mdp.core.oo.OODomain;
import burlap.mdp.core.oo.state.OOState;
import burlap.mdp.core.oo.state.generic.GenericOOState;
import burlap.mdp.core.state.State;
import burlap.mdp.singleagent.common.UniformCostRF;
import burlap.mdp.singleagent.common.VisualActionObserver;
import burlap.mdp.singleagent.environment.Environment;
import burlap.mdp.singleagent.environment.SimulatedEnvironment;
import burlap.mdp.singleagent.model.RewardFunction;
import burlap.mdp.singleagent.oo.OOSADomain;
import burlap.shell.visual.VisualExplorer;
import burlap.mdp.auxiliary.common.SinglePFTF;
import burlap.statehashing.HashableStateFactory;
import burlap.statehashing.simple.SimpleHashableStateFactory;
import burlap.visualizer.Visualizer;

public class DoorWorldDriver {

	DoorWorldDomain domainGenerator;
	OOSADomain domain;
	RewardFunction rf;
	TerminalFunction tf;
	StateConditionTest goalCondition;
	OOState initialState;
	HashableStateFactory hashingFactory;
	Environment env;
	
	public DoorWorldDriver(int w, int h){

		int goalX = w-1;
		int goalY = h-1;
		rf = new UniformCostRF();
		tf = new GridWorldTerminalFunction(goalX, goalY);
		goalCondition = new TFGoalCondition(tf);
		Random rng = new Random();
		domainGenerator = new DoorWorldDomain(w, h);
		domainGenerator.setRf(rf);
		domainGenerator.setTf(tf);
		domainGenerator.setMapFourDoorsRandom(rng);
		domain = domainGenerator.generateDomain();
//		initialState = new DoorWorldState(0, 0, new GridLocation(goalX, goalY, "loc0"));
		int ax = 1;
		int ay = 1;
		int lx = goalX;
		int ly = goalY;
		initialState = domainGenerator.getOneAgentOneLocationFourDoorsState(ax, ay, lx, ly);
		
		hashingFactory = new SimpleHashableStateFactory();
		env = new SimulatedEnvironment(domain, initialState);
		
//		VisualActionObserver observer = new VisualActionObserver(domain, GridWorldVisualizer.getVisualizer(gwdg.getMap()));
//		observer.initGUI();
		// use the next line for learning algos
//		env = new EnvironmentServer(env, observer);
		// use the next line INSTEAD for planning algos
//		((SADomain)domain).addActionObserverForAllAction(observer);	
		
	}
	
	public void runInteractiveDemo() {
		Visualizer v = GridWorldVisualizer.getVisualizer(domainGenerator.getMap());
		VisualExplorer exp = new VisualExplorer(domain, env, v);
		exp.addKeyAction("w", GridWorldDomain.ACTION_NORTH, "");
		exp.addKeyAction("s", GridWorldDomain.ACTION_SOUTH, "");
		exp.addKeyAction("d", GridWorldDomain.ACTION_EAST, "");
		exp.addKeyAction("a", GridWorldDomain.ACTION_WEST, "");
		exp.initGUI();
		exp.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
	}
	
	public void simpleValueFunctionVis(ValueFunction valueFunction, Policy p){
		List<State> allStates = StateReachability.getReachableStates(
			initialState, domain, hashingFactory);
		ValueFunctionVisualizerGUI gui = GridWorldDomain.getGridWorldValueFunctionVisualization(
			allStates, domainGenerator.getWidth(), domainGenerator.getHeight(), valueFunction, p);
		gui.initGUI();
		gui.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
	}

	public void runVI() {
		String outputPath = "./output/";
		double gamma = 0.99;
		double maxDelta = 0.001;
		int maxIterations = 100;
		Planner planner = new ValueIteration(domain, gamma, hashingFactory, maxDelta, maxIterations);
		System.out.println("begin planning");
		Policy p = planner.planFromState(initialState);
		System.out.println("begin rollout");
		PolicyUtils.rollout(p, initialState, domain.getModel()).write(outputPath + "vi");
		System.out.println("finished rollout");
		simpleValueFunctionVis((ValueFunction)planner, p);
	}
	
	public void runQLearning(){
		String outputPath = "./output/";
		double gamma = 0.95;
		double qInit = 0.;
		double learningRate = 0.1;
		int nEpisodes = 200;
		LearningAgent agent = new QLearning(domain, gamma, hashingFactory, qInit, learningRate);
		//run learning for 50 episodes
		for(int i = 0; i < nEpisodes; i++){
			Episode e = agent.runLearningEpisode(env);
			e.write(outputPath + "ql_" + i);
			System.out.println(i + ": " + e.maxTimeStep());
			//reset environment for next learning episode
			env.resetEnvironment();
		}
	}
	
	public void visualize() {
		String outputPath = "./output/";
		Visualizer v = GridWorldVisualizer.getVisualizer(domainGenerator.getMap());
		EpisodeSequenceVisualizer esv = new EpisodeSequenceVisualizer(v, domain, outputPath);
		esv.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
	}
	
	public static void main(String[] args) {
		DoorWorldDriver example = new DoorWorldDriver(9, 9);
//		example.runInteractiveDemo();
//		example.runVI();
//		example.runQLearning();
//		example.visualize();
		
		List<Class<?>> objectClasses = example.domain.stateClasses();
		System.out.println(objectClasses.get(0));
		List<String> objectClassNames = example.domain.stateClassesNames();
		System.out.println(objectClassNames.get(0));
		ConcatenatedObjectFeatures cof = new ConcatenatedObjectFeatures();
		PFFeatures pff = new PFFeatures(example.domain.propFunctions());
		for (String className : example.domain.stateClassesNames()) {
			System.out.println("in loop "+className);
			cof.addObjectVectorization(className, new NumericVariableFeatures());
		}
		System.out.println(cof.features(example.initialState).length);
		for (double d : cof.features(example.initialState)) {
			System.out.print("here "+d);
		}
		System.out.println("");
		System.out.println(cof.featureNames(example.initialState));
		System.out.println(pff.featureNames(example.initialState));
	}
	
}
