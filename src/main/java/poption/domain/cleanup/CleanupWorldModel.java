package poption.domain.cleanup;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import burlap.debugtools.RandomFactory;
import burlap.mdp.core.StateTransitionProb;
import burlap.mdp.core.action.Action;
import burlap.mdp.core.oo.ObjectParameterizedAction;
import burlap.mdp.core.oo.state.ObjectInstance;
import burlap.mdp.core.oo.state.generic.GenericOOState;
import burlap.mdp.core.state.State;
import burlap.mdp.singleagent.model.statemodel.FullStateModel;
import burlap.mdp.singleagent.oo.ObjectParameterizedActionType;
import burlap.mdp.singleagent.oo.ObjectParameterizedActionType.SAObjectParameterizedAction;
import poption.domain.cleanup.state.CleanupAgent;
import poption.domain.cleanup.state.CleanupBlock;
import poption.domain.cleanup.state.CleanupRoom;
import poption.domain.cleanup.state.CleanupWorldState;

public class CleanupWorldModel implements FullStateModel {

		protected int minX;
		protected int minY;
		protected int maxX;
		protected int maxY;
		protected double [][] transitionProbs;
		
		public CleanupWorldModel() {
			
		}
		
		public CleanupWorldModel(int minX, int minY, int maxX, int maxY, int numActions) {
			this.minX = minX;
			this.minY = minY;
			this.maxX = maxX;
			this.maxY = maxY;
			this.transitionProbs = new double[numActions][numActions];
			for(int i = 0; i < numActions; i++){
				for(int j = 0; j < numActions; j++){
					double p = i != j ? 0 : 1;
					transitionProbs[i][j] = p;
				}
			}
		}

		@Override
		public List<StateTransitionProb> stateTransitions(State s, Action a) {
			return FullStateModel.Helper.deterministicTransition(this, s, a);
		}
		
		@Override
		public State sample(State s, Action a) {
			s = s.copy();
			String actionName = a.actionName();
			if(actionName.equals(CleanupWorld.ACTION_NORTH)
			|| actionName.equals(CleanupWorld.ACTION_SOUTH)
			|| actionName.equals(CleanupWorld.ACTION_EAST)
			|| actionName.equals(CleanupWorld.ACTION_WEST)) {
				return move(s, actionName);
			} else if (actionName.equals(CleanupWorld.ACTION_PULL)) {
//				return pull(s, (SAObjectParameterizedAction)a);
				return pull(s, a);
			} else if (actionName.equals(CleanupWorld.ACTION_PUSH)) {
				return push(s, a);
			} else if (actionName.equals(CleanupWorld.ACTION_TURN_LEFT)) {
				return turn(s, a);
			} else if (actionName.equals(CleanupWorld.ACTION_TURN_RIGHT)) {
				return turn(s, a);
			}
			throw new RuntimeException("Unknown action " + actionName);
		}
		
		public State turn(State s, Action action) {
			
			CleanupWorldState cws = (CleanupWorldState)s;
			CleanupAgent agent = cws.agent;
			String current = (String) agent.get(CleanupWorld.ATT_DIR);
			String direction = current;
			if (current.equals(CleanupWorld.ACTION_NORTH)) {
				if (action.actionName().equals(CleanupWorld.ACTION_TURN_LEFT)) {
					direction = CleanupWorld.ACTION_WEST;
				} else {
					direction = CleanupWorld.ACTION_EAST;
				}
			} else if (current.equals(CleanupWorld.ACTION_SOUTH)) {
				if (action.actionName().equals(CleanupWorld.ACTION_TURN_LEFT)) {
					direction = CleanupWorld.ACTION_EAST;
				} else {
					direction = CleanupWorld.ACTION_WEST;
				}
			} else if (current.equals(CleanupWorld.ACTION_EAST)) {
				if (action.actionName().equals(CleanupWorld.ACTION_TURN_LEFT)) {
					direction = CleanupWorld.ACTION_NORTH;
				} else {
					direction = CleanupWorld.ACTION_SOUTH;
				}
			} else if (current.equals(CleanupWorld.ACTION_WEST)) {
				if (action.actionName().equals(CleanupWorld.ACTION_TURN_LEFT)) {
					direction = CleanupWorld.ACTION_SOUTH;
				} else {
					direction = CleanupWorld.ACTION_NORTH;
				}
			}
			CleanupAgent nAgent = cws.touchAgent();
			nAgent.set(CleanupWorld.ATT_DIR, direction);
			return s;
		}

		
//		public State pull(State s, SAObjectParameterizedAction action) {
		public State pull(State s, Action action) {
			
			CleanupWorldState cws = (CleanupWorldState)s;
			
			CleanupAgent agent = cws.agent;
			int direction = actionDir(agent.get(CleanupWorld.ATT_DIR).toString());
			int ax = (Integer) agent.get(CleanupWorld.ATT_X);
			int ay = (Integer) agent.get(CleanupWorld.ATT_Y);
			//first get change in x and y from direction using 0: north; 1: south; 2:east; 3: west
			int xdelta = 0;
			int ydelta = 0;
			if(direction == 0){
				ydelta = 1;
			} else if(direction == 1){
				ydelta = -1;
			} else if(direction == 2){
				xdelta = 1;
			} else{
				xdelta = -1;
			}
			int nx = ax + xdelta;
			int ny = ay + ydelta;
			CleanupBlock block = cws.getBlockAtPoint(nx, ny);
			if(block != null){
				int bx = (Integer) block.get(CleanupWorld.ATT_X);
				int by = (Integer) block.get(CleanupWorld.ATT_Y);
				int nbx = ax;
				int nby = ay;
				CleanupBlock nBlock = cws.touchBlock(block.name());
				nBlock.set(CleanupWorld.ATT_X, nbx);
				nBlock.set(CleanupWorld.ATT_Y, nby);
				//face in direction of the block movement
				String newDirection = "";
				if(by - ay > 0){
					newDirection = CleanupWorld.ACTION_SOUTH;
				}
				else if(by - ay < 0){
					newDirection = CleanupWorld.ACTION_NORTH;
				}
				else if(bx - ax > 0){
					newDirection = CleanupWorld.ACTION_WEST;
				}
				else if(bx - ax < 0){
					newDirection = CleanupWorld.ACTION_EAST;
				}
				CleanupAgent nAgent = cws.touchAgent();
				nAgent.set(CleanupWorld.ATT_X, nx);
				nAgent.set(CleanupWorld.ATT_Y, ny);
				nAgent.set(CleanupWorld.ATT_DIR, newDirection);
			} 
			return s;
		}

		public State push(State s, Action action) {
			
			CleanupWorldState cws = (CleanupWorldState)s;
			CleanupAgent agent = cws.agent;
			String agentDirection = (String) agent.get(CleanupWorld.ATT_DIR);
			int direction = actionDir(agentDirection);
			int curX = (Integer) agent.get(CleanupWorld.ATT_X);
			int curY = (Integer) agent.get(CleanupWorld.ATT_Y);
			//first get change in x and y from direction using 0: north; 1: south; 2:east; 3: west
			int xdelta = 0;
			int ydelta = 0;
			if(direction == 0){
				ydelta = 1;
			} else if(direction == 1){
				ydelta = -1;
			} else if(direction == 2){
				xdelta = 1;
			} else{
				xdelta = -1;
			}
			int nx = curX + xdelta;
			int ny = curY + ydelta;
			int nbx = nx;
			int nby = ny;

			boolean agentCanPush = false;
			CleanupRoom roomContaining = cws.roomContainingPointIncludingBorder(nx, ny);
			CleanupBlock pushedBlock = cws.getBlockAtPoint(nx, ny);
			if(pushedBlock == null){
				agentCanPush = false;
			} else {
				int bx = (Integer) pushedBlock.get(CleanupWorld.ATT_X);
				int by = (Integer) pushedBlock.get(CleanupWorld.ATT_Y);
				nbx = bx + xdelta;
				nby = by + ydelta;
				if(cws.isOpen(roomContaining, nbx, nby)){
						agentCanPush = true;
//					}
				}
			}
			if (agentCanPush){
				CleanupBlock nBlock = cws.touchBlock(pushedBlock.name());
				nBlock.set(CleanupWorld.ATT_X, nbx);
				nBlock.set(CleanupWorld.ATT_Y, nby);
				CleanupAgent nAgent = cws.touchAgent();
				nAgent.set(CleanupWorld.ATT_X, nx);
				nAgent.set(CleanupWorld.ATT_Y, ny);
				nAgent.set(CleanupWorld.ATT_DIR, agentDirection);
			}
			return s;
		}

		public State move(State s, String actionName) {
			
			CleanupWorldState cws = (CleanupWorldState)s;
			CleanupAgent agent = cws.agent;
			int direction = actionDir(actionName);
			int curX = (Integer) agent.get(CleanupWorld.ATT_X);
			int curY = (Integer) agent.get(CleanupWorld.ATT_Y);
			//first get change in x and y from direction using 0: north; 1: south; 2:east; 3: west
			int xdelta = 0;
			int ydelta = 0;
			if(direction == 0){
				ydelta = 1;
			} else if(direction == 1){
				ydelta = -1;
			} else if(direction == 2){
				xdelta = 1;
			} else{
				xdelta = -1;
			}
			int nx = curX + xdelta;
			int ny = curY + ydelta;

			boolean agentCanMove = false;
			CleanupRoom roomContaining = cws.roomContainingPointIncludingBorder(nx, ny);
			CleanupBlock block = cws.getBlockAtPoint(nx, ny);
			if(block == null){
				if(!cws.wallAt(roomContaining, nx, ny)){
					agentCanMove = true;
				}
			} else {
				agentCanMove = false;
			}
			if (agentCanMove){
				CleanupAgent nAgent = cws.touchAgent();
				nAgent.set(CleanupWorld.ATT_X, nx);
				nAgent.set(CleanupWorld.ATT_Y, ny);
				nAgent.set(CleanupWorld.ATT_DIR, actionName);
			}
			return s;
		}
		
		
		protected boolean checkDoorLockStatus(CleanupWorldState cws, int nbx, int nby) {
			boolean isUnlocked = true;
//			CleanupDoor doorAtNewPoint = cws.doorContainingPoint(nbx, nby);
//			if(doorAtNewPoint != null){
//				String val = doorAtNewPoint.get(CleanupWorld.ATT_LOCKED).toString();
//				if(val.equals("locked")) { //locked door
//					updatePosition = false;
//				} else if(val.equals("unknown")){ //unknown door
//					CleanupDoor copy = (CleanupDoor) doorAtNewPoint.copy();
//					double roll = RandomFactory.getMapped(0).nextDouble();
//					if(roll < this.lockProb){
//						updatePosition = false;
//						copy.set(CleanupWorld.ATT_LOCKED, "locked");
//					} else{
//						//unlock the door
//						copy.set(CleanupWorld.ATT_LOCKED, "unlocked");
//					}
//					s.doors.put(copy.name(),copy);
//				}
//			}
			return isUnlocked;
		}
		
		protected static int actionDir(String actionName) {
			int direction = -1;
			if (actionName.equals(CleanupWorld.ACTION_NORTH)) {
				direction = 0;
			} else if (actionName.equals(CleanupWorld.ACTION_SOUTH)) {
				direction = 1;
			} else if (actionName.equals(CleanupWorld.ACTION_EAST)) {
				direction = 2;
			} else if (actionName.equals(CleanupWorld.ACTION_WEST)) {
				direction = 3;
			} else {
				throw new RuntimeException("ERROR: not a valid direction for " + actionName);
			}
			return direction;
		}
		
		protected int actionDir(Action a){
			return actionDir(a.actionName());
		}
		
	}