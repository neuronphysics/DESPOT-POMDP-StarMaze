#include "starmaze.h"

#include <despot/solver/pomcp.h>
#include <despot/core/builtin_lower_bounds.h>
#include <despot/core/builtin_policy.h>
#include <despot/core/builtin_upper_bounds.h>
#include <despot/core/particle_belief.h>

using namespace std;

namespace despot {
/* =============================================================================
* SimpleState class
* =============================================================================*/
StarMazeProblem* StarMazeProblem::current_ = NULL;
SimpleState::SimpleState() {
}


SimpleState::SimpleState(int _state_id) {
	state_id = _state_id;
}

SimpleState::~SimpleState() {
}

string SimpleState::text() const {
          return "id=" + to_string(state_id);
}

/* =============================================================================
* StarMaze class
* =============================================================================*/


StarMazeProblem::StarMazeProblem() {
	current_ = this;
	Init();
}
void StarMazeProblem::Init() {


	SimpleState* state;
	states_.resize(NumStates());
	pos_.resize(NumStates());
	cont_.resize(NumStates());
    tim_.resize(NumStates());
	for (int position = 0; position < MAZEPOSITIONS; position++) {
		for (int context = 0; context < CONTEXTTYPE; context++) {
            for (int time=0, time<TOTALTIME; time++){
			    int s = PosConTimIndicesToStateIndex(context, position, time);
			    state      = new SimpleState(s);
			    states_[s] = state;
	            pos_[s]    = position;
			    cont_[s]   = context;
                tim_[s]    = time;
            }
		}
	}

	// Build transition matrix
	transition_probabilities_.resize(NumStates());
	for (int s = 0; s < NumStates(); s++) {
		transition_probabilities_[s].resize(NumActions());

		for (int a = 0; a < NumActions(); a++) {
            State next;
            next.state_id = PosConTimIndicesToStateIndex(cont_[s], a, StateIndexToTimIndex(s) + 1);
            if (pos_[s]==CENTER ){
                if (a!=A_TOPLEFT2 || a!=A_TOPRIGHT2){
                    next.weight =0.86;
                    //if the rat is at the center and if she doesn't take topleft2 or topright2 actions then the probability of transition is 0.86   
                }else {
                    next.weight =0.02;
                }
            }else if ( pos_[s]==CUE){
                if (a!=A_TOPLEFT2 || a!=A_TOPRIGHT2 || a!=A_CENTER){
                    //if the rat is at the center and if she doesn't take walk back to the centter or topleft2 or topright2 actions then the probability of transition is 0.86   
                    next.weight =0.86;
                }else {
                    next.weight =0.02;
                }
            }else if(pos_[s]==RIGHT){
                if (a=A_RIGHT){
                    //if the rat is at the right, the only likely transition is to stay  
                    next.weight =0.86;
                }else {
                    next.weight =0.02;
                }   
            }else if(pos_[s]==LEFT){
                if (a=A_LEFT){
                    //if the rat is at the left arm then the most likely transition will be to stay
                    next.weight =0.86;
                }else {
                    next.weight =0.02;
                }   
            }else if(pos_[s]==TOPRIGHT1){
                if (a=A_TOPRIGHT2){
                    //if the rat is at the topright1, the only likely transition is to go to the topright2  
                    next.weight =0.86;
                }else {
                    next.weight =0.02;
                }   
            }else if(pos_[s]==TOPRIGHT2){
                if (a=A_TOPRIGHT2){
                    //if the rat is at the topright2, the only likely transition is to stay  
                    next.weight =0.86;
                }else {
                    next.weight =0.02;
                }   
            }else if(pos_[s]==TOPLEFT1){
                if (a=A_TOPLEFT2){
                    //if the rat is at the topleft1 arm then the most likely to go to topleft2
                    next.weight =0.86;
                }else {
                    next.weight =0.02;
                }   
            }else if(pos_[s]==TOPLEFT2){
                if (a=A_TOPLEFT2){
                    //if the rat is at the toplef2t arm then the most likely transition will be to stay
                    next.weight =0.86;
                }else {
                    next.weight =0.02;
                }   
            }
            transition_probabilities_[s][a].push_back(next);
			
		}
	}
}
StarMazeProblem::~StarMazeProblem() {

}
/* =============================================================================
* OptimalStarMazePolicy class
* =============================================================================*/

class OptimalStarMazePolicy: public DefaultPolicy {
private:
        const StarMazeProblem* Starmaze_;
public:
        
        OptimalStarMazePolicy(const DSPOMDP* model,
               ParticleLowerBound* bound) :
               DefaultPolicy(model, bound) {
        }

        // NOTE: optimal action
        ACT_TYPE Action(const vector<State*>& particles, RandomStreams& streams,
                        History& history) const {

            return StarMazeProblem::A_CUE;
        }
};


/* ==============================
 * Deterministic simulative model
 * ==============================*/

bool StarMazeProblem::Step(State& state, double rand_num, ACT_TYPE action,
        double& reward, OBS_TYPE& obs) const {
    SimpleState& simple_state = static_cast < SimpleState& >(state);
    reward = 0.0;
    obs    = O_NONE;
    bool terminal = false;
    if (tim_[simple_state.state_id]==TIME_STEP_3){
        if ( cont_[simple_state.state_id]==C_RIGHT){
            if (action!=A_RIGHT){
               reward = -20;
            }else {
               obs    = O_RIGHT;
            }  
        }else if(action==A_LEFT && cont_[simple_state.state_id]==C_LEFT){
            reward = 10;
            obs    = O_LEFT;
        }else if ( cont_[simple_state.state_id]==C_TOPRIGHT){
            if (action!=A_TOPRIGHT2){
               reward = -20;
            }else {
               obs    = O_TOPRIGHT;
            }
        }else if(action==A_TOPLEFT2 && cont_[simple_state.state_id]==C_TOPLEFT){
            reward = 20;
            obs    = O_TOPLEFT;
        }
        terminal = true;
        return terminal;
    }else{
    
      const vector<State>& distribution =
		transition_probabilities_[state.state_id][action];
	double sum = 0;
	for (int i = 0; i < distribution.size(); i++) {
		const State& next = distribution[i];
		sum += next.weight;
		if (sum >= random_num) {
			state.state_id = next.state_id;
			break;
		}
	}

      return terminal;
    }
}

//**


int StarMazeProblem::NumStates() const {
	 return CONTEXTTYPE * MAZEPOSITIONS * TOTALTIME;
}


double StarMazeProblem::ObsProb(OBS_TYPE obs, const State& state,
    ACT_TYPE action) const {
      const SimpleState& simple_state = static_cast < const SimpleState& >(state);
      
      if (action == A_CUE) {
          // when the rat at CUE, its observation is correct with probability 0.98
          return (obs == cont_[simple_state.state_id]+1) ? 0.96 : 0.04;
          //because he first observation is none and the rest is similar to the context
      }else if (action == A_LEFT && cont_[simple_state.state_id] == C_LEFT){      
          return (obs==O_LEFT);
      }else if (action == A_TOPLEFT2 && cont_[simple_state.state_id] == C_TOPLEFT){
          return (obs==O_TOPLEFT);
       }else if (action == A_RIGHT && cont_[simple_state.state_id] == C_RIGHT){
          return (obs==O_RIGHT);
      }else if (action == A_TOPRIGHT2 && cont_[simple_state.state_id] == C_TOPRIGHT){
          return (obs==O_TOPRIGHT);//return 1 if correct and 0 otherwise
      }else{
          // when the actions are not A_CUE, the rat does not receive any observations unless rat moves to the arm with the same context then he receives the correct observation.
          // assume it receives a default observation with probability 1 which is none.
          return obs == O_NONE;
      }
}
/* ================================================
*      Randomly choosing k number from 0 to N
*  ===============================================*/
std::vector<int> pick(int N, int k) {
    std::random_device rd;
    std::mt19937 gen(rd());

    std::unordered_set<int> elems = pickSet(N, k, gen);

    // ok, now we have a set of k elements. but now
    // it's in a [unknown] deterministic order.
    // so we have to shuffle it:

    std::vector<int> result(elems.begin(), elems.end());
    std::shuffle(result.begin(), result.end(), gen);
    return result;
}
std::unordered_set<int> pickSet(int N, int k, std::mt19937& gen)
{
    std::uniform_int_distribution<> dis(1, N);
    std::unordered_set<int> elems;

    while (elems.size() < k) {
        elems.insert(dis(gen));
    }

    return elems;
}
/* ================================================
* Functions related to beliefs and starting states
* ================================================*/

State* StarMazeProblem::CreateStartState(string type) const {
  // Always rat starts at the center at time 0 with uniform belief about the context
   std::vector<int> context = pick(CONTEXTTYPE, 1);
   int s = PosConTimIndicesToStateIndex(context.front(),RAT_POSITION::CENTER,TIME_STEP::TIME_STEP_1);
   return new SimpleState(s);//????
}

Belief* StarMazeProblem::InitialBelief(const State* start, string type) const {
        
        if (type == "DEFAULT" || type == "PARTICLE") {
            vector<State*> particles;
            for (int cont = 0; cont!=CONTEXTTYPE; ++cont) {
                CONTEXT eCurrent = (CONTEXT)cont;
                int s = PosConTimIndicesToStateIndex(CENTER, cont, TIME_STEP_1);
                //Allocate() function allocates some space for creating new state;
                SimpleState* InitialState = static_cast<SimpleState*>(Allocate(states_[s], 0.25));
                particles.push_back(InitialState);
            }
            return new ParticleBelief(particles, this);
        } else {
            cerr << "Unsupported belief type: " << type << endl;
            exit(1);
        }
}
/* ========================
* Bound-related functions.
* ========================*/
double StarMazeProblem::Reward(int s, ACT_TYPE action) const {
    const SimpleState* simple_state = states_[s];
	double reward = 0;
    
	if (action == A_LEFT)  {
        if ( cont_[simple_state->state_id] ==C_LEFT && (pos_[simple_state->state_id]==CENTER || pos_[simple_state->state_id]==CUE)){
		   reward = 10;
        }
	} else if  (action == A_TOPLEFT2 ){
        if (cont_[simple_state->state_id]==C_TOPLEFT&& pos_[simple_state->state_id]==TOPLEFT1){
		   reward = 20;
        }
	} else if  (action != A_TOPRIGHT2){
        if (tim_[simple_state->state_id] == TIME_STEP_4 && cont_[simple_state->state_id] ==C_TOPRIGHT){
		   reward = -20;
        }
	}else if  (action != A_RIGHT){
         if (tim_[simple_state->state_id] == TIME_STEP_4 && cont_[simple_state->state_id] ==C_RIGHT){
		    reward = -20;
         }
	}
	return reward;
}


double StarMazeProblem::GetMaxReward() const {
       return 20;
}

ValuedAction StarMazeProblem::GetBestAction() const {
		return ValuedAction(A_CUE, 0);
}
/*problematic parts*/
void StarMazeProblem::ComputeDefaultActions(string type) const {
	cerr << "Default action = " << type << endl;
	if (type == "MDP") {
		const_cast<StarMazeProblem*>(this)->ComputeOptimalPolicyUsingVI();
		int num_states = NumStates();
		default_action_.resize(num_states);

		double value = 0;//it seems this variable is redundant 
		for (int s = 0; s < num_states; s++) {
			default_action_[s] = policy_[s].action;
			value += policy_[s].value;
		}
	} else {
		cerr << "Unsupported default action type " << type << endl;
		exit(0);
	}
}

//????
const vector<State>& StarMazeProblem::TransitionProbability(int s, ACT_TYPE a) const {
	return transition_probabilities_[s][a];
}
//????what is this function doing?? 
Belief* StarMazeProblem::Tau(const Belief* belief, ACT_TYPE action,
	OBS_TYPE obs) const {
       
	static vector<double> probs = vector<double>(NumStates());

	const vector<State*>& particles =
		static_cast<const ParticleBelief*>(belief)->particles();
  //********************

	double sum = 0;
	for (int i = 0; i < particles.size(); i++) {
		
		SimpleState* state = static_cast<SimpleState*>(particles[i]);

        const vector<State>& distribution = transition_probabilities_[GetIndex(
			state)][action];

        for (int j = 0; j < distribution.size(); j++) {
		    const State& next = distribution[j];

		    double p = state->weight * next.weight*ObsProb(obs, next, action);
		    probs[next.state_id] += p;
		    sum += p;
        }
	}
  //******************
	vector<State*> new_particles;
	for (int i = 0; i < NumStates(); i++) {
		if (probs[i] > 0) {
			State* new_particle = Copy(states_[i]);
			new_particle->weight = probs[i] / sum;
			new_particles.push_back(new_particle);
			probs[i] = 0;
		}
	}

	if (new_particles.size() == 0) {
		cout << *belief << endl;
		exit(0);
	}

	return new ParticleBelief(new_particles, this, NULL, false);
}


/*=====================================*
 *     Bound
 *=====================================*/

ParticleUpperBound* StarMazeProblem::CreateParticleUpperBound(string name) const {
        if (name == "TRIVIAL" ) {
            return new TrivialParticleUpperBound(this);
        } else if (name == "MDP"|| name == "DEFAULT") {
            return new MDPUpperBound(this, *this);
        } else {
            cerr << "Unsupported particle lower bound: " << name << endl;
            exit(1);
            return NULL;
        }
    }
ScenarioUpperBound* StarMazeProblem::CreateScenarioUpperBound(string name,
        string particle_bound_name) const {
        
        const StateIndexer* indexer = this;
        if (name == "TRIVIAL" || name == "DEFAULT") {
            return new TrivialParticleUpperBound(this);
        } else if (name == "LOOKAHEAD") {
            return new LookaheadUpperBound(this, *this,
                 CreateParticleUpperBound(particle_bound_name));

        } else if (name == "MDP") {
            return new MDPUpperBound(this, *indexer);
        } else {
            cerr << "Unsupported base upper bound: " << name << endl;
            exit(0);
        }
        
}
ScenarioLowerBound* StarMazeProblem::CreateScenarioLowerBound(string name,
                                     string particle_bound_name="DEFAULT") const {
        const DSPOMDP* model = this;
	    const StateIndexer* indexer = this;
	    const StatePolicy* policy = this;                                 
        ScenarioLowerBound* bound = NULL;
        cout<<"here lower bound..."<<endl;
        if (name == "TRIVIAL" ) {
            bound = new TrivialParticleLowerBound(this);
        } else if (name == "RANDOM") {
            bound = new RandomPolicy(this,
                          CreateParticleLowerBound(particle_bound_name));
        } else if (name == "MODE" || name == "DEFAULT") {
		    ComputeDefaultActions("MDP");
		     bound = new ModeStatePolicy(model, *indexer, *policy,
			              CreateParticleLowerBound(particle_bound_name));                             
        } else if (name == "CENTER") {
            bound = new BlindPolicy(this, CENTER,
                                    CreateParticleLowerBound(particle_bound_name));
        } else if (name == "CUE") {
            bound = new BlindPolicy(this, CUE,
                                    CreateParticleLowerBound(particle_bound_name));
        } else if (name == "RIGHT") {
            bound = new BlindPolicy(this, RIGHT,
                                    CreateParticleLowerBound(particle_bound_name));
        } else if (name == "LEFT") {
            bound = new BlindPolicy(this, LEFT,
                                    CreateParticleLowerBound(particle_bound_name));
        } else if (name == "TOPRIGHT1") {
            bound = new BlindPolicy(this, TOPRIGHT1,
                                    CreateParticleLowerBound(particle_bound_name));
        } else if (name == "TOPRIGHT2") {
            bound = new BlindPolicy(this, TOPRIGHT2,
                                    CreateParticleLowerBound(particle_bound_name));
        } else if (name == "TOPLEFT1") {
            bound = new BlindPolicy(this, TOPLEFT1,
                                    CreateParticleLowerBound(particle_bound_name));
        } else if (name == "TOPLEFT2") {
            bound = new BlindPolicy(this, TOPLEFT2,
                                    CreateParticleLowerBound(particle_bound_name));
        } else if (name == "OPTIMAL") {
            bound = new OptimalStarMazePolicy(this,
                                              CreateParticleLowerBound(particle_bound_name));
        } else {
            cerr << "Unsupported scenario lower bound: " << name << endl;
            exit(1);
        }
        return bound;
}
/*end of uncertain parts*/
/* ============================================
 *  print different elemens of the POMDP model
 * ============================================*/
void StarMazeProblem::PrintState(const State& state, ostream& out) const {
        const SimpleState& simple_state = static_cast<const SimpleState&>(state);

        out << "Rat = " << simple_state.rat_position << "; Context = "
        << simple_state.context << "; time = " << simple_state.time << endl;
}

void StarMazeProblem::PrintBelief(const Belief& belief, ostream& out) const {
     const vector<State*>& particles =
     static_cast<const ParticleBelief&>(belief).particles();
     vector<double> pos_probs(8);
     vector<double> context_probs(4);
     vector<double> time_probs(4);
     for (int i = 0; i < particles.size(); i++) {
            State* particle = particles[i];
            const SimpleState* state = static_cast<const SimpleState*>(particle);
            context_probs[state->context] += particle->weight;
            pos_probs[state->rat_position] += particle->weight;
            time_probs[state->time] += particle->weight;
     }

     out << "Rat belief about the context: " << " LEFT" << ":" << context_probs[0]<<
        " TOPLEFT" << ":" << context_probs[1] << " RIGHT" << ":" << context_probs[2]<<
        " TOPRIGHT" << ":" << context_probs[3] << endl;

     out << "Position belief:" << " CENTER" << ":" << pos_probs[0] << " CUE" << ":" <<
        pos_probs[1] << " RIGHT" << ":" << pos_probs[2] << " LEFT" << ":" << pos_probs[3]
        <<" TOPRIGHT1 " << ":" << pos_probs[4]<< " TOPRIGHT2"<< ":" << pos_probs[5] <<
        " TOPLEFT1" << ":" << pos_probs[6] << " TOPLEFT2" << ":" << pos_probs[7]<< endl;

     out<< "Time step belief: "<< "TIME_STEP_1" <<":"<< time_probs[0] << "TIME_STEP_2"
        <<":"<< time_probs[1] << "TIME_STEP_3" <<":"<< time_probs[2] <<"TIME_STEP_4" << ":"
        <<time_probs[3]<<endl;
}

void StarMazeProblem::PrintObs(const State& state, OBS_TYPE obs, ostream& out) const {
        switch (obs) {
            case O_NONE:
                out << "None" << endl;
                break;
            case O_LEFT:
                out << "small reward at left arm" << endl;
                break;
            case O_TOPLEFT:
                out << "larg reward at the end of top-left arm" << endl;
                break;
            case O_RIGHT:
                out << "Shock if at the last time step rat doesn't reach right arm" << endl;
                break;
            case O_TOPRIGHT:
                out << "Shock if at the last time step rat doesn't reach to the end of top-right arm" << endl;
                break;
        }
}


void StarMazeProblem::PrintAction(ACT_TYPE action, ostream& out) const {
        if (action == A_LEFT) {
            cout << "Move left arm" << endl;
        } else if (action == A_RIGHT) {
            cout << "Move right arm" << endl;
        } else if (action == A_TOPLEFT1) {
            cout << "Move top-left 1 position" << endl;
        } else if (action == A_TOPRIGHT1) {
            cout << "Move top right 1 position" << endl;
        } else if (action == A_TOPLEFT2) {
            cout << "Move top-left 2 position" << endl;
        } else if (action == A_TOPRIGHT2) {
            cout << "Move top right 2 position" << endl;
        } else if (action == A_CENTER){
            cout<< "Move to center" << endl;
        }else {
            cout << "check the cue" << endl;
        }
}

/* =================
 * Memory management
 * =================*/

State* StarMazeProblem::Allocate(int state_id, double weight) const {
        
        SimpleState* state = memory_pool_.Allocate();
        state->state_id = state_id;
        state->weight = weight;
        return state;
}

State* StarMazeProblem::Copy(const State* particle) const {
        SimpleState* state = memory_pool_.Allocate();
        *state = *static_cast<const SimpleState*>(particle);
        state->SetAllocated();
        return state;
}
void StarMazeProblem::Free(State* particle) const {
        memory_pool_.Free(static_cast<SimpleState*>(particle));
}
int StarMazeProblem::NumActiveParticles() const {
	return memory_pool_.num_allocated();
}

int StarMazeProblem::GetAction(const State& state) const {
	return default_action_[GetIndex(&state)];
}



}// namespace despot
