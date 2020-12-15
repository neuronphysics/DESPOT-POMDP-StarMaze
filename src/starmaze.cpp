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

SimpleState::SimpleState() {
}


SimpleState::SimpleState(int _state_id) {
	state_id = _state_id;
}
SimpleState::~SimpleState() {
}

string SimpleState::text() const {
          return "rat position in the maze = " + to_string(rat_position) + " with context = " +
                  to_string(context) + " and time step = " + to_string(time);
}
/* =============================================================================
* OptimalStarMazePolicy class
* =============================================================================*/

class OptimalStarMazePolicy: public DefaultPolicy {
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
/* =============================================================================
* SimpleRockSample class
* =============================================================================*/

StarMazeProblem::StarMazeProblem() {
    
}


/* ==============================
 * Deterministic simulative model
 * ==============================*/

bool StarMazeProblem::Step(State& state, double rand_num, ACT_TYPE action,
        double& reward, OBS_TYPE& obs) const {
    SimpleState& simple_state = static_cast < SimpleState& >(state);
    int& rat_position = simple_state.rat_position;
    int& context = simple_state.context;
    int& time = simple_state.time;
    cout<< "Hey you! I am in step functtion ..."<<endl;
    if (time < 4){
       if ( rat_position == CENTER) {
          time    = time + 1;
          context = context;
          if (action == A_CENTER) {
               obs     = O_NONE;
               reward  = (time==TIME_STEP_4 && (context== C_TOPRIGHT||context== C_RIGHT)) ? -20 : 0;
          } else if (action == A_CUE) {
               rat_position = CUE;
               if (context == C_LEFT){
                  //with the probabiliy of 95%  rat will figure out the context of trial
                  obs =  (rand_num > 0.02) ? O_LEFT : O_NONE;
                  //obs    = O_LEFT;
               } else if (context== C_TOPLEFT){
                  obs =  (rand_num > 0.02) ? O_TOPLEFT : O_NONE;
                  //obs    = O_TOPLEFT;
               } else if (context== C_RIGHT){
                  obs =  (rand_num > 0.02) ? O_RIGHT : O_NONE;
                  //obs   = O_RIGHT;
               } else {
                  obs =  (rand_num > 0.02) ? O_TOPRIGHT : O_NONE;
                  //obs = O_TOPRIGHT;
               }
               reward = (time==TIME_STEP_4 && (context== C_TOPRIGHT||context== C_RIGHT)) ? -20 : 0;
          } else if (action == A_RIGHT) {
               obs     = O_NONE;
               rat_position = RIGHT;
               if (context== C_RIGHT){
                  obs = O_RIGHT;
               }
               reward = (time==TIME_STEP_4 && context== C_TOPRIGHT) ? -20 : 0;
          } else if (action == A_LEFT){
               obs     = O_NONE;
               rat_position = LEFT;
               if (context== C_LEFT){
                  obs    = O_LEFT;
                  reward = 10;
               }else{
                  reward = (time==TIME_STEP_4 && (context== C_TOPRIGHT||context== C_RIGHT)) ? -20 : 0;
               }
          } else if (action==A_TOPRIGHT1 || action==A_TOPLEFT1){
              if (action==A_TOPLEFT1){
                 rat_position = TOPLEFT1;
              }else{
                 rat_position = TOPRIGHT1;
              }
              time    = time + 1;
              context = context; //context stays the same through each trial
              obs     = O_NONE;
              reward = (time==TIME_STEP_4 && (context== C_TOPRIGHT||context== C_RIGHT)) ? -20 : 0;
          }
      } else if (rat_position == CUE) {
          time    = time + 1;
          context = context; //context stays the same through each trial
          if (action == A_CUE) {
             reward  = (time==TIME_STEP_4 && (context== C_TOPRIGHT||context== C_RIGHT)) ? -20 : 0;
          } else if (action == A_RIGHT) {
              rat_position = RIGHT;
              reward = (time==TIME_STEP_4 && context== C_TOPRIGHT) ? -20 : 0;
          } else if (action==A_LEFT){
              rat_position = LEFT;
              if (context== C_LEFT){
                 reward =10;
              }else {
                 reward = (time==TIME_STEP_4 && (context== C_TOPRIGHT||context== C_RIGHT)) ? -20 : 0;
              }
          } else if (action==A_TOPRIGHT1 || action==A_TOPLEFT1){
              time    = time + 1;
              context = context; //context stays the same through each trial
              if (action==A_TOPLEFT1){
                 rat_position = TOPLEFT1;
              }else{
                 rat_position = TOPRIGHT1;
              }
              reward = (time==TIME_STEP_4 && (context== C_TOPRIGHT||context== C_RIGHT)) ? -20 : 0;
          }
      } else if (rat_position == RIGHT) {
          if (action == A_RIGHT) {
              time    = time + 1;
              context = context; //context stays the same through each trial
              reward  = (time==TIME_STEP_4 && context== C_TOPRIGHT) ? -20 : 0;
          }
      } else if (rat_position == LEFT){
          if (action == A_LEFT) {
             time    = time + 1;
             context = context; //context stays the same through each trial
             reward  = (time==TIME_STEP_4 && (context== C_TOPRIGHT||context== C_RIGHT)) ? -20 : 0;
          }
      } else if (rat_position==TOPRIGHT1){
          if (action == A_TOPRIGHT2) {
             time    = time + 1;
             context = context; //context stays the same through each trial
             rat_position = TOPRIGHT2;
             if (context== C_TOPRIGHT){
                obs = O_TOPRIGHT;
             }
             reward = (time==TIME_STEP_4 && context== C_RIGHT) ? -20 : 0;
          }
      } else if (rat_position==TOPRIGHT2){
         if (action == A_TOPRIGHT2) {
            time    = time + 1;
            context = context; //context stays the same through each trial
            reward  = (time==TIME_STEP_4 && context== C_RIGHT) ? -20 : 0;
         }
      } else if (rat_position==TOPLEFT1){
         if (action == A_TOPLEFT2){
            time    = time + 1;
            context = context; //context stays the same through each trial
            rat_position=TOPLEFT2;
            if (context== C_TOPLEFT){
               obs    = O_TOPLEFT;
               reward = 20;
            }else {
               reward = (time==TIME_STEP_4 && (context== C_TOPRIGHT||context== C_RIGHT)) ? -20 : 0;
            }
         }
      } else if (rat_position==TOPLEFT2){
        if (action==A_TOPLEFT2){
           time    = time + 1;
           context = context; //context stays the same through each trial
           reward  = (time==TIME_STEP_4 && (context== C_TOPRIGHT||context== C_RIGHT)) ? -20 : 0;
        }
      }
      return false;
    }else{
       // The trial finishes at time step greater than 4
       return true;
    }
}

//**


int StarMazeProblem::NumStates() const {
	 return CONTEXTTYPE * MAZEPOSITIONS * TOTALTIME;
}
int StarMazeProblem::NumActions() const {
    return 8;
}

/* ================================================
* Functions related to beliefs and starting states
* ================================================*/

double StarMazeProblem::ObsProb(OBS_TYPE obs, const State& state,
    ACT_TYPE action) const {
      const SimpleState& simple_state = static_cast < const SimpleState& >(state);
      int rat_position = simple_state.rat_position;
      int context = simple_state.context;
      
      if (action == A_CUE) {
          // when the rat at CUE, its observation is correct with probability 0.98
          return (obs == context-1) ? 0.98 : 0.02;
          //because he first observation is none and the rest is similar to the context
      }else if (action == A_LEFT && context== C_LEFT){      
            return (obs==O_LEFT);
      }else if (action == A_TOPLEFT2 && context== C_TOPLEFT){
            return (obs==O_TOPLEFT);
       }else if (action == A_RIGHT && context== C_RIGHT){
             return (obs==O_RIGHT);
      }else if (action == A_TOPRIGHT2 && context== C_TOPRIGHT){
             return (obs==O_TOPRIGHT);//return 1 if correct and 0 otherwise
      }else{
      return obs == O_NONE;
      }
}
/*=====================================*
 *     Bound
 *=====================================*/

State* StarMazeProblem::CreateStartState(string type) const {
  // Always start at the center and time 0 with random belief about the context
   
   return new SimpleState(0, Random::RANDOM.NextInt(4),0);//????
}

Belief* StarMazeProblem::InitialBelief(const State* start, string type) const {
        
        if (type == "DEFAULT" || type == "PARTICLE") {
            vector<State*> particles;
            
            //Allocate() function allocates some space for creating new state;
            SimpleState* left_context = static_cast<SimpleState*>(Allocate(1, 0.5));
            left_context->rat_position = CENTER;
            left_context->context      = O_LEFT;//why is it observation not state???
            left_context->time         = TIME_STEP_1;
            particles.push_back(left_context);

            SimpleState* topLeft_context = static_cast<SimpleState*>(Allocate(1, 0.5));
            topLeft_context->rat_position = CENTER;
            topLeft_context->context      = O_TOPLEFT;//why is it observation not state???
            topLeft_context->time         = TIME_STEP_1;
            particles.push_back(topLeft_context);

            SimpleState* right_context = static_cast<SimpleState*>(Allocate(1, 0.5));//first component is state_id, the second one is weight
            right_context->rat_position = CENTER;
            right_context->context      = O_RIGHT;//why is it observation not state???
            right_context->time         = TIME_STEP_1;
            particles.push_back(right_context);

            SimpleState* topRight_context = static_cast<SimpleState*>(Allocate(1, 0.5));
            topRight_context->rat_position = CENTER;
            topRight_context->context      = O_TOPRIGHT;//why is it matched wih observation not state???
            topRight_context->time         = TIME_STEP_1;
            particles.push_back(topRight_context);

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
        if ( cont_[simple_state->state_id] ==C_LEFT){
		   reward = 10;
        }
	} else if  (action == A_TOPLEFT2 ){
        if (cont_[simple_state->state_id]==C_TOPLEFT){
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

		double value = 0;
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


/* ==================================================
 * StarMazeBelief class
 * ==================================================*/
StarMazeBelief::StarMazeBelief(vector<State*> particles, const DSPOMDP* model, Belief* prior ):
		            ParticleBelief(particles, model, prior),
		            Starmaze_(static_cast<const StarMazeProblem*>(model)) {
}

void StarMazeBelief::Update(ACT_TYPE action, OBS_TYPE obs){ // TODO: Not complete yet
		          Belief* updated = Starmaze_->Tau(this, action, obs);

		          for (int i = 0; i < particles_.size(); i++)
			          Starmaze_->Free(particles_[i]);
		          particles_.clear();

		          const vector<State*>& new_particles =
		          	    static_cast<ParticleBelief*>(updated)->particles();
		          for (int i = 0; i < new_particles.size(); i++)
			              particles_.push_back(Starmaze_->Copy(new_particles[i]));

		          delete updated;
}

ParticleUpperBound* StarMazeProblem::CreateParticleUpperBound(string name) const {
        if (name == "TRIVIAL" || name == "DEFAULT") {
            return new TrivialParticleUpperBound(this);
        } else if (name == "MDP") {
            return new MDPUpperBound(this, *this);
        } else {
            cerr << "Unsupported particle lower bound: " << name << endl;
            exit(1);
        }
    }
ScenarioUpperBound* StarMazeProblem::CreateScenarioUpperBound(string name,
        string particle_bound_name) const {
        ScenarioUpperBound* bound = NULL;
        if (name == "TRIVIAL" || name == "DEFAULT") {
            bound = new TrivialParticleUpperBound(this);
        } else if (name == "LOOKAHEAD") {
            return new LookaheadUpperBound(this, *this,
                 CreateParticleUpperBound(particle_bound_name));

        } else if (name == "MDP") {
            return new MDPUpperBound(this, *this);
        } else {
            cerr << "Unsupported base upper bound: " << name << endl;
            exit(0);
        }
        return bound;
}
ScenarioLowerBound* StarMazeProblem::CreateScenarioLowerBound(string name,
                                     string particle_bound_name="DEFAULT") const {
        const DSPOMDP* model = this;
	    const StateIndexer* indexer = this;
	    const StatePolicy* policy = this;                                 
        ScenarioLowerBound* bound = NULL;
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
