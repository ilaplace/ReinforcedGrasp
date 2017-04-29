import burlap.mdp.core.action.Action;
import burlap.mdp.core.oo.propositional.PropositionalFunction;
import burlap.mdp.core.oo.state.OOState;
import burlap.mdp.core.state.*;
import burlap.mdp.core.state.State;
import burlap.mdp.singleagent.model.RewardFunction;
import burlap.mdp.singleagent.oo.OOSADomain;

/**
 * Created by ilaplace on 4/23/17.
 */
public class GraspRF implements RewardFunction {


    public double goalReward = 1000.0;

    //TODO Not implemented
    public double collisionReward = -100.0;

    public double defaultReward = -1.0;

    protected PropositionalFunction graspSucces;

    public GraspRF(OOSADomain domain) {

        this.graspSucces = domain.propFunction(Domain.PF_SUCCES);

    }

    @Override
    public double reward(State s, Action a, State sprime) {

        //TODO CHECK THIS
        if (graspSucces.someGroundingIsTrue((OOState)sprime)){
            return goalReward;
        }
        return defaultReward;
    }
}
