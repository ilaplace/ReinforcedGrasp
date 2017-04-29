import burlap.mdp.core.TerminalFunction;
import burlap.mdp.core.oo.propositional.PropositionalFunction;
import burlap.mdp.core.oo.state.OOState;
import burlap.mdp.core.state.*;
import burlap.mdp.singleagent.oo.OOSADomain;

/**
 * Created by ilaplace on 4/23/17.
 */
public class GraspTF implements TerminalFunction {

    private PropositionalFunction graspSucces;

    public GraspTF(OOSADomain domain) {
        this.graspSucces = domain.propFunction(Domain.PF_SUCCES);
    }

    @Override
    public boolean isTerminal(burlap.mdp.core.state.State state) {
        return this.graspSucces.someGroundingIsTrue((OOState)state);
    }
}
