import burlap.mdp.core.oo.state.MutableOOState;
import burlap.mdp.core.oo.state.ObjectInstance;
import burlap.mdp.core.oo.state.exceptions.UnknownClassException;
import burlap.mdp.core.state.MutableState;

import java.util.List;

/**
 * Created by semih on 14.04.2017.
 */
public class State implements MutableOOState {

    public Agent agent;

    public State() {

    }
    public State(Agent agent){
        this.agent = agent;
    }

    public MutableOOState addObject(ObjectInstance objectInstance) {
        if (objectInstance instanceof Agent){
            agent = (Agent) objectInstance;
        }
        else {
            throw new UnknownClassException(objectInstance.className());
        }
        return this;
    }

    public MutableOOState removeObject(String oName) {
        if (agent.name().equals(oName)){
            agent = new Agent();
        }
        return this;
    }

    public MutableOOState renameObject(String s, String s1) {
        return null;
    }

    public int numObjects() {
        return 0;
    }

    public ObjectInstance object(String s) {
        return null;
    }

    public List<ObjectInstance> objects() {
        return null;
    }

    public List<ObjectInstance> objectsOfClass(String s) {
        return null;
    }

    public MutableState set(Object o, Object o1) {
        return null;
    }

    public List<Object> variableKeys() {
        return null;
    }

    public Object get(Object o) {
        return null;
    }

    public burlap.mdp.core.state.State copy() {
        return null;
    }
}
