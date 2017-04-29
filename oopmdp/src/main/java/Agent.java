import burlap.mdp.core.oo.state.ObjectInstance;
import burlap.mdp.core.state.*;
import sun.reflect.generics.reflectiveObjects.NotImplementedException;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Created by semih on 14.04.2017.
 */
public class Agent implements ObjectInstance, MutableState{

    private static List<String> keys = Arrays.asList(Domain.variables);
    public String name = "agent";

    public double[] p_values = new double[Domain.DOF];

    public Agent() {
    }

    public Agent(double[] values) {
        this.p_values = values;
    }

    public Agent(double[] values, String name) {
            this.p_values = values;
             this.name = name;
 }

    @Override
    public MutableState set(Object variableKey, Object value) {
        for (int i = 0; i <this.p_values.length ; i++) {
            if (variableKey.toString().equals(Domain.variables[i]))
                this.p_values[i] = new Double(value.toString());
        }
        return this;
    }

    @Override
    public List<Object> variableKeys() {
        List<Object> keys_obje = new ArrayList<Object>(Domain.DOF);
        for (int i = 0; i < Domain.DOF; i++) {
            keys_obje.add(i, (Object) keys.get(i));
        }
        return null;
    }

    @Override
    public Object get(Object variableKey) {
        for (int i = 0; i < Domain.DOF; i++) {
            if (variableKey.equals(Domain.variables[i]))
                return this.p_values[i];
        }
        throw new UnknownKeyException(variableKey);
    }
    @Override
    public Agent copy(){
        return new Agent();
}
    @Override
    public ObjectInstance copyWithName(String objectName) {
        return new Agent(p_values, objectName);
    }

    @Override
    public String className() {
        return Domain.CLASS_AGENT;
    }

    @Override
    public String name() {
        return name;
    }

    public static void main(String[] args) {

        Object obj = 2.00002;
        Float as = new Float(obj.toString());
        System.out.print(as);
        Agent a = new Agent();

    }
}
