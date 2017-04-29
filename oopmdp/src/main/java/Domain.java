import burlap.mdp.auxiliary.DomainGenerator;
import burlap.mdp.core.TerminalFunction;
import burlap.mdp.core.action.UniversalActionType;
import burlap.mdp.core.oo.propositional.PropositionalFunction;
import burlap.mdp.core.oo.state.OOState;
import burlap.mdp.core.oo.state.ObjectInstance;
import burlap.mdp.singleagent.model.RewardFunction;
import burlap.mdp.singleagent.oo.OOSADomain;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Created by semih on 14.04.2017.
 */
public class Domain implements DomainGenerator {

    public static final Integer DOF = 15;
    public static final String HAND_CLASS = "hand";
    public static final String GRASPABLE_OBJECT_CLASS = "grasp_object";
    public static final String QUALITY = "quality";

    public static String[] actionMin = new String[DOF];
    public static String[] actionPlus = new String[DOF];
    public static String[] variables = new String[DOF];


    //Object
    public static String VAR_OBJECT_POSE_X;
    public static String VAR_OBJECT_POSE_Y;
    public static String VAR_OBJECT_POSE_Z;
    public static String VAR_OBJECT_ORIENTATION_X;
    public static String VAR_OBJECT_ORIENTATION_Y;
    public static String VAR_OBJECT_ORIENTATION_Z;
    public static String VAR_OBJECT_ORIENTATION_W;
    //Hand
    public static String VAR_HAND_POSE_X;
    public static String VAR_HAND_POSE_Y;
    public static String VAR_HAND_POSE_Z;
    public static String VAR_HAND_ORIENTATION_X;
    public static String VAR_HAND_ORIENTATION_Y;
    public static String VAR_HAND_ORIENTATION_Z;
    public static String VAR_HAND_ORIENTATION_W;


    public static final String VAR_TYPE = "type";
    public static final String CLASS_AGENT = "agent";
    public static String PF_CONTACT = "contactOccured";
    public static String PF_SUCCES = "succesfullGrasp";



    public List<PropositionalFunction> generatePfs(){
        //TODO Implement pf's
        return Arrays.<PropositionalFunction>asList(new GraspedPF(PF_SUCCES));
    }

    protected List <Double>	thrustValues;
    protected RewardFunction rf;
    protected TerminalFunction tf;
    protected RobotParams robotParams = new RobotParams();

    public Domain() {

        for(int j = 0; j< actionMin.length; j++){
            actionMin[j] = ("ACTION_P"+ j +"_MIN");
            actionPlus[j] = ("ACTION_P"+ j +"_PLUS");
            variables[j] = ("p" +j);
        }
        thrustValues = new ArrayList<Double>();
    }

    public void addThrustActionWithThrust(double t){
        //TODO Hack is this
        this.thrustValues.add(t);
    }

    public TerminalFunction getTf(){
        return tf;
    }

    public void setTf(TerminalFunction tf){
        this.tf = tf;
    }

    public RewardFunction getRf(){
        return rf;
    }

    public void setTf(RewardFunction rf){
        this.rf = rf;
    }

    public static class RobotParams{

        protected double p_minim = 0.;
        protected double p_max = 2*Math.PI;
        //TODO Check margins for robot

        public double getP_minim() {
            return p_minim;
        }

        public double getP_max() {
            return p_max;
        }

        public void setP_minim(double p_minim) {
            this.p_minim = p_minim;
        }

        public void setP_max(double p_max) {
            this.p_max = p_max;
        }
    }

    @Override
    public OOSADomain generateDomain(){

        OOSADomain domain = new OOSADomain();
        domain.addStateClass(CLASS_AGENT, Agent.class);

        //Actions added to the domain
        for (int i = 0; i < actionPlus.length; i++) {
            domain.addActionTypes(new UniversalActionType(actionMin[i]));
            domain.addActionTypes(new UniversalActionType(actionPlus[i]));
        }

        OOSADomain.Helper.addPfsToDomain(domain, this.generatePfs());

        RewardFunction rf = this.rf;
        TerminalFunction tf = this.tf;
        if (rf== null){
            rf = new GraspRF(domain);
        }

        if (tf== null){
            tf = new GraspTF(domain);
        }

        return domain;
    }


    public class GraspedPF extends PropositionalFunction{

        public GraspedPF(String name) {
            super(name, new String[]{CLASS_AGENT, GRASPABLE_OBJECT_CLASS});
        }

        @Override
        public boolean isTrue(OOState ooState, String... strings) {

            //TODO Check quality
            return false;
        }
    }

    public static void main(String[] args) {
        Domain dm = new Domain();
        Agent ag = new Agent();
        ag.variableKeys();
        System.out.print("sad");

    }
}
