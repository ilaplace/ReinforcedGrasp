import burlap.mdp.core.oo.state.OOStateUtilities;
import burlap.mdp.core.oo.state.ObjectInstance;
import burlap.mdp.core.state.*;
import burlap.mdp.core.state.annotations.DeepCopyState;

import java.util.Arrays;
import java.util.List;

/**
 * Created by ilaplace on 4/19/17.
 */
public abstract class Objects implements ObjectInstance{

    public double pose_x;
    public double pose_y;
    public double pose_z;

    public double orientation_x;
    public double orientation_y;
    public double orientation_z;
    public double orientation_w;

    protected String name;
    private static final List<Object> keys = Arrays.<Object>asList(Domain.VAR_OBJECT_POSE_X, Domain.VAR_OBJECT_POSE_Y,
            Domain.VAR_OBJECT_POSE_Z, Domain.VAR_OBJECT_ORIENTATION_X,Domain.VAR_OBJECT_ORIENTATION_Y,
            Domain.VAR_OBJECT_ORIENTATION_Z, Domain.VAR_OBJECT_ORIENTATION_W);

    public Objects() {
    }

    public Objects(double pose_x, double pose_y, double pose_z, double orientation_x,
                       double orientation_y, double orientation_z, double orientation_w, String name) {
        this.pose_x = pose_x;
        this.pose_y = pose_y;
        this.pose_z = pose_z;
        this.orientation_x = orientation_x;
        this.orientation_y = orientation_y;
        this.orientation_z = orientation_z;
        this.orientation_w = orientation_w;
        this.name = name;
    }


    @Override
    public List<Object> variableKeys() {
        return keys;
    }

    @Override
    public Object get(Object variableKey) {

        if(variableKey.equals(Domain.VAR_OBJECT_POSE_X)){
            return pose_x ;
        }
        else if(variableKey.equals(Domain.VAR_OBJECT_POSE_Y)){
            return pose_y;
        }
        else if(variableKey.equals(Domain.VAR_OBJECT_POSE_Z)){
            return pose_z;
        }
        else if(variableKey.equals(Domain.VAR_OBJECT_ORIENTATION_W)){
            return orientation_w;
        }
        else if(variableKey.equals(Domain.VAR_OBJECT_ORIENTATION_X)){
            return orientation_x;
        }
        else if(variableKey.equals(Domain.VAR_OBJECT_ORIENTATION_Y)){
            return orientation_y;
        }
        else if(variableKey.equals(Domain.VAR_OBJECT_ORIENTATION_Z)){
            return orientation_z;
        }
        throw new UnknownKeyException(variableKey);
    }
    @Override
    abstract public Objects copy();

    @Override
    public String name() {
        return name;
    }

    @Override
    public ObjectInstance copyWithName(String s) {
        Objects obje = this.copy();
        obje.name = s;
        return obje;
    }
    @Override
    public String toString() {
        return OOStateUtilities.objectInstanceToString(this);
    }

    @DeepCopyState
    public static class Hand extends Objects{

        public Hand() {
        }

        public Hand(double pose_x, double pose_y, double pose_z, double orientation_x, double orientation_y, double orientation_z, double orientation_w, String name) {
            super(pose_x, pose_y, pose_z, orientation_x, orientation_y, orientation_z, orientation_w, name);
        }

        @Override
        public String className(){
            return Domain.HAND_CLASS;
        }
        @Override
        public Hand copy(){
            return new Hand(pose_x, pose_y, pose_z, orientation_x, orientation_y, orientation_z, orientation_w, name);

        }
    }

    @DeepCopyState
    public static class GraspableObject extends Objects{

        public GraspableObject() {
        }

        public GraspableObject(double pose_x, double pose_y, double pose_z, double orientation_x, double orientation_y, double orientation_z, double orientation_w, String name) {
            super(pose_x, pose_y, pose_z, orientation_x, orientation_y, orientation_z, orientation_w, name);
        }

        @Override
        public String className(){
            return Domain.GRASPABLE_OBJECT_CLASS;
        }
        @Override
        public GraspableObject copy(){
            return new GraspableObject(pose_x, pose_y, pose_z, orientation_x, orientation_y, orientation_z, orientation_w, name);

        }
    }
}
