import burlap.behavior.functionapproximation.DifferentiableStateActionValue;
import burlap.behavior.functionapproximation.dense.ConcatenatedObjectFeatures;
import burlap.behavior.functionapproximation.dense.DenseCrossProductFeatures;
import burlap.behavior.functionapproximation.dense.NormalizedVariableFeatures;
import burlap.behavior.functionapproximation.dense.NumericVariableFeatures;
import burlap.behavior.functionapproximation.dense.fourier.FourierBasis;
import burlap.behavior.functionapproximation.dense.rbf.DistanceMetric;
import burlap.behavior.functionapproximation.dense.rbf.RBFFeatures;
import burlap.behavior.functionapproximation.dense.rbf.functions.GaussianRBF;
import burlap.behavior.functionapproximation.dense.rbf.metrics.EuclideanDistance;
import burlap.behavior.functionapproximation.sparse.tilecoding.TileCodingFeatures;
import burlap.behavior.functionapproximation.sparse.tilecoding.TilingArrangement;
import burlap.behavior.policy.GreedyQPolicy;
import burlap.behavior.policy.Policy;
import burlap.behavior.policy.PolicyUtils;
import burlap.behavior.singleagent.Episode;
import burlap.behavior.singleagent.auxiliary.EpisodeSequenceVisualizer;
import burlap.behavior.singleagent.auxiliary.gridset.FlatStateGridder;
import burlap.behavior.singleagent.learning.lspi.LSPI;
import burlap.behavior.singleagent.learning.lspi.SARSCollector;
import burlap.behavior.singleagent.learning.lspi.SARSData;
import burlap.behavior.singleagent.learning.tdmethods.vfa.GradientDescentSarsaLam;
import burlap.behavior.singleagent.planning.stochastic.sparsesampling.SparseSampling;
import burlap.domain.singleagent.cartpole.CartPoleVisualizer;
import burlap.domain.singleagent.cartpole.InvertedPendulum;
import burlap.domain.singleagent.cartpole.states.InvertedPendulumState;
import burlap.domain.singleagent.lunarlander.LLVisualizer;
import burlap.domain.singleagent.lunarlander.LunarLanderDomain;
import burlap.domain.singleagent.lunarlander.state.LLAgent;
import burlap.domain.singleagent.lunarlander.state.LLBlock;
import burlap.domain.singleagent.lunarlander.state.LLState;
import burlap.domain.singleagent.mountaincar.MCRandomStateGenerator;
import burlap.domain.singleagent.mountaincar.MCState;
import burlap.domain.singleagent.mountaincar.MountainCar;
import burlap.domain.singleagent.mountaincar.MountainCarVisualizer;
import burlap.mdp.auxiliary.StateGenerator;
import burlap.mdp.core.TerminalFunction;
import burlap.mdp.core.state.State;
import burlap.mdp.core.state.vardomain.VariableDomain;
import burlap.mdp.singleagent.SADomain;
import burlap.mdp.singleagent.common.VisualActionObserver;
import burlap.mdp.singleagent.environment.SimulatedEnvironment;
import burlap.mdp.singleagent.model.RewardFunction;
import burlap.mdp.singleagent.oo.OOSADomain;
import burlap.statehashing.simple.SimpleHashableStateFactory;
import burlap.visualizer.Visualizer;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
/**
 * Created by semih on 12.04.2017.
 */

public class mdp {
    public static void main(String[] args) {


        LunarLanderDomain lld = new LunarLanderDomain();
        OOSADomain domain = lld.generateDomain();

        LLState s = new LLState(new LLAgent(5, 0, 0), new LLBlock.LLPad(75, 95, 0, 10, "pad"));

        ConcatenatedObjectFeatures inputFeatures = new ConcatenatedObjectFeatures()
                .addObjectVectorizion(LunarLanderDomain.CLASS_AGENT, new NumericVariableFeatures());

        int nTilings = 5;
        double resolution = 10.;

        double xWidth = (lld.getXmax() - lld.getXmin()) / resolution;
        double yWidth = (lld.getYmax() - lld.getYmin()) / resolution;
        double velocityWidth = 2 * lld.getVmax() / resolution;
        double angleWidth = 2 * lld.getAngmax() / resolution;



        TileCodingFeatures tilecoding = new TileCodingFeatures(inputFeatures);
        tilecoding.addTilingsForAllDimensionsWithWidths(
                new double []{xWidth, yWidth, velocityWidth, velocityWidth, angleWidth},
                nTilings,
                TilingArrangement.RANDOM_JITTER);





        double defaultQ = 0.5;
        DifferentiableStateActionValue vfa = tilecoding.generateVFA(defaultQ/nTilings);
        GradientDescentSarsaLam agent = new GradientDescentSarsaLam(domain, 0.99, vfa, 0.02, 0.5);

        SimulatedEnvironment env = new SimulatedEnvironment(domain, s);
        List<Episode> episodes = new ArrayList<Episode>();
        for(int i = 0; i < 1000; i++){
            Episode ea = agent.runLearningEpisode(env);
            episodes.add(ea);
            System.out.println(i + ": " + ea.maxTimeStep());
            env.resetEnvironment();
        }

        Visualizer v = LLVisualizer.getVisualizer(lld.getPhysParams());
        new EpisodeSequenceVisualizer(v, domain, episodes);



    }
}
