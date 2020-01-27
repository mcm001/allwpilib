/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.estimator;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.json.JsonMapper;
import com.fasterxml.jackson.databind.jsontype.BasicPolymorphicTypeValidator;
import com.fasterxml.jackson.databind.jsontype.PolymorphicTypeValidator;
import edu.wpi.first.wpilibj.controller.LinearQuadraticRegulatorTest;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpiutil.math.MatBuilder;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N2;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static edu.wpi.first.wpilibj.controller.LinearSystemLoopTest.kDt;

public class KalmanFilterTest {
    @BeforeAll
    public static void setup() {
        LinearQuadraticRegulatorTest.setUp();
    }

    @Test
    public void testElevatorKalmanFilter() {
        var plant = LinearQuadraticRegulatorTest.elevatorPlant;

        var Q = new MatBuilder<>(Nat.N2(), Nat.N1()).fill(0.05, 1.0);
        var R = new MatBuilder<>(Nat.N1(), Nat.N1()).fill(0.0001);

        var filter = new KalmanFilter<>(Nat.N2(), Nat.N1(), Nat.N1(), plant, Q, R, kDt);

        var p = filter.getP();
        var gain = filter.getXhat();

        System.out.printf("p: \n%s\n: gain: \n%s\n", p, gain);

    }

    @Test
    public void testmeme() {
        List<List<Double>> reference = List.of(List.of(4.61589, 0.0, 0.0, 0.0), List.of(4.715973, 3.0, 0.0, 0.0), List.of(4.740273, 3.0, 0.0, 0.0), List.of(4.760862, 3.0, 0.0, 0.0), List.of(4.780011, 3.0, 0.0, 0.0), List.of(4.810805, 3.0, 0.001227338028, 8.77E-4), List.of(4.835815, 3.0, 0.008211475857, 0.03097567405), List.of(4.867488, 3.0, 0.008211475857, 0.03097567405), List.of(4.889113, 3.0, 0.0218290835, 0.1396827566), List.of(4.91739, 3.0, 0.02743977163, 0.1791329075), List.of(4.935668, 3.0, 0.03337190544, 0.2141997083), List.of(4.956285, 3.0, 0.03880725956, 0.2375775755), List.of(4.975033, 3.0, 0.0448562827, 0.2580332093), List.of(5.002849, 3.0, 0.0448562827, 0.2717677063), List.of(5.028339, 3.0, 0.0618636811, 0.2843333099), List.of(5.043665, 3.0, 0.0618636811, 0.2843333099), List.of(5.060565, 3.0, 0.0618636811, 0.2843333099), List.of(5.076453, 3.0, 0.07214994267, 0.2819955232), List.of(5.093316, 3.0, 0.07758529679, 0.2784888431), List.of(5.112272, 3.0, 0.08316676259, 0.274982163), List.of(5.127893, 3.0, 0.08316676259, 0.274982163), List.of(5.142189, 3.0, 0.09441736118, 0.2717677063), List.of(5.15897, 3.0, 0.09441736118, 0.2717677063), List.of(5.176439, 3.0, 0.09967738131, 0.272352153), List.of(5.195192, 3.0, 0.1058725161, 0.2738132697), List.of(5.211283, 3.0, 0.1058725161, 0.2761510564), List.of(5.229971, 3.0, 0.1111617586, 0.2787810664), List.of(5.247867, 3.0, 0.1231136932, 0.2819955232), List.of(5.262513, 3.0, 0.1231136932, 0.2819955232), List.of(5.278653, 3.0, 0.1231136932, 0.2819955232), List.of(5.298522, 3.0, 0.1345688481, 0.2872555433), List.of(5.32019, 3.0, 0.1402964256, 0.2884244367), List.of(5.33821, 3.0, 0.1456441127, 0.2884244367), List.of(5.357286, 3.0, 0.1516054688, 0.28696332), List.of(5.375323, 3.0, 0.1516054688, 0.2855022033), List.of(5.392346, 3.0, 0.1618625081, 0.2825799699), List.of(5.409086, 3.0, 0.1618625081, 0.2781966198), List.of(5.428381, 3.0, 0.1723241037, 0.272352153), List.of(5.456587, 2.853588174, 0.1723241037, 0.2667999095), List.of(5.472607, 2.716981738, 0.1829610332, 0.2632932294), List.of(5.492952, 2.59514258, 0.1886009437, 0.2624165594), List.of(5.509493, 2.479187271, 0.1940655202, 0.2635854528), List.of(5.528521, 2.366034669, 0.1993547626, 0.2662154628), List.of(5.549324, 2.254209373, 0.2046732274, 0.2685532495), List.of(5.571848, 2.143953417, 0.2096702465, 0.2688454729), List.of(5.58995, 2.046517386, 0.2096702465, 0.2688454729), List.of(5.609194, 1.936638294, 0.2187583924, 0.2600787727), List.of(5.635953, 1.840750965, 0.2187583924, 0.2600787727), List.of(5.654989, 1.736220862, 0.2263561992, 0.2416687023), List.of(5.671679, 1.63649987, 0.2298044346, 0.2270575353), List.of(5.697963, 1.54163323, 0.2325221117, 0.2118619216), List.of(5.719075, 1.450981395, 0.2349767878, 0.1940362978), List.of(5.743087, 1.36478735, 0.2369931288, 0.1750417807), List.of(5.76999, 1.279961671, 0.2400030292, 0.1338382898), List.of(5.815968, 1.200954677, 0.2412595896, 0.09438813885), List.of(5.831767, 1.129145355, 0.2413764789, 0.07334805836), List.of(5.850205, 1.063296609, 0.2413764789, 0.05493798794), List.of(5.872074, 1.002547624, 0.2413764789, 0.03652791751), List.of(5.886978, 0.9463713058, 0.2413764789, 0.02191675051), List.of(5.902746, 0.8943766467, 0.2413764789, 0.01139671026), List.of(5.919849, 0.8462363275, 0.2413764789, 0.01139671026), List.of(5.935559, 0.8031726503, 0.2407335876, 0.003798903421), List.of(5.951914, 0.7646377291, 0.2400030292, -0.002630010061), List.of(5.967409, 0.7302466263, 0.2392140262, -0.009643370223), List.of(5.984636, 0.6996600715, 0.2383958008, -0.01753340041), List.of(6.006065, 0.6706165205, 0.2383958008, -0.02483898391), List.of(6.022624, 0.6466975552, 0.2370223511, -0.03097567405), List.of(6.041768, 0.6236494079, 0.2370223511, -0.03097567405), List.of(6.059186, 0.6019836428, 0.2370223511, -0.03097567405), List.of(6.076544, 0.5843524927, 0.2359411248, -0.03565124749), List.of(6.095768, 0.5682451578, 0.2355612344, -0.03389790745), List.of(6.110964, 0.5534146628, 0.2353274558, -0.03068345071), List.of(6.127023, 0.5393773774, 0.2353274558, -0.03068345071), List.of(6.142633, 0.52675215, 0.2351228994, -0.02104008049), List.of(6.15936, 0.5149028861, 0.2351228994, -0.02104008049), List.of(6.177817, 0.5038744216, 0.2351228994, -0.01578006036), List.of(6.19653, 0.4936428636, 0.2351228994, -0.01110448692), List.of(6.211945, 0.4841624883, 0.2351228994, -0.01110448692), List.of(6.230175, 0.4753824405, 0.2351228994, -0.003506680081), List.of(6.250483, 0.4672525559, 0.2351228994, -0.0014611167), List.of(6.271264, 0.459725274, 0.2351228994, -0.0014611167), List.of(6.289552, 0.4527561544, 0.2351228994, 0.0), List.of(6.306151, 0.4463039022, 0.2351228994, 0.0), List.of(6.322864, 0.440330228, 0.2351228994, 0.0), List.of(6.340453, 0.4347996593, 0.2351228994, 0.0), List.of(6.355137, 0.4296793443, 0.2351228994, 0.0), List.of(6.372015, 0.4249388624, 0.2351228994, 0.0), List.of(6.387373, 0.4205500462, 0.2351228994, 0.0), List.of(6.404549, 0.4164868154, 0.2351228994, 0.0), List.of(6.422616, 0.4127250225, 0.2351228994, 0.0), List.of(6.440965, 0.4092423098, 0.2351228994, 0.0), List.of(6.460945, 0.4060179771, 0.2351228994, 0.0), List.of(6.479382, 0.4030328588, 0.2351228994, 0.0), List.of(6.497691, 0.4002692102, 0.2351228994, 0.0), List.of(6.518226, 0.3977106027, 0.2351228994, 0.0), List.of(6.535247, 0.3953418256, 0.2351228994, 0.0), List.of(6.554081, 0.3931487965, 0.2351228994, 0.0), List.of(6.569642, 0.3911184772, 0.2351228994, 0.0), List.of(6.589503, 0.3892387967, 0.2351228994, 0.0), List.of(6.604579, 0.3874985793, 0.2351228994, 0.0), List.of(6.626066, 0.3858874782, 0.2351228994, 0.0), List.of(6.643646, 0.384395914, 0.2351228994, 0.0), List.of(6.660937, 0.3830150181, 0.2351228994, 0.0), List.of(6.678626, 0.3817365797, 0.2351228994, 0.0), List.of(6.695713, 0.380552997, 0.2351228994, 0.0), List.of(6.714616, 0.3794572323, 0.2351228994, 0.0), List.of(6.734292, 0.3784427699, 0.2351228994, 0.0), List.of(6.751058, 0.3775035777, 0.2351228994, 0.0), List.of(6.769701, 0.3766340708, 0.2351228994, 0.0), List.of(6.78699, 0.3758290789, 0.2351228994, 0.0), List.of(6.805117, 0.3750838152, 0.2351228994, 0.0), List.of(6.824717, 0.3743938481, 0.2351228994, 0.0), List.of(6.846398, 0.3737550746, 0.2351228994, 0.0), List.of(6.864995, 0.3731636965, 0.2351228994, 0.0), List.of(6.882301, 0.372616197, 0.2351228994, 0.0), List.of(6.900657, 0.3721093205, 0.2351228994, 0.0), List.of(6.921761, 0.3716400529, 0.2351228994, 0.0), List.of(6.940525, 0.3712056037, 0.2351228994, 0.0), List.of(6.959675, 0.3708033894, 0.2351228994, 0.0), List.of(6.977352, 0.3704310182, 0.2351228994, 0.0), List.of(6.997375, 0.3700862759, 0.2351228994, 0.0), List.of(7.016674, 0.3697671125, 0.2351228994, 0.0), List.of(7.040615, 0.36947163, 0.2351228994, 0.0), List.of(7.061111, 0.3691980714, 0.2351228994, 0.0), List.of(7.080897, 0.3689448101, 0.2351228994, 0.0), List.of(7.099389, 0.3687103398, 0.2351228994, 0.0), List.of(7.119672, 0.3684932666, 0.2351228994, 0.0), List.of(7.137315, 0.3682922994, 0.2351228994, 0.0), List.of(7.156976, 0.3681062433, 0.2351228994, 0.0), List.of(7.175992, 0.3679339919, 0.2351228994, 0.0), List.of(7.195996, 0.3677745209, 0.2351228994, 0.0), List.of(7.217836, 0.3676268821, 0.2351228994, 0.0), List.of(7.240198, 0.3674901975, 0.2351228994, 0.0), List.of(7.259395, 0.3673636544, 0.2351228994, 0.0), List.of(7.27782, 0.3672465003, 0.2351228994, 0.0), List.of(7.296549, 0.3671380385, 0.2351228994, 0.0), List.of(7.319769, 0.3670376242, 0.2351228994, 0.0), List.of(7.338109, 0.3669446602, 0.2351228994, 0.0), List.of(7.357742, 0.3668585937, 0.2351228994, 0.0), List.of(7.376777, 0.366778913, 0.2351228994, 0.0), List.of(7.396287, 0.3667051442, 0.2351228994, 0.0), List.of(7.417514, 0.3666368488, 0.2351228994, 0.0), List.of(7.437638, 0.3665736205, 0.2351228994, 0.0), List.of(7.45672, 0.3665150835, 0.2351228994, 0.0), List.of(7.474645, 0.3664608897, 0.2351228994, 0.0), List.of(7.492367, 0.3664107168, 0.2351228994, 0.0), List.of(7.513825, 0.3663642665, 0.2351228994, 0.0), List.of(7.533785, 0.3663212625, 0.2351228994, 0.0), List.of(7.551205, 0.3662814492, 0.2351228994, 0.0), List.of(7.570451, 0.3662445899, 0.2351228994, 0.0), List.of(7.588289, 0.3662104654, 0.2351228994, 0.0), List.of(7.60586, 0.3661788727, 0.2351228994, 0.0), List.of(7.629546, 0.366149624, 0.2351228994, 0.0), List.of(7.645728, 0.3661225454, 0.2351228994, 0.0), List.of(7.663368, 0.3660974759, 0.2351228994, 0.0), List.of(7.680314, 0.3660742664, 0.2351228994, 0.0), List.of(7.698625, 0.3660527789, 0.2351228994, 0.0), List.of(7.721329, 0.3660328857, 0.2351228994, 0.0), List.of(7.73973, 0.3660144684, 0.2351228994, 0.0), List.of(7.757094, 0.3659974176, 0.2351228994, 0.0), List.of(7.776817, 0.3659816319, 0.2351228994, 0.0), List.of(7.798738, 0.3659670173, 0.2351228994, 0.0), List.of(7.815891, 0.3659534871, 0.2351228994, 0.0), List.of(7.835569, 0.3659409607, 0.2351228994, 0.0), List.of(7.854935, 0.3659293638, 0.2351228994, 0.0), List.of(7.874852, 0.3659186272, 0.2351228994, 0.0), List.of(7.895017, 0.3659775612, 0.2350936771, 0.0), List.of(7.918608, 0.36601495, 0.2350936771, 0.0), List.of(7.942042, 0.3660433699, 0.2350936771, 0.0), List.of(7.960693, 0.3660674467, 0.2350936771, -2.92E-4), List.of(7.980471, 0.3660889311, 0.2350936771, -2.92E-4), List.of(8.000121, 0.3661085307, 0.2350936771, 0.0), List.of(8.026045, 0.3661265713, 0.2350936771, 0.0), List.of(8.04757, 0.3661432357, 0.2350936771, 0.0), List.of(8.066927, 0.36615865, 0.2350936771, 0.0), List.of(8.08473, 0.3661729159, 0.2350936771, 0.0), List.of(8.100504, 0.3661861215, 0.2350936771, 0.0), List.of(8.118761, 0.3661983468, 0.2350936771, 0.0), List.of(8.138544, 0.3662096649, 0.2350936771, 0.0), List.of(8.15813, 0.3662201432, 0.2350936771, 0.0), List.of(8.175942, 0.3662298441, 0.2350936771, 0.0), List.of(8.191266, 0.3662388252, 0.2350936771, 0.0), List.of(8.212422, 0.36624714, 0.2350936771, 0.0), List.of(8.232516, 0.366254838, 0.2350936771, 0.0), List.of(8.25611, 0.3662619648, 0.2350936771, 0.0), List.of(8.277983, 0.3662685629, 0.2350936771, 0.0), List.of(8.301396, 0.3662746715, 0.2350936771, 0.0), List.of(8.320337, 0.3662803268, 0.2350936771, 0.0), List.of(8.343201, 0.3662855626, 0.2350936771, 0.0), List.of(8.358596, 0.36629041, 0.2350936771, 0.0), List.of(8.374659, 0.3662948977, 0.2350936771, 0.0), List.of(8.400573, 0.3662990525, 0.2350936771, 0.0), List.of(8.422314, 0.366302899, 0.2350936771, 0.0), List.of(8.438787, 0.3663064602, 0.2350936771, 0.0), List.of(8.459606, 0.3663097571, 0.2350936771, 0.0), List.of(8.476528, 0.3663128095, 0.2350936771, 0.0), List.of(8.492553, 0.3663156354, 0.2350936771, 0.0), List.of(8.517669, 0.3663182516, 0.2350936771, 0.0), List.of(8.534685, 0.3663206738, 0.2350936771, 0.0), List.of(8.552316, 0.3663229162, 0.2350936771, 0.0), List.of(8.568982, 0.3663249923, 0.2350936771, 0.0), List.of(8.587188, 0.3663269143, 0.2350936771, 0.0), List.of(8.605449, 0.3663286938, 0.2350936771, 0.0), List.of(8.632429, 0.3663303412, 0.2350936771, 0.0), List.of(8.651823, 0.3663318664, 0.2350936771, 0.0), List.of(8.669737, 0.3663332784, 0.2350936771, 0.0), List.of(8.686462, 0.3663345857, 0.2350936771, 0.0), List.of(8.703354, 0.366335796, 0.2350936771, 0.0), List.of(8.72613, 0.3663369165, 0.2350936771, 0.0), List.of(8.754162, 0.3663379539, 0.2350936771, 0.0), List.of(8.772318, 0.3663389143, 0.2350936771, 0.0), List.of(8.790638, 0.3663398035, 0.2350936771, 0.0), List.of(8.807819, 0.3663406267, 0.2350936771, 0.0), List.of(8.832089, 0.3663413888, 0.2350936771, 0.0), List.of(8.852899, 0.3663420944, 0.2350936771, 0.0), List.of(8.883611, 0.3663427476, 0.2350936771, 0.0), List.of(8.905941, 0.3663433524, 0.2350936771, 0.0), List.of(8.927069, 0.3663439123, 0.2350936771, 0.0), List.of(8.945666, 0.3663444306, 0.2350936771, 0.0), List.of(8.962636, 0.3663449105, 0.2350936771, 0.0), List.of(8.979273, 0.3663453548, 0.2350936771, 0.0), List.of(9.001364, 0.3663457662, 0.2350936771, 0.0), List.of(9.019922, 0.366346147, 0.2350936771, 0.0), List.of(9.040643, 0.3663464995, 0.2350936771, 0.0), List.of(9.059335, 0.3663468259, 0.2350936771, 0.0), List.of(9.076871, 0.3663471281, 0.2350936771, 0.0), List.of(9.09484, 0.3663474079, 0.2350936771, 0.0), List.of(9.119007, 0.3663476669, 0.2350936771, 0.0), List.of(9.136713, 0.3663479067, 0.2350936771, 0.0), List.of(9.154978, 0.3663481287, 0.2350936771, 0.0), List.of(9.173618, 0.3663483342, 0.2350936771, 0.0), List.of(9.195607, 0.3663485245, 0.2350936771, 0.0), List.of(9.217902, 0.3663487007, 0.2350936771, 0.0), List.of(9.238173, 0.3663488638, 0.2350936771, 0.0), List.of(9.254975, 0.3663490148, 0.2350936771, 0.0), List.of(9.275195, 0.3663491546, 0.2350936771, 0.0), List.of(9.291944, 0.366349284, 0.2350936771, 0.0), List.of(9.308417, 0.3663494038, 0.2350936771, 0.0), List.of(9.331234, 0.3663495148, 0.2350936771, 0.0), List.of(9.35386, 0.3663496175, 0.2350936771, 0.0), List.of(9.372391, 0.3663497126, 0.2350936771, 0.0), List.of(9.391028, 0.3663498006, 0.2350936771, 0.0), List.of(9.413683, 0.3663498821, 0.2350936771, 0.0), List.of(9.434578, 0.3663499575, 0.2350936771, 0.0), List.of(9.452771, 0.3663500274, 0.2350936771, 0.0), List.of(9.479427, 0.3663500921, 0.2350936771, 0.0), List.of(9.4983, 0.3663501519, 0.2350936771, 0.0), List.of(9.532805, 0.3663502074, 0.2350936771, 0.0), List.of(9.556522, 0.3663502587, 0.2350936771, 0.0), List.of(9.578713, 0.3663503062, 0.2350936771, 0.0), List.of(9.607636, 0.3663503502, 0.2350936771, 0.0), List.of(9.63611, 0.3663503909, 0.2350936771, 0.0), List.of(9.654794, 0.3663504286, 0.2350936771, 0.0), List.of(9.671895, 0.3663504635, 0.2350936771, 0.0), List.of(9.693132, 0.3663504958, 0.2350936771, 0.0), List.of(9.717967, 0.3663505257, 0.2350936771, 0.0), List.of(9.740573, 0.3663505534, 0.2350936771, 0.0), List.of(9.763917, 0.3663505791, 0.2350936771, 0.0), List.of(9.784229, 0.3663506028, 0.2350936771, 0.0), List.of(9.804456, 0.3663506248, 0.2350936771, 0.0), List.of(9.830199, 0.3663506452, 0.2350936771, 0.0), List.of(9.850664, 0.366350664, 0.2350936771, 0.0), List.of(9.874384, 0.3663506814, 0.2350936771, 0.0), List.of(9.894857, 0.3663506976, 0.2350936771, 0.0), List.of(9.916526, 0.3663507125, 0.2350936771, 0.0), List.of(9.937931, 0.3663507264, 0.2350936771, 0.0), List.of(9.957411, 0.3663507392, 0.2350936771, 0.0), List.of(9.98017, 0.366350751, 0.2350936771, 0.0), List.of(10.003542, 0.366350762, 0.2350936771, 0.0), List.of(10.023659, 0.3663507722, 0.2350936771, 0.0), List.of(10.042654, 0.3663507816, 0.2350936771, 0.0), List.of(10.064342, 0.3663507903, 0.2350936771, 0.0), List.of(10.084883, 0.3663507984, 0.2350936771, 0.0), List.of(10.110145, 0.3663508059, 0.2350936771, 0.0), List.of(10.133805, 0.3663508128, 0.2350936771, 0.0), List.of(10.153578, 0.3663508192, 0.2350936771, 0.0), List.of(10.174174, 0.3663508251, 0.2350936771, 0.0), List.of(10.192996, 0.3663508306, 0.2350936771, 0.0), List.of(10.212888, 0.3663508357, 0.2350936771, 0.0), List.of(10.243101, 0.3663508404, 0.2350936771, 0.0), List.of(10.265012, 0.3663508447, 0.2350936771, 0.0), List.of(10.288967, 0.3663508488, 0.2350936771, 0.0), List.of(10.312388, 0.3663508525, 0.2350936771, 0.0), List.of(10.341221, 0.366350856, 0.2350936771, 0.0), List.of(10.364062, 0.3663508592, 0.2350936771, 0.0), List.of(10.386007, 0.3663508621, 0.2350936771, 0.0), List.of(10.407878, 0.3663508649, 0.2350936771, 0.0), List.of(10.436645, 0.3663508674, 0.2350936771, 0.0), List.of(10.456527, 0.3663508697, 0.2350936771, 0.0), List.of(10.478134, 0.3663508719, 0.2350936771, 0.0), List.of(10.500668, 0.3663508739, 0.2350936771, 0.0), List.of(10.52363, 0.3663508758, 0.2350936771, 0.0), List.of(10.545456, 0.3663508775, 0.2350936771, 0.0), List.of(10.570845, 0.3663508791, 0.2350936771, 0.0), List.of(10.594894, 0.3663508806, 0.2350936771, 0.0), List.of(10.620741, 0.366350882, 0.2350936771, 0.0), List.of(10.63948, 0.3663508832, 0.2350936771, 0.0), List.of(10.658405, 0.3663508844, 0.2350936771, 0.0), List.of(10.679335, 0.3663508855, 0.2350936771, 0.0), List.of(10.714328, 0.3663508865, 0.2350936771, 0.0), List.of(10.756297, 0.3663508874, 0.2350936771, 0.0), List.of(10.782365, 0.3663508883, 0.2350936771, 0.0), List.of(10.801345, 0.3663508891, 0.2350936771, 0.0), List.of(10.824258, 0.3663508898, 0.2350936771, 0.0), List.of(10.845576, 0.3663508905, 0.2350936771, 0.0), List.of(10.867934, 0.3663508912, 0.2350936771, 0.0), List.of(10.891777, 0.3663508918, 0.2350936771, 0.0), List.of(10.912504, 0.3663508923, 0.2350936771, 0.0), List.of(10.938322, 0.3663508928, 0.2350936771, 0.0), List.of(10.958972, 0.3663508933, 0.2350936771, 0.0), List.of(10.980226, 0.3663508937, 0.2350936771, 0.0), List.of(11.000915, 0.3663508941, 0.2350936771, 0.0), List.of(11.02862, 0.3663508945, 0.2350936771, 0.0), List.of(11.050561, 0.3663508948, 0.2350936771, 0.0), List.of(11.07246, 0.3663508951, 0.2350936771, 0.0), List.of(11.095157, 0.0, 0.2350936771, 0.0));

        var plant = LinearSystem.createElevatorSystem(
                DCMotor.getVex775Pro(4),
                2.0,
                0.75 * 25.4 / 1000d,
                14.67,
                3.0
        );

        var filter = new KalmanFilter<>(Nat.N2(), Nat.N1(), Nat.N1(), plant,
                new MatBuilder<>(Nat.N2(), Nat.N1()).fill(0.05, 200.0),
                new MatBuilder<>(Nat.N1(), Nat.N1()).fill(0.0001), 0.005);

        StringBuilder log = new StringBuilder("pos\n");

        var now = System.currentTimeMillis();

        for(List<Double> state: reference) {

            var voltage = state.get(1);
            var position = state.get(2);
            var velocity = state.get(3);

            var u = new MatBuilder<>(Nat.N1(), Nat.N1()).fill(voltage);
            filter.correct(u, plant.calculateY(new MatBuilder<>(Nat.N2(), Nat.N1()).fill(position, velocity), u));
            filter.predict(u, 0.005);

            log.append(filter.getXhat(0)).append("\n");
        }

        System.out.println(log);

    }

}
