package ru.yandex.selfdriven.contest.test;

import org.junit.Assert;
import org.junit.Test;
import ru.yandex.selfdriven.contest.Main;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Arrays;

import static ru.yandex.selfdriven.contest.Main.RANSACSolution.planeThoughPoints;

public class SimpleTest {
    private Path basePath = Paths.get("/home/mark/code/opensource/yandex-self-driven-car-contest/src/main/resources");


    @Test
    public void test1() throws Exception {
        processFileAndValidateResult(basePath.resolve("input1.txt"));
    }

    @Test
    public void test2() throws Exception {
        processFileAndValidateResult(basePath.resolve("input2.txt"));
    }

    @Test
    public void test3() throws Exception {
        processFileAndValidateResult(basePath.resolve("input3.txt"));
    }

    @Test
    public void testRealCloud() throws Exception {
        double[] doubles = processFileAndValidateResult(basePath.resolve("sdc_point_cloud.txt"));
        System.out.println(Arrays.toString(doubles));
    }

    @Test
    public void testPlaneThoughPoints() {
        double[] a = {1, -2, 0};
        double[] b = {2, 0, -1};
        double[] c = {0, -1, 2};

        double[] res = planeThoughPoints(a, b, c);
        Assert.assertArrayEquals(res, new double[]{5, -1, 3, -7}, 0);
    }

    public double[] processFileAndValidateResult(Path filePath) throws Exception {
        Main.LidarInputContainer lidarInputContainer = Main.readLidarInputFile(filePath.toFile());
        double[] solution = Main.solve(lidarInputContainer);
        Assert.assertTrue(Main.Utils.validateSolution(solution, lidarInputContainer));
        return solution;
    }


}
