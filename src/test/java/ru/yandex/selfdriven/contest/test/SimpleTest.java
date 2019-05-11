package ru.yandex.selfdriven.contest.test;

import org.junit.Assert;
import org.junit.Test;
import ru.yandex.selfdriven.contest.Main;

public class SimpleTest {
    @Test
    public void test1() throws Exception {
        processFile("E:\\Developing\\IdeaProjects\\self-driving-car\\src\\main\\resources\\input1.txt");
    }

    @Test
    public void test2() throws Exception {
        processFile("E:\\Developing\\IdeaProjects\\self-driving-car\\src\\main\\resources\\input2.txt");
    }

    @Test
    public void test3() throws Exception {
        processFile("E:\\Developing\\IdeaProjects\\self-driving-car\\src\\main\\resources\\input3.txt");
    }

    public void processFile(String filePath) throws Exception {
        Main.LidarInputContainer lidarInputContainer = Main.readLidarInputFile(filePath);
        double[] solution = Main.solve(lidarInputContainer);
        Assert.assertTrue(validateSolution(solution, lidarInputContainer));
    }

    private boolean validateSolution(double[] solution, Main.LidarInputContainer input) {
        int errCtx = 0;
        for (int i = 0; i < input.n; i++) {
            double x = input.xArr[i];
            double y = input.yArr[i];
            double z = input.zArr[i];

            double err = Math.abs(solution[0] * x + solution[1] * y + solution[2] * z + solution[3]);
            if (err > input.p) {
                errCtx++;
                System.out.printf("error thresold reached (%.6f,%.6f,%.6f) -> %.6f \n", x, y, z, err);
            }
            if (errCtx > (input.n / 2)) {
                return false;
            }
        }
        return true;
    }
}
