package ru.yandex.selfdriven.contest;

import java.io.FileInputStream;
import java.io.IOException;
import java.util.Arrays;
import java.util.Scanner;

public class Main {
    public static final String FILE_PATH = "E:\\Developing\\IdeaProjects\\self-driving-car\\src\\main\\resources\\input3.txt";

    public static class LidarInputContainer {
        public int n;
        public double[] xArr;
        public double[] yArr;
        public double[] zArr;
        public double p;

        public LidarInputContainer(int n, double[] xArr, double[] yArr, double[] zArr, double p) {
            this.n = n;
            this.xArr = xArr;
            this.yArr = yArr;
            this.zArr = zArr;
            this.p = p;
        }

        @Override
        public String toString() {
            return "LidarInputContainer{" +
                    "n=" + n +
                    ", xArr=" + Arrays.toString(xArr) +
                    ", yArr=" + Arrays.toString(yArr) +
                    ", zArr=" + Arrays.toString(zArr) +
                    ", p=" + p +
                    '}';
        }
    }

    public static LidarInputContainer readLidarInputFile(String filePath) throws IOException {
        FileInputStream fis = new FileInputStream(filePath);
        Scanner sc = new Scanner(fis);
        sc.useDelimiter("\r\n");
        double p = Double.parseDouble(sc.next());
        int n = sc.nextInt();
        double[] xArr = new double[n];
        double[] yArr = new double[n];
        double[] zArr = new double[n];

        int i = 0;
        while (sc.hasNext()) {
            String s = sc.next();
            String[] split = s.split("\t");
            double x = Double.parseDouble(split[0]);
            double y = Double.parseDouble(split[1]);
            double z = Double.parseDouble(split[2]);
            xArr[i] = x;
            yArr[i] = y;
            zArr[i] = z;

            i++;
        }

        return new LidarInputContainer(n, xArr, yArr, zArr, p);
    }

    public static double[] solve(LidarInputContainer input) {
        return new DummyLS().lsq(input);
    }

    public static void main(String[] args) throws Exception {
        LidarInputContainer input = readLidarInputFile(FILE_PATH);
        System.out.println(input);
        double[] lsq = solve(input);
        System.out.println(Arrays.toString(lsq));
    }
}
