package ru.yandex.selfdriven.contest;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.*;

//https://contest.yandex.ru/contest/12698/problems/
public class Main {
    public static final String FILE_PATH = "input.txt";
    //    public static final String FILE_PATH = "/home/mark/code/opensource/yandex-self-driven-car-contest/src/main/resources/sdc_point_cloud.txt";
    public static final boolean DEBUG = false;

    public static class Utils {
        public static boolean validateSolution(double[] solution, Main.LidarInputContainer input) {
            //Если больше половины точек превысили допустимый порог (input.p) то решение не пройдет.
            int err = countTresholdReachedPoints(solution, input);
            return isSolutionGood(err, input);
        }

        //Число точек, отклоняющихся от модели не больше чем на p
        //должно составлять не менее 50% от общего числа точек.
        public static boolean isSolutionGood(int err, LidarInputContainer input) {
            return err < (input.n / 2.0);
        }

        //Посчитывает кол-во точек, которые отстоят от плоскости больше чем на p
        public static int countTresholdReachedPoints(double[] solution, Main.LidarInputContainer input) {
            int errCtx = 0;
            double a = solution[0];
            double b = solution[1];
            double c = solution[2];
            double d = solution[3];
            if (a != 0 && b != 0 && c != 0 && d != 0) {
                for (int i = 0; i < input.n; i++) {

                    double x = input.xArr[i];
                    double y = input.yArr[i];
                    double z = input.zArr[i];

                    //Расстояние от точки до плоскости
                    double distance = Math.abs(a * x + b * y + c * z + d) / (Math.sqrt(a * a + b * b + c * c));

                    if (distance > input.p) {
                        errCtx++;
                    }
                }
                return errCtx;
            } else {
                //Плоскость 0,0,0,0 - это не ответ.
                return input.n;
            }
        }
    }

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

        public double[] nthPoint(int n) {
            return new double[]{xArr[n], yArr[n], zArr[n]};
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

    //решение алгоритмом RANSAC https://ru.wikipedia.org/wiki/RANSAC
    public static class RANSACSolution {
        private static Random RND = new Random();

        public static double[] solve(LidarInputContainer lic) {

            Set<Integer> ids = new HashSet<>(3);

            int iterations = 100;
            for (int i = 0; i < iterations; i++) {

                //Выбираем 3 рандомные точки из датасета
                while (ids.size() < 3) {
                    int p = RND.nextInt(lic.n);
                    ids.add(p);
                }

                Integer[] idsArr = ids.toArray(new Integer[3]);
                double[] a = lic.nthPoint(idsArr[0]);
                double[] b = lic.nthPoint(idsArr[1]);
                double[] c = lic.nthPoint(idsArr[2]);

                //Строим плоскость через эти точки
                double[] plane = planeThoughPoints(a, b, c);

                //Как только попали под критерии задачи - сразу же выходим
                if (Utils.validateSolution(plane, lic))
                    return plane;

                //Плохие точки выбрали, пробуем еще раз.
                ids.clear();

            }
            return new double[]{0, 0, 0, 0};
            //throw new RuntimeException("Failed ");
        }

        /*
         * Строит плоскость по трем точкам 0 = ax+by+cz+d
         * Возвращает массив коэффициентов [a,b,c,d]
         * */
        public static double[] planeThoughPoints(double[] aPoint, double[] bPoint, double[] cPoint) {
            //На основе точек строим 2 вектора
            double[] abVec = new double[]{bPoint[0] - aPoint[0], bPoint[1] - aPoint[1], bPoint[2] - aPoint[2]};
            double[] bcVec = new double[]{cPoint[0] - bPoint[0], cPoint[1] - bPoint[1], cPoint[2] - bPoint[2]};
            //(a,b,c)  - это векторное произведение, является коэффициентами в нашем уравнении плоскости
            double a = abVec[1] * bcVec[2] - abVec[2] * bcVec[1];
            double b = abVec[2] * bcVec[0] - abVec[0] * bcVec[2];
            double c = abVec[0] * bcVec[1] - abVec[1] * bcVec[0];
            //Подставляем в уравнение 1 точку находим d
            double d = -(a * aPoint[0] + b * aPoint[1] + c * aPoint[2]);
            return new double[]{a, b, c, d};
        }


    }

    //Решение градиентным спуском
    public static class GradDescSolution {
        //TODO: Понять как эти параметры задавать..
        private static final double EPS = 0.01;
        private static final double STEP_MULTIPLIER = 0.01;
        private static final double MAX_ITERATIONS = 10000000;


        private static double f(LidarInputContainer lic, double... params) {

            double res = 0.0;

            for (int i = 0; i < lic.n; i++) {
                //Расстояние от точки до плоскости
                double a = params[0];
                double b = params[1];
                double c = params[2];
                double d = params[3];
                double x = lic.xArr[i];
                double y = lic.yArr[i];
                double z = lic.zArr[i];
                double distance = Math.abs(a * x + b * y + c * z + d) / (Math.sqrt(a * a + b * b + c * c));
                //Если превысили порог - прибавляем, если нет - вычитаем.
                if (distance > lic.p)
                    res += distance;
                else
                    res -= distance;
            }

            return res;
        }

        //Вектор градиента расчитывает
        private static double[] gradient(LidarInputContainer lic, double... params) {
            double[] answ = new double[params.length];
            for (int i = 0; i < params.length; i++) {
                double[] xWithEps = new double[params.length];
                //Расчитываем результирующий вектор, добавляем EPS к параметру для расчтеа производной.
                for (int k = 0; k < xWithEps.length; k++) {
                    if (i == k) {
                        xWithEps[k] = params[k] + EPS;
                    } else {
                        xWithEps[k] = params[k];
                    }
                }
                answ[i] = (f(lic, xWithEps) - f(lic, params)) / EPS;
            }
            return answ;
        }

        public static double[] solve(LidarInputContainer lic) {
            double[] params = {1, 1, 1, 1}; //Начальное предположение
            //TODO: while(true)? А то рискуем выдать плохое решение. С другой стороны уже по времени все лимиты пройдут к тому моменту.
            for (int i = 0; i < MAX_ITERATIONS; i++) {
                double[] grad = gradient(lic, params);
                int errCount = Utils.countTresholdReachedPoints(params, lic);

                if (Utils.isSolutionGood(errCount, lic))
                    break;

                if (DEBUG)
                    System.out.printf("i: %d err: %d params: %s, grad: %s \n", i, errCount, Arrays.toString(params), Arrays.toString(grad));


                for (int k = 0; k < params.length; k++) {
                    params[k] = params[k] - STEP_MULTIPLIER * grad[k];
                }

            }

            return params;
        }
    }

    //Вариации на тему метода наименьших квадратов, но это сделать рабочее решение так и не удалось.
    public static class LeastSquareSolution {
        private static Random RND = new Random();

        private static double[] lsq(Main.LidarInputContainer input, double[] centroid) {
            double[] x = input.xArr;
            double[] y = input.yArr;
            double[] z = input.zArr;

            double xx = 0.0,
                    xy = 0.0,
                    xz = 0.0,
                    yy = 0.0,
                    yz = 0.0,
                    zz = 0.0;
            //Строим плоскость на основе этой точки вида z = T
            double a = 0;
            double b = 0;
            double c = 1;
            double d = -centroid[2];

            for (int i = 0; i < input.n; i++) {
                double distance = Math.abs(a * x[i] + b * y[i] + c * z[i] + d) / (Math.sqrt(a * a + b * b + c * c));
                if (distance <= input.p) {
                    double[] centroidDiff = new double[]{
                            x[i] - centroid[0],
                            y[i] = centroid[1],
                            z[i] - centroid[2]
                    };
                    xx += centroidDiff[0] * centroidDiff[0];
                    xy += centroidDiff[0] * centroidDiff[1];
                    xz += centroidDiff[0] * centroidDiff[2];

                    yy += centroidDiff[1] * centroidDiff[1];
                    yz += centroidDiff[1] * centroidDiff[2];

                    zz += centroidDiff[2] * centroidDiff[2];
                }
            }

            double detX = yy * zz - yz * yz;
            double detY = xx * zz - xz * xz;
            double detZ = xx * yy - xy * xy;

            double maxDet = Math.max(Math.max(detX, detY), detZ);
            if (maxDet <= 0.0) {
                //Нет решения, точки не могут образовать плоскость. Возможно выбрали не ту точку
                return new double[]{0.0, 0.0, 0.0, 0.0};
            }
            if (maxDet == detX) {
                return new double[]{detX, xz * yz - xy * zz, 1, (xy * yz - xz * yy)};
            } else if (maxDet == detY) {
                return new double[]{xz * yz - xy * zz, detY, 1, (xy * xz - yz * xx)};
            } else {
                return new double[]{xy * yz - xz * yy, xy * xz - yz * xx, 1, detZ};
            }
        }

        public static double[] solve(Main.LidarInputContainer input) {

            double[] solution = null;
            boolean good = false;
            int idx = 0;
            while (!good) {
                int i = RND.nextInt(input.n);
                double[] centroid = new double[]{input.xArr[i], input.yArr[i], input.zArr[i]};
                solution = lsq(input, centroid);
                good = Main.Utils.validateSolution(solution, input);
                idx++;
                if (idx > 100)
                    throw new RuntimeException("Too many iterations..");
            }
            return solution;
        }
    }

    public static double[] solve(LidarInputContainer input) {
        //RANSAC - эффективней
        return RANSACSolution.solve(input);
    }

    public static void main(String[] args) throws Exception {
        LidarInputContainer input = readLidarInputFile(new File(FILE_PATH));
        double[] answ = solve(input);
        System.out.printf("%.6f %.6f %.6f %.6f \n", answ[0], answ[1], answ[2], answ[3]);
    }

    //Считывает файл в контейнер
    public static LidarInputContainer readLidarInputFile(File f) throws IOException {
        FileInputStream fis = new FileInputStream(f);
        Scanner sc = new Scanner(fis);
        double p = Double.parseDouble(sc.nextLine());
        int n = Integer.parseInt(sc.nextLine());
        double[] xArr = new double[n];
        double[] yArr = new double[n];
        double[] zArr = new double[n];

        int i = 0;
        while (sc.hasNextLine()) {
            String s = sc.nextLine();
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
}
