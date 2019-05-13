package ru.yandex.selfdriven.contest;

//https://www.ilikebigbits.com/2015_03_04_plane_from_points.html
// Метод наименьших квадратов
// Но это не будет работать. Т.к. в процессе учитываются все точки - а у нас заранее известно что много шума будет.
// И есть доп условие на input.p, которое здесь никак не учитывается.
public class DummyLS {
    public double[] lsq(Main.LidarInputContainer input) {

        double[] x = input.xArr;
        double[] y = input.yArr;
        double[] z = input.zArr;
        double[] centroid = centroid(x, y, z);

        double xx = 0.0,
                xy = 0.0,
                xz = 0.0,
                yy = 0.0,
                yz = 0.0,
                zz = 0.0;
        int arrLen = x.length;
        for (int i = 0; i < arrLen; i++) {
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

        double detX = yy * zz - yz * yz;
        double detY = xx * zz - xz * xz;
        double detZ = xx * yy - xy * xy;

        double maxDet = Math.max(Math.max(detX, detY), detZ);
        if (maxDet <= 0.0) {
            throw new RuntimeException("Points dont span a plane");
        }
        if (maxDet == detX) {
            return new double[]{detX, xz * yz - xy * zz, 1, xy * yz - xz * yy};
        } else if (maxDet == detY) {
            return new double[]{xz * yz - xy * zz, detY, 1, xy * xz - yz * xx};
        } else {
            return new double[]{xy * yz - xz * yy, xy * xz - yz * xx, 1, detZ};
        }
    }

    //Поидее нельзя так считать медоид надо выбрать.
    private static double[] centroid(double[] x, double[] y, double[] z) {
        int arrLen = x.length;
        double xS = 0, yS = 0, zS = 0;
        for (int i = 0; i < arrLen; i++) {
            xS += x[i];
            yS += y[i];
            zS += z[i];
        }

        return new double[]{xS / arrLen, yS / arrLen, zS / arrLen};
    }
}
