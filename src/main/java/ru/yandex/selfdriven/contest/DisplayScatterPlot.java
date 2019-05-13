package ru.yandex.selfdriven.contest;

import org.math.plot.Plot3DPanel;

import javax.swing.*;
import java.awt.*;
import java.io.File;
import java.io.IOException;

//Визуализация точек из примера
public class DisplayScatterPlot {
    public static void main(String[] args) throws IOException {
        String path = "/home/mark/code/opensource/yandex-self-driven-car-contest/src/main/resources/sdc_point_cloud.txt";
        Main.LidarInputContainer input = Main.readLidarInputFile(new File(path));

        Plot3DPanel chartPane = new Plot3DPanel();
        chartPane.addScatterPlot("lidar", input.xArr, input.yArr, input.zArr);
        chartPane.setPreferredSize(new Dimension(900,900));

        JFrame frame = new JFrame("a plot panel");
        frame.setContentPane(chartPane);
        frame.setVisible(true);
        frame.pack();
    }
}
