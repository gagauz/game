import java.io.File;
import java.io.IOException;

public class Iter {
    public static void main(String[] args) throws IOException, InterruptedException {

        File dir = new File(args[0]);
        float a0 = Float.parseFloat(args[1]);// -0.5;
        float a1 = Float.parseFloat(args[2]);//  2;
        float da = Float.parseFloat(args[3]);

        float b0 = Float.parseFloat(args[4]);//  -5;
        float b1 = Float.parseFloat(args[5]);//  5;
        float db = Float.parseFloat(args[6]);

        float c0 = Float.parseFloat(args[7]);//  -2000;
        float c1 = Float.parseFloat(args[8]);//  2000;
        float dc = Float.parseFloat(args[9]);

        float d0 = Float.parseFloat(args[10]);//  0.5;
        float d1 = Float.parseFloat(args[11]);//  1;
        float dd = Float.parseFloat(args[12]);

        float e0 = Float.parseFloat(args[13]);//  0.5;
        float e1 = Float.parseFloat(args[14]);//  1;
        float de = Float.parseFloat(args[15]);

        float f0 = Float.parseFloat(args[16]);// 12;
        float f1 = Float.parseFloat(args[17]);// 30;
        float df = Float.parseFloat(args[18]);

        System.out.println(a1 +
                " " + b1 +
                " " + c1 +
                " " + d1 +
                " " + e1 +
                " " + f1);
        for (float a = a0; a <= a1; a += da) {
            for (float b = b0; b <= b1; b += db) {
                for (float c = c0; c <= c1; c += dc) {
                    for (float d = d0; d <= d1; d += dd) {
                        for (float e = e0; e <= e1; e += de) {
                            //                            for (float f = f0; f <= f1; f += df) {

                            long t = System.currentTimeMillis();
                            System.out.println("c=" + c);
                            String s = dir + "/local-runner-console-params.bat" +
                                    " " + a +
                                    " " + b +
                                    " " + c +
                                    " " + d +
                                    " " + e
                            //                            " " + f
                            ;
                            System.out.println(s);
                            try {
                                Process p = Runtime.getRuntime().exec(s, null, dir);
                                p.waitFor();
                                System.out.println("Time: " + (System.currentTimeMillis() - t) + " ms");
                            } catch (Exception ex) {
                                ex.printStackTrace();
                            }
                        }
                    }
                    //                    }
                }
            }
        }
    }
}
