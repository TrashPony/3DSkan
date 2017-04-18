/**
 * Created by TrashPony13@gmail.com on 19.03.2017.
 */
import jssc.*;
import java.util.ArrayList;

import static java.lang.Thread.sleep;

public class Scan {
    private static SerialPort serialPort;
    private static ArrayList<Float> iR_sensor1 = new ArrayList<>();
    private static ArrayList<Float> iR_sensor2 = new ArrayList<>();
    private static String dataString = "";

    public static void main(String args[]) {

        //Calculations();
        serialPort = new SerialPort("COM3");
        try {
            serialPort.openPort();
            serialPort.setParams(SerialPort.BAUDRATE_256000, SerialPort.DATABITS_8, SerialPort.STOPBITS_1, SerialPort.PARITY_NONE);
            serialPort.addEventListener(new EventListener());
        } catch (Exception e) {
            System.err.println("Scan init.comport" + e);
        }
    }

    private static void Calculations(){
        ArrayList<ArrayList<Coordinate>> boxSide;
        ArrayList<Coordinate> iR_sensorCoordinate1;
        ArrayList<Coordinate> iR_sensorCoordinate2;

        ArrayList<Coordinate> iR_sensorCoordinate1_Kalman;
        ArrayList<Coordinate> iR_sensorCoordinate2_Kalman;
        int MaxMin[];

        float k1;
        float k2;

        try {
            //Переводим показания в обьект типа координата
                // intervalX - это переменная означает что между каждым измерение проходит столько растония в см.
            float intervalX = (float)0.5;
            iR_sensorCoordinate1 = createCoordinates(iR_sensor1, intervalX);
            iR_sensorCoordinate2 = createCoordinates(iR_sensor2, intervalX);
            // Сглаживание данных упрощенным методом Калмана
                // коэффецент храниться в переменной k. 1 Это не обработаные значения, 0.1 максимальное сглаживание.
            k1 = (float) 0.1;
                //Создаем масивы координат с максимальным сглаживанием для нахождения углов.
            iR_sensorCoordinate1_Kalman = checkIndications(iR_sensorCoordinate1, k1);
            iR_sensorCoordinate2_Kalman = checkIndications(iR_sensorCoordinate2, k1);
                //Проганяем масивы координат через фильтр что бы убрать скачки показаний.
            k2 = (float) 0.8;
            iR_sensorCoordinate1 = checkIndications(iR_sensorCoordinate1, k2);
            iR_sensorCoordinate2 = checkIndications(iR_sensorCoordinate2, k2);
            // Находим точки которые теоритически являються углами прямоугольника на максимально сглаженых показаниях
                // и находим точки возможных углов прямоугольника
            MaxMin = findCenterPoint(iR_sensorCoordinate1_Kalman,iR_sensorCoordinate2_Kalman);
            //делим 2 масивка с координатами на 4 масива в точках обозночающие точки сторон прямоугольника MinMax
            boxSide = divideOnSide(iR_sensorCoordinate1,iR_sensorCoordinate2,MaxMin);
            // нахождение коэфицента отклонения AB в формуле прямой y = A*x+B;
            float[] deviationAB1 = Approximation(boxSide.get(0));
            float[] deviationAB2 = Approximation(boxSide.get(1));
            float[] deviationAB3 = Approximation(boxSide.get(2));
            float[] deviationAB4 = Approximation(boxSide.get(3));
            // Координаты пересечений прямых y=A*x+B в формате x:y
            Coordinate crossingCoordinates1 = crossingLiner(deviationAB1[0], deviationAB1[1], deviationAB2[0], deviationAB2[1]);
            Coordinate crossingCoordinates2 = crossingLiner(deviationAB2[0], deviationAB2[1], deviationAB4[0], deviationAB4[1]);
            Coordinate crossingCoordinates3 = crossingLiner(deviationAB3[0], deviationAB3[1], deviationAB4[0], deviationAB4[1]);
            Coordinate crossingCoordinates4 = crossingLiner(deviationAB3[0], deviationAB3[1], deviationAB1[0], deviationAB1[1]);
            // Нахождение длинны прямой по координатам.
            System.out.println("Result: ");
            lengthSide(crossingCoordinates1.x, crossingCoordinates1.y, crossingCoordinates2.x, crossingCoordinates2.y);
            lengthSide(crossingCoordinates2.x, crossingCoordinates2.y, crossingCoordinates3.x, crossingCoordinates3.y);
            lengthSide(crossingCoordinates3.x, crossingCoordinates3.y, crossingCoordinates4.x, crossingCoordinates4.y);
            lengthSide(crossingCoordinates4.x, crossingCoordinates4.y, crossingCoordinates1.x, crossingCoordinates1.y);


            iR_sensor1.clear();
            iR_sensor2.clear();
        } catch (Exception e){
            System.err.println("Scan" + e);
        }

    }

    private static class Coordinate {
        float x;
        float y;
        private Coordinate (float x, float y){
            this.x = x;
            this.y = y;
        }
    }

    private static ArrayList<Coordinate> createCoordinates(ArrayList<Float> y, float intervalX){
        ArrayList<Coordinate> coordinatesXY = new ArrayList<>();
        for(int i = 0; i < y.size(); i++){
            coordinatesXY.add(new Coordinate((float)i*intervalX,y.get(i)));
        }
        return  coordinatesXY;
    }

    private static int[] findCenterPoint(ArrayList<Coordinate> iR_sensor1, ArrayList<Coordinate> iR_sensor2){

        int posMaxUnitIR1 = 0;
        int posMinUnitIR2 = 0;
        float max = iR_sensor1.get(0).y;
        float min = iR_sensor2.get(0).y;
        int[] MaxMin = new int[2];

        for(int i = 0; i < iR_sensor1.size(); i++) {
            if(iR_sensor1.get(i).y > max) {
                posMaxUnitIR1 = i;
                max = iR_sensor1.get(i).y;
            }
        }

        for(int i = 0; i < iR_sensor2.size(); i++) {
            if(iR_sensor2.get(i).y < min) {
                posMinUnitIR2 = i;
                min = iR_sensor2.get(i).y;
            }
        }

        MaxMin[0] = posMaxUnitIR1;
        MaxMin[1] = posMinUnitIR2;

        return MaxMin;
    }

    private static ArrayList<ArrayList<Coordinate>> divideOnSide(ArrayList<Coordinate> iR_sensor1, ArrayList<Coordinate> iR_sensor2, int[] MaxMin){

        ArrayList<ArrayList<Coordinate>> boxSide = new ArrayList<>();
        ArrayList<Coordinate> boxSide1 = new ArrayList<>();
        ArrayList<Coordinate> boxSide2 = new ArrayList<>();
        ArrayList<Coordinate> boxSide3 = new ArrayList<>();
        ArrayList<Coordinate> boxSide4 = new ArrayList<>();


        boxSide1.addAll(iR_sensor1.subList(0,MaxMin[0]));
        boxSide2.addAll(iR_sensor1.subList(MaxMin[0],iR_sensor1.size()));
        boxSide3.addAll(iR_sensor2.subList(0,MaxMin[1]));
        boxSide4.addAll(iR_sensor2.subList(MaxMin[1],iR_sensor2.size()));


        boxSide.add(boxSide1);
        boxSide.add(boxSide2);
        boxSide.add(boxSide3);
        boxSide.add(boxSide4);

        return boxSide;
    }

    private static ArrayList<Coordinate> checkIndications (ArrayList<Coordinate> coordinates, float k){
        // Mn показание после фильтрования
        float Mn = coordinates.get(0).y;

        for(Coordinate point: coordinates){
            Mn = k * point.y + (1 - k) * Mn;
            point.y = Mn;
        }

        return coordinates;
    }

    private static float[] Approximation (ArrayList<Coordinate> coordinates){
        float A;
        float B;
        float sumXiYi = 0;
        float sumXi2 = 0;
        float sumX = 0;
        float sumY = 0;
        float[] rateAB = new float[2];

        for (Coordinate point: coordinates) {
            sumXiYi = sumXiYi + (point.x * point.y);
            sumXi2 = sumXi2 + (point.x * point.x);
            sumX=sumX + (point.x);
            sumY=sumY + (point.y);
        }

        A = (coordinates.size()*sumXiYi-(sumX*sumY))/(coordinates.size()*sumXi2-(sumX*sumX));
        B = (sumY - (A*sumX))/coordinates.size();

        rateAB[0] = A;
        rateAB[1] = B;
        return rateAB;
    }

    private static Coordinate crossingLiner (float a, float b, float a2, float b2){
        float buffer;
        float x;
        float y;

        Coordinate crossingCoordinates;
        buffer = a - a2;
        x = (b2-b)/buffer;
        y=a*x+b;
        crossingCoordinates = new Coordinate(x,y);

        return crossingCoordinates;
    }

    private static float lengthSide (float x1, float y1, float x2, float y2){
        double length;

        length = Math.sqrt(Math.pow(x1-x2,2) + Math.pow(y1-y2,2));
        System.out.print("Side: ");
        System.out.printf("%.2f", length);
        System.out.println(" mm");

        return (float)length;
    }

    private static class EventListener implements SerialPortEventListener {

        public void serialEvent(SerialPortEvent event) {
            try {
                String data = serialPort.readString();

                char [] dataBytes = data.toCharArray ();
                if(dataBytes.length > 0) {
                    for (char ch : dataBytes) {
                        dataString = dataString + ch;
                        if(ch == 'e'){
                            //System.out.println(dataString);
                            Protocol(dataString);
                            dataString = "";
                        }
                        if(ch == 'n'){
                            dataString = "";
                            Calculations();
                        }
                    }
                }
            } catch (Exception ex) {
                System.out.println("EventListener" + ex);
            }
        }
/*
s (МАРКЕР НАЧАЛА СТРОКИ) f (ид датчика 1) **(остаток) *(множитель) t (ид датчика 2) **(остаток) *(множитель) e (МАРКЕР КОНЦА СТРОКИ);
Если нет обьекта на сканере то всегда посылается n;
s-f-xx-xx-t-xx-xx-e; есть объект
nnnnnnnnnnnnnnnnnnn; нет объекта
*/
        private static void Protocol (String data){
            String[] dataBlocks = data.split("-");
            for (int i = 0; i < dataBlocks.length; i++){
                System.out.print(dataBlocks[i]);

                if(dataBlocks[i].equals("f")){
                    int balance = Integer.parseInt(dataBlocks[i+1]);
                    int factor = Integer.parseInt(dataBlocks[i+2]);
                    iR_sensor1.add((float)balance+(factor * 100));
                }

                if(dataBlocks[i].equals("t")){
                    int balance = Integer.parseInt(dataBlocks[i+1]);
                    int factor = Integer.parseInt(dataBlocks[i+2]);
                    iR_sensor2.add((float)balance+(factor * 100));
                }
                if(dataBlocks[i].equals("e")){
                    System.out.println();
                }
            }
        }
    }
}
