package Quaternions;

import java.util.Scanner;

public class Calculator {
    public static void main(String[] args){
        Calculator c = new Calculator();
        Scanner scanski = new Scanner(System.in);
        System.out.println(" 1 or 2 ");
        int option = scanski.nextInt();
        if(option == 1) {
            System.out.println("Enter p0 p1 p2 p3");
            double p0 = scanski.nextDouble();
            double p1 = scanski.nextDouble();
            double p2 = scanski.nextDouble();
            double p3 = scanski.nextDouble();
            System.out.println("Enter q0 q1 q2 q3");
            double q0 = scanski.nextDouble();
            double q1 = scanski.nextDouble();
            double q2 = scanski.nextDouble();
            double q3 = scanski.nextDouble();

            double[] qQUAT = {q0, q1, q2, q3};
            double[] pQUAT = {p0, p1, p2, p3};

            double[] quatPRODUCT = c.QUATERNION_PRODUCT(pQUAT, qQUAT);
            for (int i = 0; i < quatPRODUCT.length; i++) {
                System.out.println(quatPRODUCT[i]);
            }

            System.out.println();
            double[] quatINVERSE = c.quatInverse(pQUAT);
            for (int i = 0; i < quatINVERSE.length; i++) {
                System.out.println(quatINVERSE[i]);
            }
        }
        else{

            System.out.println("Enter point to be rotato");
            double v0 = scanski.nextDouble();
            double v1 = scanski.nextDouble();
            double v2 = scanski.nextDouble();
            double[] v = new double[]{v0, v1, v2};

            System.out.println("Enter axis of rotato");
            double u0 = scanski.nextDouble();
            double u1 = scanski.nextDouble();
            double u2 = scanski.nextDouble();
            double[] u = new double[]{u0, u1, u2};

            System.out.println("Enter angle in degrees to be rotato'd");
            double theta = -scanski.nextDouble();

            System.out.println("Resultant Point");
            double[] uhat = c.unitVect(u);
            double[] vector_component = c.applyScalarToVector(Math.sin(theta/2 * Math.PI/180), uhat);
            double[] qQUAT = new double[]{Math.cos(theta/2 * Math.PI/180), vector_component[0], vector_component[1], vector_component[2]};

            double[] resultantPoint = c.quatOpOnVec(qQUAT, v);
            for(int i = 0; i< resultantPoint.length; i++)
            {
                System.out.println(resultantPoint[i]);
            }
        }

    }

    private double[] unitVect(double[] v){
        return applyScalarToVector(1/Math.sqrt(dotProduct(v,v)), v);
    }
    private double dotProduct(double[] p, double[] q){
        return p[0]*q[0] + p[1]*q[1] + p[2]*q[2];
    }

    private double[] crossProduct(double[] p, double[] q){
        double r1 = p[1]*q[2]-q[1]*p[2];
        double r2 = p[2]*q[0]-q[2]*p[0];
        double r3 = p[0]*q[1]-q[0]*p[1];
        return new double[]{r1, r2, r3};
    }

    private double[] applyScalarToVector(double k, double[] r){
        return new double[]{k*r[0], k*r[1], k*r[2]};
    }

    private double[] applyScalarToQuat(double k, double[] rQUAT){
        return new double[]{k*rQUAT[0], k*rQUAT[1], k*rQUAT[2], k*rQUAT[3]};
    }

    private double[] addVectors(double[] p, double[] q){
        return new double[]{p[0] + q[0], p[1] + q[1], p[2] + q[2]};
    }

    private double[] quatConjugate(double[] pQUAT){
        return new double[]{pQUAT[0], -pQUAT[1], -pQUAT[2], -pQUAT[3]};
    }

    private double quatNorm (double[] pQUAT){
        double[] pConj = quatConjugate(pQUAT);
        return Math.sqrt(QUATERNION_PRODUCT(pQUAT, pConj)[0]);
    }

    private double vectNorm (double[] v){
        double sumSquares = Math.pow(v[0], 2) + Math.pow(v[1], 2) + Math.pow(v[2], 2);
        return Math.sqrt(sumSquares);
    }

    private double[] quatInverse(double[] p){
        double scalar = 1/Math.pow(quatNorm(p), 2);

        return applyScalarToQuat(scalar, quatConjugate(p));
    }

    private double[] quatOpOnVec(double[] qQUAT, double[] vector){
        double[] q = new double[]{qQUAT[1], qQUAT[2], qQUAT[3]};
        double q0 = qQUAT[0];

        double firstCoeff = Math.pow(q0, 2) - Math.pow(vectNorm(q),2);
        double[] firstTerm = applyScalarToVector(firstCoeff, vector);

        double secondCoeff = 2 * dotProduct(q, vector);
        double[] secondTerm = applyScalarToVector(secondCoeff, q);

        double thirdCoeff = 2 * q0;
        double[] thirdTerm = applyScalarToVector(thirdCoeff, crossProduct(q, vector));

        double[] resultantVector = addVectors(addVectors(firstTerm, secondTerm), thirdTerm);
        for(int i = 0; i < resultantVector.length; i++){
            if(Math.abs(resultantVector[i]) < Math.pow(2, -20)){
                resultantVector[i] = 0;
            }
        }
        return resultantVector;
    }

    private double[] QUATERNION_PRODUCT(double[] pQUAT, double[] qQUAT){
        double[] p = new double[]{pQUAT[1], pQUAT[2], pQUAT[3]};
        double[] q = new double[]{qQUAT[1], qQUAT[2], qQUAT[3]};
        double pConst = pQUAT[0];
        double qConst = qQUAT[0];

        double rConst = pConst*qConst - dotProduct(p, q);

        double[] firstTerm = applyScalarToVector(pConst, q);
        double[] secondTerm = applyScalarToVector(qConst, p);
        double[] thirdTerm = crossProduct(p, q);
        double[] vectorTerm = addVectors(addVectors(firstTerm, secondTerm), thirdTerm);

        return new double[]{rConst, vectorTerm[0], vectorTerm[1], vectorTerm[2]};
    }

}
