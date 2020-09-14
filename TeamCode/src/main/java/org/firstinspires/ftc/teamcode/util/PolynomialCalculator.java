package org.firstinspires.ftc.teamcode.util;

// LICENSE
//
//   This software is dual-licensed to the public domain and under the following
//   license: you are granted a perpetual, irrevocable license to copy, modify,
//   publish, and distribute this file as you see fit.
//
// C# VERSION
//   0.1.0  (2016-06-01)  Initial release
//
// AUTHOR
//   Forrest Smith
//
// TRANSPOSED FROM C# TO JAVA
//   By: Letian Li - FTC Team 12611 TechNova
//
//   Float(s) were replaced with Doubles
//   out obj(s) were replaced with Vector3 and DoubleObj
//   File was split to PolynomialCalculator and BallisticTrajectoryUtil
//
// ADDITIONAL READING
//   @link https://medium.com/@ForrestTheWoods/solving-ballistic-trajectories-b0165523348c
//   @link https://www.forrestthewoods.com/blog/solving_ballistic_trajectories/
//
// API
//    boolean IsZero(double d);
//    int SolveQuadric(double c0, double c1, double c2, out double s0, out double s1);
//    int SolveCubic(double c0, double c1, double c2, double c3, out double s0, out double s1, out double s2);
//    int SolveQuartic(double c0, double c1, double c2, double c3, double c4, out double s0, out double s1, out double s2, out double s3);
//
//    Others in BallisticTrajectoryUtil.java
//
// SOURCE
//    https://github.com/forrestthewoods/lib_fts/blob/master/code/fts_ballistic_trajectory.cs
//
//      SolveQuadric, SolveCubic, and SolveQuartic were ported from C as written for Graphics Gems I
//      Original Author: Jochen Schwarze (schwarze@isa.de)
//      https://github.com/erich666/GraphicsGems/blob/240a34f2ad3fa577ef57be74920db6c4b00605e4/gems/Roots3And4.c
//

import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.util.Comparator;

public class PolynomialCalculator {

    /**
     * Utility function used by SolveQuadratic, SolveCubic, and SolveQuartic
     */
    public static boolean IsZero(double d) {
        final double eps = 1e-9;
        return d > -eps && d < eps;
    }

    /**
     * Class to replace (out double) of C# by use of object and reference
     */
    public static class DoubleObj {
        private double number;

        public DoubleObj(double number) {
            this.number = number;
        }

        public void setNaN() {
            this.number = Double.NaN;
        }

        public void plus(double other) {
            this.number += other;
        }

        public void minus(double other) {
            this.number -= other;
        }

        public void times(double scalar) {
            this.number *= scalar;
        }

        public void div(double scalar) {
            this.number /= scalar;
        }

        public void set(double newNumber) {
            this.number = newNumber;
        }

        public double get() {
            return this.number;
        }

        public static class sorter implements Comparator<DoubleObj> {
            public int compare(DoubleObj a, DoubleObj b) {
                return Double.compare(a.get(), b.get());
            }
        }
    }

    /**
     * Class to replace Unity3d's Vector3.
     * Has Pose2d support
     * @link (https://docs.unity3d.com/ScriptReference/Vector3.html)
     */
    public static class Vector3 {
        public double x; // Left-Right
        public double y; // Down-Up
        public double z; // Back-Forward
        public static final Vector3
                back    = new Vector3(0 , 0 , -1),
                down    = new Vector3(0 , -1, 0 ),
                forward = new Vector3(0 , 0 , 1 ),
                left    = new Vector3(-1, 0 , 0 ),
                one     = new Vector3(1 , 1 , 1 ),
                right   = new Vector3(1 , 0 , 0 ),
                up      = new Vector3(0 , 1 , 0 ),
                zero    = new Vector3(0 , 0 , 0 );

        public Vector3(Vector2d vector2d, double height) {
            this.z = vector2d.getX();
            this.x = -vector2d.getY();
            this.y = height;
        }

        public Vector3(double x, double y, double z) {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        public void set(double x, double y, double z) {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        public void set(Vector3 newVector3) {
            set(newVector3.x, newVector3.y, newVector3.z);
        }

        public Vector3 plus(Vector3 other) {
            return new Vector3(this.x + other.x, this.y + other.y, this.z + other.z);
        }

        public Vector3 minus(Vector3 other) {
            return new Vector3(this.x - other.x, this.y - other.y, this.z - other.z);
        }

        public Vector3 times(double scalar) {
            return new Vector3(this.x * scalar, this.y * scalar, this.z * scalar);
        }

        public Vector3 div(double scalar) {
            return new Vector3(this.x/scalar, this.y/scalar, this.z/scalar);
        }

        public static double Dot(Vector3 lhs, Vector3 rhs) {
            return lhs.x*rhs.x + lhs.y*rhs.y + lhs.z*rhs.z;
        }

        /**
         * @return Length of the vector
         */
        public double getMagnitude() {
            return Math.sqrt(this.x*this.x + this.y*this.y + this.z*this.z);
        }

        /**
         * @return Vector when length is 1
         */
        public Vector3 getNormalized() {
            double magnitude = getMagnitude();
            return this.div(magnitude);
        }

        /**
         * @return the Vector2d on the ground plane (converted into RR coord system)
         */
        public Vector2d getGroundVector2d() {
            return new Vector2d(this.z, -this.x);
        }

        /**
         * @return the Vector2d between the ground plane and the height (converted into RR coord system)
         */
        public Vector2d getSkyVector2d() {
            return new Vector2d(getGroundVector2d().norm(), this.y);
        }

        public static double[] getTargetHeadings(Vector3 vector3) {
            double groundHeading = vector3.getGroundVector2d().angle();
            double skyHeading = vector3.getSkyVector2d().angle();
            return new double[]{groundHeading, skyHeading};
        }

        public boolean equals(Vector3 comparison) {
            return this.x == comparison.x && this.y == comparison.y && this.z == comparison.z;
        }
    }

    /**
     * Solve quadratic equation: c0*x^2 + c1*x + c2.
     * @param c0 leading coefficient c0*x^2
     * @param c1 second coefficient c1*x^1
     * @param c2 constant c2*x^0
     *
     * @param s0 solution 1 reference
     * @param s1 solution 2 reference
     * @return number of solutions.
     */
    public static int SolveQuadric(double c0, double c1, double c2, DoubleObj s0, DoubleObj s1) {
        s0.setNaN();
        s1.setNaN();

        double p, q, D;

        /* normal form: x^2 + px + q = 0 */
        p = c1 / (2 * c0);
        q = c2 / c0;

        D = p * p - q;

        if (IsZero(D)) {
            s0.set(-p);
            return 1;
        }
        else if (D < 0) {
            return 0;
        }
        else /* if (D > 0) */ {
            double sqrt_D = Math.sqrt(D);

            s0.set(sqrt_D - p);
            s1.set(-sqrt_D - p);
            return 2;
        }
    }

    /**
     * Solve cubic equation: c0*x^3 + c1*x^2 + c2*x + c3.
     * @param c0 leading coefficient c0*x^3
     * @param c1 second coefficient c1*x^2
     * @param c2 third coefficient c2*x^1
     * @param c3 constant c3*x^0
     * @param s0 solution 1 reference
     * @param s1 solution 2 reference
     * @param s2 solution 3 reference
     * @return number of solutions.
     */
    public static int SolveCubic(double c0, double c1, double c2, double c3, DoubleObj s0, DoubleObj s1, DoubleObj s2) {
        s0.setNaN();
        s1.setNaN();
        s2.setNaN();

        int     num;
        double  sub;
        double  A, B, C;
        double  sq_A, p, q;
        double  cb_p, D;

        /* normal form: x^3 + Ax^2 + Bx + C = 0 */
        A = c1 / c0;
        B = c2 / c0;
        C = c3 / c0;

        /*  substitute x = y - A/3 to eliminate quadric term:  x^3 +px + q = 0 */
        sq_A = A * A;
        p = 1.0/3 * (-1.0/3 * sq_A + B);
        q = 1.0/2 * (2.0/27 * A * sq_A - 1.0/3 * A * B + C);

        /* use Cardano's formula */
        cb_p = p * p * p;
        D = q * q + cb_p;

        if (IsZero(D)) {
            if (IsZero(q)) /* one triple solution */ {
                s0.set(0);
                num = 1;
            }
            else /* one single and one double solution */ {
                double u = Math.pow(-q, 1.0/3.0);
                s0.set(2 * u);
                s1.set(-u);
                num = 2;
            }
        }
        else if (D < 0) /* Casus irreducibilis: three real solutions */ {
            double phi = 1.0/3 * Math.acos(-q / Math.sqrt(-cb_p));
            double t = 2 * Math.sqrt(-p);

            s0.set( t * Math.cos(phi));
            s1.set(-t * Math.cos(phi + Math.PI / 3));
            s2.set(-t * Math.cos(phi - Math.PI / 3));
            num = 3;
        }
        else /* one real solution */ {
            double sqrt_D = Math.sqrt(D);
            double u =  Math.pow(sqrt_D - q, 1.0/3.0);
            double v = -Math.pow(sqrt_D + q, 1.0/3.0);

            s0.set(u + v);
            num = 1;
        }

        /* resubstitute */
        sub = 1.0/3 * A;

        if (num > 0)    s0.minus(sub);
        if (num > 1)    s1.minus(sub);
        if (num > 2)    s2.minus(sub);

        return num;
    }

    /**
     * Solve quartic function: c0*x^4 + c1*x^3 + c2*x^2 + c3*x + c4.
     * @param c0 leading coefficient c0*x^4
     * @param c1 second coefficient c1*x^3
     * @param c2 third coefficient c2*x^2
     * @param c3 forth coefficient c3*x^1
     * @param c4 constant c4*x^0
     * @param s0 solution 1 reference
     * @param s1 solution 2 reference
     * @param s2 solution 3 reference
     * @param s3 solution 4 reference
     * @return number of solutions.
     */
    public static int SolveQuartic(double c0, double c1, double c2, double c3, double c4, DoubleObj s0, DoubleObj s1, DoubleObj s2, DoubleObj s3) {
        s0.setNaN();
        s1.setNaN();
        s2.setNaN();
        s3.setNaN();

        double[]  coeffs = new double[4];
        double  z, u, v, sub;
        double  A, B, C, D;
        double  sq_A, p, q, r;
        int     num;

        /* normal form: x^4 + Ax^3 + Bx^2 + Cx + D = 0 */
        A = c1 / c0;
        B = c2 / c0;
        C = c3 / c0;
        D = c4 / c0;

        /*  substitute x = y - A/4 to eliminate cubic term: x^4 + px^2 + qx + r = 0 */
        sq_A = A * A;
        p = -3.0/8 * sq_A + B;
        q = 1.0/8 * sq_A * A - 1.0/2 * A * B + C;
        r = -3.0/256*sq_A*sq_A + 1.0/16*sq_A*B - 1.0/4*A*C + D;

        if (IsZero(r)) {
            /* no absolute term: y(y^3 + py + q) = 0 */

            coeffs[ 3 ] = q;
            coeffs[ 2 ] = p;
            coeffs[ 1 ] = 0;
            coeffs[ 0 ] = 1;

            num = SolveCubic(coeffs[0], coeffs[1], coeffs[2], coeffs[3], s0, s1, s2);
        }
        else {
            /* solve the resolvent cubic ... */
            coeffs[ 3 ] = 1.0/2 * r * p - 1.0/8 * q * q;
            coeffs[ 2 ] = -r;
            coeffs[ 1 ] = -1.0/2 * p;
            coeffs[ 0 ] = 1;

            SolveCubic(coeffs[0], coeffs[1], coeffs[2], coeffs[3], s0, s1, s2);

            /* ... and take the one real solution ... */
            z = s0.get();

            /* ... to build two quadric equations */
            u = z * z - r;
            v = 2 * z - p;

            if (IsZero(u))
                u = 0;
            else if (u > 0)
                u = Math.sqrt(u);
            else
                return 0;

            if (IsZero(v))
                v = 0;
            else if (v > 0)
                v = Math.sqrt(v);
            else
                return 0;

            coeffs[ 2 ] = z - u;
            coeffs[ 1 ] = q < 0 ? -v : v;
            coeffs[ 0 ] = 1;

            num = SolveQuadric(coeffs[0], coeffs[1], coeffs[2], s0, s1);

            coeffs[ 2 ]= z + u;
            coeffs[ 1 ] = q < 0 ? v : -v;
            coeffs[ 0 ] = 1;

            if (num == 0) num += SolveQuadric(coeffs[0], coeffs[1], coeffs[2], s0, s1);
            if (num == 1) num += SolveQuadric(coeffs[0], coeffs[1], coeffs[2], s1, s2);
            if (num == 2) num += SolveQuadric(coeffs[0], coeffs[1], coeffs[2], s2, s3);
        }

        /* resubstitute */
        sub = 1.0/4 * A;

        if (num > 0)    s0.minus(sub);
        if (num > 1)    s1.minus(sub);
        if (num > 2)    s2.minus(sub);
        if (num > 3)    s3.minus(sub);

        return num;
    }
}
