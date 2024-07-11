/**
 * @file trajectory.cpp
 * @brief This file contains the implementation of the trajectory generation functions.
 */

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>

/**
 * @brief Calculates the Euclidean distance between two points.
 * @param point1 The first point.
 * @param point2 The second point.
 * @return The Euclidean distance between the two points.
 */
double calculateDistance(const Eigen::Vector3f& point1, const Eigen::Vector3f& point2) {
    return (point1 - point2).norm();
}

/**
 * @brief Assigns start times to each segment of the trajectory.
 * @param waypoints The waypoints of the trajectory.
 * @param averageVelocity The average velocity of the trajectory.
 * @return A vector of start times for each segment of the trajectory.
 */
std::vector<double> assignSegmentStartTimes(const std::vector<Eigen::Vector3f>& waypoints, const float averageVelocity) {
    std::vector<double> segmentStartTimes;
    segmentStartTimes.push_back(0.0);
    for (int i = 1; i < waypoints.size(); ++i) {
        double distance = calculateDistance(waypoints[i], waypoints[i - 1]);
        double time = distance / averageVelocity;
        segmentStartTimes.push_back(segmentStartTimes.back() + time);
    }
    return segmentStartTimes;
}

/**
 * @brief Normalizes the start times of the trajectory.
 * @param startTimes The start times of the trajectory.
 * @return A vector of normalized start times.
 */
std::vector<double> normalizeStartTimes(const std::vector<double>& startTimes) {
    std::vector<double> normalizedStartTimes;
    double maxTime = startTimes.back();
    for (double time : startTimes) {
        normalizedStartTimes.push_back(time / maxTime);
    }
    return normalizedStartTimes;
}



/**
 * @brief Solves for the coefficients of the polynomial trajectory.
 * @param waypoints The waypoints of the trajectory.
 * @param startTimes The start times of each segment of the trajectory.
 * @param component The component of the trajectory to solve for (X, Y, or Z).
 * @return A vector of coefficients for the polynomial trajectory.
 */
Eigen::VectorXd solveCoefficients(const std::vector<Eigen::Vector3f>& waypoints, const std::vector<double>& startTimes, char component) {

    // Normalize the start times
    std::vector<double> normalizedStartTimes = normalizeStartTimes(startTimes);

    // Number of waypoints
    int n = waypoints.size();

    // Number of segment polynomials
    int m = n - 1;

    // Number of coefficients to solve for
    int numCoefficients = 8 * m;

    // Initialize the constraint matrix
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(numCoefficients, numCoefficients);

    // Initialize the constraint vector
    Eigen::VectorXd b = Eigen::VectorXd::Zero(numCoefficients);

    // Set up the constraints
    // 8 * (n - 1) constraints are required to solve for the 8 * (n - 1) polynomial coefficients.

    // Constraint #1: Velocity is zero at the start position
    Eigen::VectorXd values1(8);
    double t0 = normalizedStartTimes[0];
    values1 << 7 * pow(t0, 6), 6 * pow(t0, 5), 5 * pow(t0, 4), 4 * pow(t0, 3), 3 * pow(t0, 2), 2 * t0, 1, 0;
    A.block<1, 8>(0, 0) = values1.transpose();
    b(0) = 0;

    // Constraint #2: Acceleration is zero at the start position
    Eigen::VectorXd values2(8);
    values2 << 42 * pow(t0, 5), 30 * pow(t0, 4), 20 * pow(t0, 3), 12 * pow(t0, 2), 6 * t0, 2, 0, 0;
    A.block<1, 8>(1, 0) = values2.transpose();
    b(1) = 0;

    // Constraint #3: Jerk is zero at the start position
    Eigen::VectorXd values3(8);
    values3 << 210 * pow(t0, 4), 120 * pow(t0, 3), 60 * pow(t0, 2), 24 * t0, 6, 0, 0, 0;
    A.block<1, 8>(2, 0) = values3.transpose();
    b(2) = 0;

    // Constraint #4: Velocity is zero at the end position
    Eigen::VectorXd values4(8);
    double tf = normalizedStartTimes.back();
    values4 << 7 * pow(tf, 6), 6 * pow(tf, 5), 5 * pow(tf, 4), 4 * pow(tf, 3), 3 * pow(tf, 2), 2 * tf, 1, 0;
    A.block<1, 8>(3, 8 * (m - 1)) = values4.transpose();
    b(3) = 0;

    // Constraint #5: Acceleration is zero at the end position
    Eigen::VectorXd values5(8);
    values5 << 42 * pow(tf, 5), 30 * pow(tf, 4), 20 * pow(tf, 3), 12 * pow(tf, 2), 6 * tf, 2, 0, 0;
    A.block<1, 8>(4, 8 * (m - 1)) = values5.transpose();
    b(4) = 0;

    // Constraint #6: Jerk is zero at the end position
    Eigen::VectorXd values6(8);
    values6 << 210 * pow(tf, 4), 120 * pow(tf, 3), 60 * pow(tf, 2), 24 * tf, 6, 0, 0, 0;
    A.block<1, 8>(5, 8 * (m - 1)) = values6.transpose();
    b(5) = 0;

    // CONSTRAINT ACCOUNTING: 6 constraints have been added so far

    // The next N constraints (N is the number of waypoints) come from the waypoint positions themselves
    for (int i = 0; i < waypoints.size(); i++) {

        Eigen::VectorXd values(8);
        double t = normalizedStartTimes[i];
        values << pow(t, 7), pow(t, 6), pow(t, 5), pow(t, 4), pow(t, 3), pow(t, 2), t, 1;


        // Handle the last waypoint separately when inserting values into the `A` matrix
        if (i == waypoints.size() - 1) {

            // The last waypoint
            double t = normalizedStartTimes.back();
            A.block<1, 8>(6 + i, 8 * (waypoints.size() - 2)) = values.transpose();

        } else {

            // All other waypoints
            A.block<1, 8>(6 + i, 8 * i) = values.transpose();
        }

        switch (component) {
        case 'x':
            b(6 + i) = waypoints[i].x();
            break;
        case 'y':
            b(6 + i) = waypoints[i].y();
            break;
        case 'z':
            b(6 + i) = waypoints[i].z();
            break;
        default:
            throw std::invalid_argument("Invalid component. Use 'x', 'y', or 'z'.");
        }

    }

    // CONSTRAINT ACCOUNTING: 6 + N constraints have been added so far

    // The remaining constraints come from enforcing continuity of position, velocity, ..., snap,
    // crackle, pop (motion derivatives 0 to 6) at each of the interior waypoints.
    for (int i = 1; i < waypoints.size() - 1; i++) {
        double t1 = normalizedStartTimes[i];

        // Position continuity
        Eigen::VectorXd values1(8);
        values1 << pow(t1, 7), pow(t1, 6), pow(t1, 5), pow(t1, 4), pow(t1, 3), pow(t1, 2), t1, 1;
        A.block<1, 8>(6 + waypoints.size() + 7 * (i - 1), 8 * (i - 1)) = values1.transpose();
        A.block<1, 8>(6 + waypoints.size() + 7 * (i - 1), 8 * i) = -values1.transpose();
        b(6 + waypoints.size() + 7 * (i - 1)) = 0;

        // Velocity continuity
        Eigen::VectorXd values2(8);
        values2 << 7 * pow(t1, 6), 6 * pow(t1, 5), 5 * pow(t1, 4), 4 * pow(t1, 3), 3 * pow(t1, 2), 2 * t1, 1, 0;
        A.block<1, 8>(6 + waypoints.size() + 7 * (i - 1) + 1, 8 * (i - 1)) = values2.transpose();
        A.block<1, 8>(6 + waypoints.size() + 7 * (i - 1) + 1, 8 * i) = -values2.transpose();
        b(6 + waypoints.size() + 7 * (i - 1) + 1) = 0;

        // Acceleration continuity
        Eigen::VectorXd values3(8);
        values3 << 42 * pow(t1, 5), 30 * pow(t1, 4), 20 * pow(t1, 3), 12 * pow(t1, 2), 6 * t1, 2, 0, 0;
        A.block<1, 8>(6 + waypoints.size() + 7 * (i - 1) + 2, 8 * (i - 1)) = values3.transpose();
        A.block<1, 8>(6 + waypoints.size() + 7 * (i - 1) + 2, 8 * i) = -values3.transpose();
        b(6 + waypoints.size() + 7 * (i - 1) + 2) = 0;

        // Jerk continuity
        Eigen::VectorXd values4(8);
        values4 << 210 * pow(t1, 4), 120 * pow(t1, 3), 60 * pow(t1, 2), 24 * t1, 6, 0, 0, 0;
        A.block<1, 8>(6 + waypoints.size() + 7 * (i - 1) + 3, 8 * (i - 1)) = values4.transpose();
        A.block<1, 8>(6 + waypoints.size() + 7 * (i - 1) + 3, 8 * i) = -values4.transpose();
        b(6 + waypoints.size() + 7 * (i - 1) + 3) = 0;

        // Snap continuity
        Eigen::VectorXd values5(8);
        values5 << 840 * pow(t1, 3), 360 * pow(t1, 2), 120 * t1, 24, 0, 0, 0, 0;
        A.block<1, 8>(6 + waypoints.size() + 7 * (i - 1) + 4, 8 * (i - 1)) = values5.transpose();
        A.block<1, 8>(6 + waypoints.size() + 7 * (i - 1) + 4, 8 * i) = -values5.transpose();
        b(6 + waypoints.size() + 7 * (i - 1) + 4) = 0;

        // Crackle continuity
        Eigen::VectorXd values6(8);
        values6 << 2520 * pow(t1, 2), 720 * t1, 120, 0, 0, 0, 0, 0;
        A.block<1, 8>(6 + waypoints.size() + 7 * (i - 1) + 5, 8 * (i - 1)) = values6.transpose();
        A.block<1, 8>(6 + waypoints.size() + 7 * (i - 1) + 5, 8 * i) = -values6.transpose();
        b(6 + waypoints.size() + 7 * (i - 1) + 5) = 0;

        // Pop continuity
        Eigen::VectorXd values7(8);
        values7 << 5040 * t1, 720, 0, 0, 0, 0, 0, 0;
        A.block<1, 8>(6 + waypoints.size() + 7 * (i - 1) + 6, 8 * (i - 1)) = values7.transpose();
        A.block<1, 8>(6 + waypoints.size() + 7 * (i - 1) + 6, 8 * i) = -values7.transpose();
        b(6 + waypoints.size() + 7 * (i - 1) + 6) = 0;

    }

    // CONSTRAINT ACCOUNTING: 6 + N + 7 * (N - 2) constraints have been added so far
    // This is the same as 6 + N + 7N - 14 = 8N - 8 constraints
    // This is the same as 8 * (n - 1) constraints, which is the required number
    // So the system is ready to be solved

    return A.colPivHouseholderQr().solve(b);
}


/**
 * @brief Evaluates the trajectory at a given time.
 * @param coeffsX The coefficients of the polynomial trajectory in the X direction.
 * @param coeffsY The coefficients of the polynomial trajectory in the Y direction.
 * @param coeffsZ The coefficients of the polynomial trajectory in the Z direction.
 * @param startTimes The start times of each segment of the trajectory.
 * @param t The time at which to evaluate the trajectory.
 * @return A 3D point representing the position of the trajectory at time t.
 */
Eigen::Vector3f evaluateTrajectory(const Eigen::VectorXd& coeffsX, const Eigen::VectorXd& coeffsY, const Eigen::VectorXd& coeffsZ, const std::vector<double>& startTimes, double t) {
    // Find the segment corresponding to time t
    int segment = 0;
    for (size_t i = 0; i < startTimes.size() - 1; ++i) {
        if (t >= startTimes[i] && t <= startTimes[i + 1]) {
            segment = i;
            break;
        }
    }

    // Normalize the time t within the segment
    double t0 = startTimes[segment];
    double t1 = startTimes[segment + 1];
    double normalizedT = (t - startTimes.front()) / (startTimes.back() - startTimes.front());

    // Evaluate the polynomial at the normalized time
    Eigen::VectorXd T(8);
    T << pow(normalizedT, 7), pow(normalizedT, 6), pow(normalizedT, 5), pow(normalizedT, 4), pow(normalizedT, 3), pow(normalizedT, 2), normalizedT, 1;

    Eigen::Vector3f point;
    point.x() = T.dot(coeffsX.segment<8>(8 * segment));
    point.y() = T.dot(coeffsY.segment<8>(8 * segment));
    point.z() = T.dot(coeffsZ.segment<8>(8 * segment));

    return point;
}