#pragma once

#include <vector>
#include <Eigen/Dense>

/**
 * @brief Calculates the Euclidean distance between two points.
 * @param point1 The first point.
 * @param point2 The second point.
 * @return The Euclidean distance between the two points.
 */
double calculateDistance(const Eigen::Vector3f& point1, const Eigen::Vector3f& point2);

/**
 * @brief Assigns start times to each segment of the trajectory.
 * @param waypoints The waypoints of the trajectory.
 * @param averageVelocity The average velocity of the trajectory.
 * @return A vector of start times for each segment of the trajectory.
 */
std::vector<double> assignSegmentStartTimes(const std::vector<Eigen::Vector3f>& waypoints, const float averageVelocity);

/**
 * @brief Solves for the coefficients of the polynomial trajectory.
 * @param waypoints The waypoints of the trajectory.
 * @param startTimes The start times of each segment of the trajectory.
 * @param component The component of the trajectory to solve for (X, Y, or Z).
 * @return A vector of coefficients for the polynomial trajectory.
 */
Eigen::VectorXd solveCoefficients(const std::vector<Eigen::Vector3f>& waypoints, const std::vector<double>& startTimes, char component);


/**
 * @brief Normalizes the start times of the trajectory.
 * @param startTimes The start times of the trajectory.
 * @return A vector of normalized start times.
 */
std::vector<double> normalizeStartTimes(const std::vector<double>& startTimes);

/**
 * @brief Evaluates the trajectory at a given time.
 * @param coeffsX The coefficients of the polynomial trajectory for the X component.
 * @param coeffsY The coefficients of the polynomial trajectory for the Y component.
 * @param coeffsZ The coefficients of the polynomial trajectory for the Z component.
 * @param startTimes The start times of each segment of the trajectory.
 * @param t The time at which to evaluate the trajectory.
 * @return A 3D point representing the position of the trajectory at time t.
 */
Eigen::Vector3f evaluateTrajectory(const Eigen::VectorXd& coeffsX, const 

/**
 * @brief Saves the trajectory to a CSV file.
 * @param trajectory The trajectory to save.
 * @param filename The name of the file to save the trajectory to.
 */
Eigen::VectorXd& coeffsY, const Eigen::VectorXd& coeffsZ, const 

/**
 * @brief Saves the trajectory to a CSV file.
 * @param trajectory The trajectory to save.
 * @param filename The name of the file to save the trajectory to.
 */
std::vector<double>& startTimes, double t);