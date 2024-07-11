/**
 * @file main.cpp
 * @brief Main function to test the ObstacleParser, Lattice, and Trajectory classes.
 * @version 1.0
 * @date 2021-10-13
 */

#include "../headers/env_data.h"
#include "../headers/lattice.h"
#include "../headers/trajectory.h"
#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <fstream>


/**
 * @brief Saves the obstacles to a CSV file.
 * @param obstacles A vector of Obstacle objects.
 * @param filename Name of the CSV file to save the obstacles.
 */
void saveObstacles(const std::vector<Obstacle>& obstacles, const std::string& filename) {
    std::ofstream file(filename, std::ofstream::trunc);

    file << "minX,maxX,minY,maxY,minZ,maxZ\n";
    for (const auto& obstacle : obstacles) {
        file << obstacle.getMinX() << "," << obstacle.getMaxX() << "," << obstacle.getMinY() << "," << obstacle.getMaxY() << "," << obstacle.getMinZ() << "," << obstacle.getMaxZ() << "\n";
    }
    file.close();
}

/**
 * @brief Saves the lattice points to a CSV file.
 * @param points A vector of 3D points representing the lattice points.
 * @param filename Name of the CSV file to save the lattice points.
 */
void saveLatticePoints(const std::vector<Eigen::Vector3f>& points, const std::string& filename) {
    std::ofstream file(filename, std::ofstream::trunc);

    file << "x,y,z\n";
    for (const auto& point : points) {
        file << point.x() << "," << point.y() << "," << point.z() << "\n";
    }
    file.close();
}

/**
 * @brief Saves the edges to a CSV file.
 * @param points A vector of 3D points representing the lattice points.
 * @param edges A map of edges between lattice points.
 * @param filename Name of the CSV file to save the edges.
 */
void saveEdges(const std::vector<Eigen::Vector3f>& points, const std::map<int, std::map<int, float>>& edges, const std::string& filename) {
    std::ofstream file(filename, std::ofstream::trunc);

    file << "x1,y1,z1,x2,y2,z2\n";
    for (const auto& edge : edges) {
        int point1 = edge.first;
        for (const auto& connection : edge.second) {
            int point2 = connection.first;
            file << points[point1].x() << "," << points[point1].y() << "," << points[point1].z() << ","
                << points[point2].x() << "," << points[point2].y() << "," << points[point2].z() << "\n";
        }
    }
    file.close();
}

/**
 * @brief Saves the path to a CSV file.
 * @param path A vector of 3D points representing the path.
 * @param filename Name of the CSV file to save the path.
 */
void savePath(const std::vector<Eigen::Vector3f>& path, const std::string& filename) {
    std::ofstream file(filename, std::ofstream::trunc);

    file << "x,y,z\n";
    for (const auto& point : path) {
        file << point.x() << "," << point.y() << "," << point.z() << "\n";
    }
    file.close();
}

/**
 * @brief Saves the trajectory to a CSV file.
 * @param trajectory A vector of 3D points representing the trajectory.
 * @param filename Name of the CSV file to save the trajectory.
 */
void saveTrajectory(const std::vector<Eigen::Vector3f>& trajectory, const std::string& filename) {
    std::ofstream file(filename, std::ofstream::trunc); 

    file << "x,y,z\n";
    for (const auto& point : trajectory) {
        file << point.x() << "," << point.y() << "," << point.z() << "\n";
    }
    file.close();
}

/**
 * @brief Main function to test the ObstacleParser, Lattice, and Trajectory classes.
 * @return int Status code.
 */
int main() {
    // =============================
    // Test ObstacleParser class
    // =============================

    // Create an ObstacleParser object and read the obstacles from the CSV file
    ObstacleParser parser("../data/obstacles.csv");

    // Get the obstacles and bounds
    std::vector<Obstacle> obstacles = parser.getObstacles();
    const std::vector<float>& bounds = parser.getBounds();

    // Print the obstacles and bounds
    std::cout << "Number of obstacles: " << obstacles.size() << std::endl;
    std::cout << "Bounds: " 
              << bounds[0] << ", " << bounds[1] << ", " 
              << bounds[2] << ", " << bounds[3] << ", " 
              << bounds[4] << ", " << bounds[5] << std::endl;

    // =============================
    // Test Lattice class
    // =============================

    // Specify the resolution of the lattice
    float resolutionOfLattice = 30.0;

    // Create a Lattice object
    Lattice lattice(parser, resolutionOfLattice);

    // Validate the lattice construction
    std::cout << "Lattice created with resolution: " << lattice.getResolution() << std::endl;
    std::cout << "Number of free space points: " << lattice.getFreeSpacePoints().size() << std::endl;
    std::cout << "Number of edges: " << lattice.getEdges().size() << std::endl;

    // Save the obstacles and lattice points to files
    saveObstacles(parser.getObstacles(), "../data/obstacles_output.csv");
    saveLatticePoints(lattice.getFreeSpacePoints(), "../data/lattice_points_output.csv");
    saveEdges(lattice.getFreeSpacePoints(), lattice.getEdges(), "../data/edges_output.csv");

    // =============================
    // Test A* search
    // =============================

    // Specify the start and goal points
    Eigen::Vector3f start(10, 10, 10);
    Eigen::Vector3f goal(90, 90, 90);

    // Perform A* search
    std::vector<Eigen::Vector3f> path = lattice.aStarSearch(start, goal);

    // Check if path is found
    if (!path.empty()) {
        std::cout << "Path found:" << std::endl;
        for (const auto& point : path) {
            std::cout << "(" << point.x() << ", " << point.y() << ", " << point.z() << ")" << std::endl;
        }
    } else {
        std::cout << "No path found." << std::endl;
    }

    // Save the path to a CSV file
    savePath(path, "../data/path_output.csv");

    // =============================
    // Test trajectory generation
    // =============================

    if (!path.empty()) {
        // Assign start times to each segment of the path
        float averageVelocity = 1.0; 
        std::vector<double> startTimes = assignSegmentStartTimes(path, averageVelocity);

        // Solve for x, y, z components separately
        Eigen::VectorXd coeffsX = solveCoefficients(path, startTimes, 'x');
        Eigen::VectorXd coeffsY = solveCoefficients(path, startTimes, 'y');
        Eigen::VectorXd coeffsZ = solveCoefficients(path, startTimes, 'z');

        // Print the solutions
        std::cout << "Coefficients for x: " << coeffsX.transpose() << std::endl;
        std::cout << "Coefficients for y: " << coeffsY.transpose() << std::endl;
        std::cout << "Coefficients for z: " << coeffsZ.transpose() << std::endl;

        // Evaluate the trajectory at various time points and save the results
        std::vector<Eigen::Vector3f> trajectory;
        double totalTime = startTimes.back();
        int numSamples = 100;
        for (int i = 0; i <= numSamples; ++i) {
            double t = totalTime * i / numSamples;
            Eigen::Vector3f point = evaluateTrajectory(coeffsX, coeffsY, coeffsZ, startTimes, t);
            trajectory.push_back(point);
        }

        // Save the trajectory to a CSV file
        saveTrajectory(trajectory, "../data/trajectory_output.csv");

        // Print the trajectory
        std::cout << "Trajectory points:" << std::endl;
        for (const auto& point : trajectory) {
            std::cout << "(" << point.x() << ", " << point.y() << ", " << point.z() << ")" << std::endl;
        }
    }

    return 0;
}
