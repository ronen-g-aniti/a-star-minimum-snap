/**
 * @file env_data.cpp
 * @brief Implementation of the Obstacle and ObstacleParser classes.
 */

#include "../headers/env_data.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <limits>
#include <algorithm>
#include <Eigen/Dense>

/**
 * @brief Constructs an Obstacle.
 * @param posX X-coordinate of the obstacle's center.
 * @param posY Y-coordinate of the obstacle's center.
 * @param posZ Z-coordinate of the obstacle's center.
 * @param halfSizeX Half-size of the obstacle along the X-axis.
 * @param halfSizeY Half-size of the obstacle along the Y-axis.
 * @param halfSizeZ Half-size of the obstacle along the Z-axis.
 */
Obstacle::Obstacle(float posX, float posY, float posZ, float halfSizeX, float halfSizeY, float halfSizeZ)
    : posX(posX), posY(posY), posZ(posZ), halfSizeX(halfSizeX), halfSizeY(halfSizeY), halfSizeZ(halfSizeZ) {}


/**
 * @brief Parses a CSV file containing obstacle data.
 * @param filename Name of the CSV file containing obstacle data.
 */
float Obstacle::isCollision(const Eigen::Vector3f& point) const {
    return (point.x() >= getMinX() && point.x() <= getMaxX() &&
        point.y() >= getMinY() && point.y() <= getMaxY() &&
        point.z() >= getMinZ() && point.z() <= getMaxZ());
}

/**
 * @brief Gets the minimum X coordinate of the obstacle.
 * @return Minimum X coordinate.
 */
float Obstacle::getMinX() const { return posX - halfSizeX; }

/**
 * @brief Gets the maximum X coordinate of the obstacle.
 * @return Maximum X coordinate.
 */
float Obstacle::getMaxX() const { return posX + halfSizeX; }

/**
 * @brief Gets the minimum Y coordinate of the obstacle.
 * @return Minimum Y coordinate.
 */
float Obstacle::getMinY() const { return posY - halfSizeY; }

/**
 * @brief Gets the maximum Y coordinate of the obstacle.
 * @return Maximum Y coordinate.
 */
float Obstacle::getMaxY() const { return posY + halfSizeY; }

/**
 * @brief Gets the minimum Z coordinate of the obstacle.
 * @return Minimum Z coordinate.
 */
float Obstacle::getMinZ() const { return posZ - halfSizeZ; }

/**
 * @brief Gets the maximum Z coordinate of the obstacle.
 * @return Maximum Z coordinate.
 */
float Obstacle::getMaxZ() const { return posZ + halfSizeZ; }

/**
 * @brief Constructs an ObstacleParser.
 * @param filename Name of the CSV file containing obstacle data.
 */
ObstacleParser::ObstacleParser(const std::string& filename) {
    parseCSV(filename);
    computeBounds();
}

/**
 * @brief Parses a CSV file containing obstacle data.
 * @param filename Name of the CSV file containing obstacle data.
 */
void ObstacleParser::parseCSV(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    std::string line;

    while (std::getline(file, line)) {
        std::istringstream ss(line);
        std::string token;
        std::vector<float> values;

        while (std::getline(ss, token, ',')) {
            values.push_back(std::stof(token));
        }
        obstacles.emplace_back(values[0], values[1], values[2], values[3], values[4], values[5]);
        
    }
}

/**
 * @brief Computes the bounds of the obstacles.
 */
void ObstacleParser::computeBounds() {
    float minX = std::numeric_limits<float>::max();
    float maxX = std::numeric_limits<float>::lowest();
    float minY = std::numeric_limits<float>::max();
    float maxY = std::numeric_limits<float>::lowest();
    float minZ = std::numeric_limits<float>::max();
    float maxZ = std::numeric_limits<float>::lowest();

    for (const auto& obstacle : obstacles) {
        minX = std::min(minX, obstacle.getMinX());
        maxX = std::max(maxX, obstacle.getMaxX());
        minY = std::min(minY, obstacle.getMinY());
        maxY = std::max(maxY, obstacle.getMaxY());
        minZ = std::min(minZ, obstacle.getMinZ());
        maxZ = std::max(maxZ, obstacle.getMaxZ());
    }

    bounds = { minX, maxX, minY, maxY, minZ, maxZ };
}

/**
 * @brief Gets the bounds of the environment.
 * @return A six-element vector representing the bounds of the environment: minX, maxX, minY, maxY, minZ, maxZ.
 */
const std::vector<float>& ObstacleParser::getBounds() const {
    return bounds;
}

/**
 * @brief Gets the obstacles parsed from the CSV file.
 * @return A vector of Obstacle objects.
 */
std::vector<Obstacle> ObstacleParser::getObstacles() const {
    return obstacles;
}