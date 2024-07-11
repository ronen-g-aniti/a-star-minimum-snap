#pragma once

#include <vector>
#include <string>
#include <Eigen/Dense>

/**
 * @class Obstacle
 * @brief Represents a 3D obstacle defined by its center position and half-sizes along each axis.
 */
class Obstacle {
public:
    /**
     * @brief Constructs an Obstacle.
     * @param posX X-coordinate of the obstacle's center.
     * @param posY Y-coordinate of the obstacle's center.
     * @param posZ Z-coordinate of the obstacle's center.
     * @param halfsizeX Half-size of the obstacle along the X-axis.
     * @param halfsizeY Half-size of the obstacle along the Y-axis.
     * @param halfsizeZ Half-size of the obstacle along the Z-axis.
     */
    Obstacle(float posX, float posY, float posZ, float halfsizeX, float halfsizeY, float halfsizeZ);

    /**
     * @brief Checks if a point is in collision with the obstacle.
     * @param point A 3D point to check for collision.
     * @return A float representing the collision status (0 if no collision, 1 if collision).
     */
    float isCollision(const Eigen::Vector3f& point) const;
    
    /**
     * @brief Gets the minimum X coordinate of the obstacle.
     * @return Minimum X coordinate.
     */  
    float getMinX() const;

    /**
     * @brief Gets the maximum X coordinate of the obstacle.
     * @return Maximum X coordinate.
     */    
    float getMaxX() const;

    /**
     * @brief Gets the minimum Y coordinate of the obstacle.
     * @return Minimum Y coordinate.
     */
    float getMinY() const;
    
    /**
     * @brief Gets the maximum Y coordinate of the obstacle.
     * @return Maximum Y coordinate.
     */ 
    float getMaxY() const;
    
    /**
     * @brief Gets the minimum Z coordinate of the obstacle.
     * @return Minimum Z coordinate.
     */
    float getMinZ() const;
    
    /**
     * @brief Gets the maximum Z coordinate of the obstacle.
     * @return Maximum Z coordinate.
     */
    float getMaxZ() const;

private:
    float posX; ///< X-coordinate of the obstacle's center.
    float posY; ///< Y-coordinate of the obstacle's center.
    float posZ; ///< Z-coordinate of the obstacle's center.
    float halfSizeX; ///< Half-size of the obstacle along the X-axis.
    float halfSizeY; ///< Half-size of the obstacle along the Y-axis.
    float halfSizeZ; ///< Half-size of the obstacle along the Z-axis.
};

/**
 * @class ObstacleParser
 * @brief Parses a CSV file containing obstacle data.
 */
class ObstacleParser {
public:
    /**
     * @brief Constructs an ObstacleParser.
     * @param filename Name of the CSV file containing obstacle data.
     */
    ObstacleParser(const std::string& filename);

    /**
     * @brief Gets the obstacles parsed from the CSV file.
     * @return A vector of Obstacle objects.
     */
    std::vector<Obstacle> getObstacles() const;
    
    /**
     * @brief Gets the bounds of the environment.
     * @return A six-element vector representing the bounds of the environment: minX, maxX, minY, maxY, minZ, maxZ.
     */
    const std::vector<float>& getBounds() const;

private:

    /**
     * @brief Parses the CSV file containing obstacle data.
     * @param filename Name of the CSV file containing obstacle data.
     */
    void parseCSV(const std::string& filename);
    
    /**
     * @brief Computes the bounds of the environment.
     */
    void computeBounds();

    
    std::vector<Obstacle> obstacles; ///< Vector of Obstacle objects.
    std::vector<float> bounds; ///< Bounds of the environment: minX, maxX, minY, maxY, minZ, maxZ.
};