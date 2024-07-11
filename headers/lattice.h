#pragma once

#include <vector>
#include <map>
#include <Eigen/Dense>
#include <iostream>
#include <string>
#include "env_data.h"

/**
 * @class Lattice
 * @brief Represents a lattice-based representation of the environment.
 */
class Lattice {
public:

	/**
	 * @brief Constructs a Lattice.
	 * @param obstacleParser An ObstacleParser object containing obstacle data.
	 * @param resolution The resolution of the lattice.
	 */
	Lattice(const ObstacleParser& obstacleParser, float resolution);
	
	/**
	 * @brief Gets the free space points in the lattice.
	 * @return A vector of 3D points representing the free space points.
	 */
	std::vector<Eigen::Vector3f> getFreeSpacePoints() const;
	
	/**
	 * @brief Gets the edges of the lattice.
	 * @return A map of edges between free space points.
	 */
	std::map<int, std::map<int, float>> getEdges() const;
	
	/**
	 * @brief Gets the resolution of the lattice.
	 * @return The resolution of the lattice.
	 */
	float getResolution() const;

	/**
	 * @brief Performs an A* search to find a path between two points.
	 * @param start The start point of the path.
	 * @param goal The goal point of the path.
	 * @return A vector of 3D points representing the path.
	 */
	std::vector<Eigen::Vector3f> aStarSearch(const Eigen::Vector3f& start, const Eigen::Vector3f& goal) const;

private:
	float resolution; ///< Resolution of the lattice.
	Eigen::Vector3f lowerBounds; ///< Lower bounds of the environment.
	Eigen::Vector3f upperBounds; ///< Upper bounds of the environment.
	std::vector<Eigen::Vector3f> freeSpacePoints; ///< Free space points in the lattice.
	std::map<int, std::map<int, float>> edges; ///< Edges between free space points.
	std::vector<Obstacle> obstacles; ///< Obstacles in the environment.

	/**
	 * @brief Computes the free space points in the lattice.
	 */
	void computeFreeSpacePoints();

	/**
	 * @brief Computes the edges of the lattice.
	 */
	void buildGraph();

	/**
	 * @brief Checks if a point is in collision with the obstacles.
	 * @param point The point to check.
	 * @return True if the point is in collision, false otherwise.
	 */
	bool isCollision(const Eigen::Vector3f& point) const;

	/**
	 * @brief Heuristic function for A* search.
	 * @param point1 The first point.
	 * @param point2 The second point.
	 * @return The heuristic value.
	 */
	float heuristic(const Eigen::Vector3f& point1, const Eigen::Vector3f& point2) const;

	/**
	 * @brief Finds the nearest node in the lattice to a given point.
	 * @param point The point to find the nearest node to.
	 * @return The index of the nearest node.
	 */
	int findNearestNode(const Eigen::Vector3f& point) const;


};