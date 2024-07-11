/**
 * @file lattice.cpp
 * @brief Implementation of the Lattice class.
 */

#include "../headers/lattice.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>
#include <queue>
#include <map>
#include <algorithm>
#include <limits>

/**
 * @brief Represents a node in the A* search algorithm.
 */
struct Node {

	int index;
	float cost;

	bool operator<(const Node& other) const {
		return cost > other.cost;
	}
};

/**
 * @brief Constructs a Lattice.
 * @param obstacleParser An ObstacleParser object containing obstacle data.
 * @param resolution The resolution of the lattice.
 */
Lattice::Lattice(const ObstacleParser& obstacleParser, float resolution)
	: resolution(resolution), obstacles(obstacleParser.getObstacles()) {
	lowerBounds = Eigen::Vector3f(obstacleParser.getBounds()[0], obstacleParser.getBounds()[2], obstacleParser.getBounds()[4]);
	upperBounds = Eigen::Vector3f(obstacleParser.getBounds()[1], obstacleParser.getBounds()[3], obstacleParser.getBounds()[5]);
	computeFreeSpacePoints();
	buildGraph();
}
/**
 * @brief Computes the free space points in the lattice.
 */
void Lattice::computeFreeSpacePoints() {
	for (float x = lowerBounds.x(); x <= upperBounds.x(); x += resolution) {
		for (float y = lowerBounds.y(); y <= upperBounds.y(); y += resolution) {
			for (float z = lowerBounds.z(); z <= upperBounds.z(); z += resolution) {
				Eigen::Vector3f point(x, y, z);
				if (!isCollision(point)) {
					freeSpacePoints.push_back(point);
				}
			}
		}
	}
}

/**
 * @brief Computes the edges of the lattice.
 */
void Lattice::buildGraph() {
	float radius = 1.0001f * resolution; // Distance between two adjacent points in the lattice, no diagonals for now; add a small buffer
	for (int i = 0; i < freeSpacePoints.size(); i++) {
		for (int j = i + 1; j < freeSpacePoints.size(); j++) {
			float distance = (freeSpacePoints[i] - freeSpacePoints[j]).norm();
			if (distance <= radius) {
				edges[i][j] = distance;
				edges[j][i] = distance;
			}
			
		}
    std::cout << "Number of edges: " << edges.size() << std::endl;
	}
}

/**
 * @brief Checks if a point is in collision with any obstacles.
 * @param point The point to check.
 * @return True if the point is in collision, false otherwise.
 */
bool Lattice::isCollision(const Eigen::Vector3f& point) const {
	for (const Obstacle& obstacle : obstacles) {
		if (obstacle.isCollision(point)) {
			return true;
		}
	}
	return false;
}

/**
 * @brief Gets the free space points in the lattice.
 * @return A vector of 3D points representing the free space points.
 */
std::vector<Eigen::Vector3f> Lattice::getFreeSpacePoints() const {
	return freeSpacePoints;
}

/**
 * @brief Gets the edges of the lattice.
 * @return A map of edges between free space points.
 */
std::map<int, std::map<int, float>> Lattice::getEdges() const {
	return edges;
}

/**
 * @brief Gets the resolution of the lattice.
 * @return The resolution of the lattice.
 */
float Lattice::getResolution() const {
	return resolution;
}

/**
 * @brief Calculates the Euclidean distance between two points.
 * @param point1 The first point.
 * @param point2 The second point.
 * @return The Euclidean distance between the two points.
 */
float Lattice::heuristic(const Eigen::Vector3f& point1, const Eigen::Vector3f& point2) const {
	return (point1 - point2).norm();
}

/**
 * @brief Finds the index of the nearest node to a given point.
 * @param point The point to find the nearest node to.
 * @return The index of the nearest node.
 */
int Lattice::findNearestNode(const Eigen::Vector3f& point) const {
	int nearestIdx = -1;
	float minDist = std::numeric_limits<float>::max();
	for (int i = 0; i < freeSpacePoints.size(); ++i) {
		float dist = (freeSpacePoints[i] - point).norm();
		if (dist < minDist) {
			minDist = dist;
			nearestIdx = i;
		}
	}
	return nearestIdx;
}

/**
 * @brief Performs an A* search to find a path between two points.
 * @param start The start point of the path.
 * @param goal The goal point of the path.
 * @return A vector of 3D points representing the path.
 */
std::vector<Eigen::Vector3f> Lattice::aStarSearch(const Eigen::Vector3f& start, const Eigen::Vector3f& goal) const {

	// Note: Nodes are indexed by their position in the freeSpacePoints vector

	// Find the nearest nodes to the start and goal points
	int startIdx = findNearestNode(start);
	int goalIdx = findNearestNode(goal);

	// Priority queue to store the nodes to be expanded
	std::priority_queue<Node> openSet;

	// Maps to store the g and f scores of each node
	std::map<int, float> gScore;
	std::map<int, float> fScore;

	// Map to store the parent of each node
	std::map<int, int> cameFrom;

	// Initialize the start point
	openSet.push({ startIdx, heuristic(start, goal) });
	gScore[startIdx] = 0;
	fScore[startIdx] = heuristic(start, goal);

	// Main loop
	while (!openSet.empty()) {

		Node currentNode = openSet.top(); // Get the lowest cost node
		openSet.pop(); // Remove the node from the open set
		int current = currentNode.index;

		// Check if the goal has been reached
		if (current == goalIdx) {
			// Reconstruct the path
			std::vector<Eigen::Vector3f> path;
			while (cameFrom.find(current) != cameFrom.end()) {
				path.push_back(freeSpacePoints[current]);
				current = cameFrom[current];
			}
			path.push_back(start);
			std::reverse(path.begin(), path.end());
			return path;
		}

		// Explore the neighbors of the current node
		for (const auto& neighbor : edges.at(current)) {
			int neighborIdx = neighbor.first;
			float tentative_gScore = gScore[current] + neighbor.second;

			// Check if this path to the neighbor is better
			if (gScore.find(neighborIdx) == gScore.end() || tentative_gScore < gScore[neighborIdx]) {
				cameFrom[neighborIdx] = current;
				gScore[neighborIdx] = tentative_gScore;
				fScore[neighborIdx] = gScore[neighborIdx] + heuristic(freeSpacePoints[neighborIdx], goal);
				openSet.push({ neighborIdx, fScore[neighborIdx] });
			}
		}
	}

	return {}; // Return an empty vector if no path is found
}