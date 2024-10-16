#ifndef PATHFINDING_H
#define PATHFINDING_H

#include <mutex>
#include <climits>  // for INT_MAX
#include <cmath>
#include <queue>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

/*
To do:
    - add section to use hazard map
    - improve transform calculation functions to include rotation
    - improve efficiency by using binary trees instead of vectors where viable (closed list, maybe the open list using a pair?)
    - filter out non-critical poses from array (i.e. where the yaw changes)
    - add path smoothing?
*/


/*!
 * \brief   A* pathfinding class for K-10 project. 
 * \details To use: \n
 *          - create pointer to PathFinding object, passing though the starting occupancy and hazard maps as arguments \n
 *          - in occupancy map and hazard map callbacks, call updateOccupancyMap() and updateHazardMap() respectively to update local data \n
 *          - find path between two poses with AStar(startPose, goalPose) \n
 *          - function returns path as geometry_msgs::msg::PoseArray
 *          - publish path with publishMarkers(path)
 * \file    PathFinding.h
 * \author  J. Tarbath (14163885) 
 * \date    xx/10/2024
 * \bug     Nil known as of xx/10/2024
 */
class PathFinding : public rclcpp::Node {
public:
    PathFinding::PathFinding(const nav_msgs::msg::OccupancyGrid &mapOccupancy, const nav_msgs::msg::OccupancyGrid &mapHazard);

    /**
     * @brief publish's path markers
     * 
     * @param path 
     */
    void publishMarkers(const geometry_msgs::msg::PoseArray &path);

    /**
     * @brief Get the occupancy map member variable
     * 
     * @return nav_msgs::msg::OccupancyGrid mapOccupancy_
     */
    nav_msgs::msg::OccupancyGrid getOccupancyMap(void);

    /**
     * @brief Update occupancy map (mapOccupancy_) member variable
     * 
     * @param msg 
     */
    void updateOccupancyMap(const nav_msgs::msg::OccupancyGrid &msg);

    /**
     * @brief Get the hazard map member variable
     * 
     * @return nav_msgs::msg::OccupancyGrid mapHazard_
     */
    nav_msgs::msg::OccupancyGrid getHazardMap(void);

    /**
     * @brief Update hazard map (mapHazard_) member variable
     * 
     * @param msg 
     */
    void updateHazardMap(const nav_msgs::msg::OccupancyGrid &msg);

    /**
     * @brief Calculates path using A* algorithm
     * 
     * @param startPose geometry_msgs::msg::Pose
     * @param goalPose geometry_msgs::msg::Pose
     * @return geometry_msgs::msg::PoseArray path waypoints
     */
    geometry_msgs::msg::PoseArray AStar(const geometry_msgs::msg::Pose &startPose, const geometry_msgs::msg::Pose &goalPose);

private:
    std::mutex mtx_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_; //!< Marker publisher

    const int CELL_OCCUPIED_THRESHOLD;  //!< Cell occupied threshold value
    const int CELL_VALUE_MIN;           //!< Cell min value
    const int CELL_VALUE_MAX;           //!< Cell max value

    nav_msgs::msg::OccupancyGrid mapOccupancy_; //!< Occupancy map data
    nav_msgs::msg::OccupancyGrid mapHazard_;    //!< Hazard map data

    /**
     * @brief cell data struct
     * 
     */
    struct CellData {
            unsigned int index = 0;         // index of cell
            bool visited = 0;               // has this cell been visited (g, h and f cost calculated)
            int occupancy = 0;              // occupancy value
            int hazard = 0;                 // hazard value
            unsigned int parentCell = 0;    // index of parent cell
            double gCost = 0;               // (cost to get there from start)
            double hCost = 0;               // (cost to get to goal from here)
            double fCost = 0;               // (G cost + H cost + hazard value)

            CellData(unsigned int &index, bool &visited, int &occupancy, int &hazard, unsigned int &parentCell, double &gCost, double &hCost, double &fCost)
                : index(index), visited(visited), occupancy(occupancy), hazard(hazard), parentCell(parentCell), gCost(gCost), hCost(hCost), fCost(fCost) {}

            CellData() : index(0), visited(false), occupancy(0), hazard(0), parentCell(0), gCost(0), hCost(0), fCost(0) {}
    };

    /**
     * @brief Compare function for priority queue
     * 
     */
    struct Compare
    {
        bool operator()(const CellData &a, const CellData &b)
        {
            return a.fCost > b.fCost;
        }
    };

    /**
     * @brief convert point to map cell index
     * 
     * @param pose 
     * @param map 
     * @return int cell index
     */
    int point2cell(const geometry_msgs::msg::Point &point, const nav_msgs::msg::OccupancyGrid &map);

    /**
     * @brief convert map cell index to point
     * 
     * @param index 
     * @param map 
     * @return geometry_msgs::msg::Point center point of cell
     */
    geometry_msgs::msg::Point cell2Point(const int &index, const nav_msgs::msg::OccupancyGrid &map);

    /**
     * @brief find the adjacent cell ID's in clockwise order starting from the positive Y axis.
     * 
     * @param index current cell
     * @param map 
     * @return std::array<int, 8> ID of adjacent cells. -1 represents no cell
     */
    std::array<int, 8> findAdjacentCells(const int &index, const nav_msgs::msg::OccupancyGrid &map);

    /**
     * @brief Finds distance between cells using modified manhattan distance. \n
     *        Only allowing 8 directional movement between cell centers for simplicity. \n
     *        horizontal and vertical movement = cost 1. \n 
     *        diagonal movement = cost squrt(2).
     * @param startCell 
     * @param goalCell 
     * @param map 
     * @return double distance between cells as number of cells.
     */
    double findEuklidDistance(const unsigned int &startCell, const unsigned int &goalCell, const nav_msgs::msg::OccupancyGrid &map);

    /**
     * @brief Reorders the priority queue
     * 
     * @param queue 
     */
    void reorderQueue(std::priority_queue<CellData, std::vector<CellData>, Compare> &queue);
};

#endif // PATHFINDING_H