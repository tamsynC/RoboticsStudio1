#include "PathFinding.h"

PathFinding::PathFinding(const nav_msgs::msg::OccupancyGrid &mapOccupancy, const nav_msgs::msg::OccupancyGrid &mapHazard) 
    : Node("PathFinding"), CELL_OCCUPIED_THRESHOLD(75), CELL_VALUE_MIN(-1), CELL_VALUE_MAX(100), mapOccupancy_(mapOccupancy), mapHazard_(mapHazard)
{
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/path_visualization", 10);
}

void PathFinding::publishMarkers(const geometry_msgs::msg::PoseArray &path)
{   
    if (path.poses.size() == 0) {
        return;
    }
    
    visualization_msgs::msg::MarkerArray markerArray;

    for (unsigned int i = 0; i < path.poses.size(); i++)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "path";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = path.poses.at(i);
        marker.lifetime = rclcpp::Duration::from_seconds(1);    // this may need to be changed

        // start marker
        if (i == 0)
        {
            // scale
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;

            // colour
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;

        // end marker 
        }

        else if (i == path.poses.size() - 1) 
        {
            // scale
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;

            // colour
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;

        // any other marker
        }

        else
        {
            // scale
            marker.scale.x = 0.05;
            marker.scale.y = 0.05;
            marker.scale.z = 0.05;

            // colour
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
        }
        
        markerArray.markers.push_back(marker);
    }

    marker_pub_->publish(markerArray);
}

void PathFinding::updateOccupancyMap(const nav_msgs::msg::OccupancyGrid &msg)
{
    std::unique_lock<std::mutex> lck(mtx_);
    mapOccupancy_ = msg;
}

nav_msgs::msg::OccupancyGrid PathFinding::getOccupancyMap(void)
{
    std::unique_lock<std::mutex> lck(mtx_);
    return mapOccupancy_;
}

void PathFinding::updateHazardMap(const nav_msgs::msg::OccupancyGrid &msg)
{
    std::unique_lock<std::mutex> lck(mtx_);
    mapHazard_ = msg;
}

nav_msgs::msg::OccupancyGrid PathFinding::getHazardMap(void)
{
    std::unique_lock<std::mutex> lck(mtx_);
    return mapHazard_;
}

int PathFinding::point2cell(const geometry_msgs::msg::Point &point, const nav_msgs::msg::OccupancyGrid &map)
{   
    // NEEDS ROTATION COMPONENT!!!

    // convert to map frame
    geometry_msgs::msg::Point mapFrame;
    mapFrame.x = point.x - map.info.origin.position.x;
    mapFrame.y = point.y - map.info.origin.position.y;
    mapFrame.z = 0; // axis not used
    
    // find grid reference
    int column = mapFrame.x / map.info.resolution;
    int row = mapFrame.y / map.info.resolution;

    // Bounds checking
    if (column < 0 || column >= map.info.width || row < 0 || row >= map.info.height)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Point (" << point.x << ", " << point.y << ") is out of bounds");

        if (column < 0 || column >= map.info.width) 
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Column is:" << column << " [0-" << map.info.width << "]");
        }
        
        if (row < 0 || row >= map.info.height) 
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Row is:" << row << " [0-" << map.info.height << "]");
        }
        
        return -1;
    }

    // find index
    int index = (row * map.info.width) + column;
   
    return index;
}

geometry_msgs::msg::Point PathFinding::cell2Point(const int &index, const nav_msgs::msg::OccupancyGrid &map)
{
    if (index < 0 || index >= map.info.width * map.info.height) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Index " << index << " is out of bounds. [0-" << map.info.width * map.info.height << "]");
        return geometry_msgs::msg::Point();
    }

    // find grid reference
    int column = index % map.info.width;
    int row = index / map.info.width;

    // convert to cartesian cordinates in the map frame
    geometry_msgs::msg::Point point;
    point.x = column * map.info.resolution;
    point.y = row * map.info.resolution;
    point.z = 0;    // not used

    // conveert to world frame
    point.x += map.info.origin.position.x;
    point.y += map.info.origin.position.y;

    return point;
}

std::array<int, 8> PathFinding::findAdjacentCells(const int &index, const nav_msgs::msg::OccupancyGrid &map)
{   
    std::array<int, 8> adjacentCells = {-1, -1, -1, -1, -1, -1, -1, -1};

    if (index < 0 || index >= map.info.width * map.info.height) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Index " << index << " is out of bounds. [0-" << map.info.width * map.info.height << "]");
        return adjacentCells;
    }

    // limit map size to INT_MAX cells so I dont have to keep changing variables
    if (map.info.width * map.info.height > INT_MAX)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Map too large to process, size [" << map.info.width * map.info.height << " / " << INT_MAX << "]");
        return adjacentCells;
    }

    // find grid reference
    int column = index % map.info.width;
    int row = index / map.info.width;

    bool isAtTop = false;
    bool isAtRight = false;
    bool isAtBottom = false;
    bool isAtLeft = false;

    if (column == 0) {
        isAtLeft = true;
    }

    if (column == map.info.width - 1) {
        isAtRight = true;
    }

    if (row == 0) {
        isAtBottom = true;
    }

    if (row == map.info.height - 1) {
        isAtTop = true;
    }

    if (!isAtTop) {
        adjacentCells[0] = index + map.info.width;  // N
    }
    
    if (!isAtTop && !isAtRight) {
        adjacentCells[1] = index + map.info.width + 1;  // NE
    }

    if (!isAtRight) {
        adjacentCells[2] = index + 1;  // E
    }

    if (!isAtBottom && !isAtRight) {
        adjacentCells[3] = index - map.info.width + 1;  // SE
    }

    if (!isAtBottom) {
        adjacentCells[4] = index - map.info.width;  // S
    }

    if (!isAtBottom && !isAtLeft) {
        adjacentCells[5] = index - map.info.width - 1;  // SW
    }

    if (!isAtLeft) {
        adjacentCells[6] = index - 1;  // W
    }

    if (!isAtTop && !isAtLeft) {
        adjacentCells[7] = index + map.info.width - 1;  // NW
    }

    // set all out of bounds cells to -1
    for (int i = 0; i < 8; i++)
    {
        if (adjacentCells[i] < -1 || adjacentCells[i] >= static_cast<int>(map.info.width * map.info.height))
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "adjacent cell in position " << i << " is out of bounds, setting to -1");
            adjacentCells[i] = -1;
        }
    }

    return adjacentCells;
}

double PathFinding::findEuklidDistance(const unsigned int &startCell, const unsigned int &goalCell, const nav_msgs::msg::OccupancyGrid &map)
{   
    const float diagonalCost = sqrt(2);
    const char linearCost = 1;

    // X and Y distances
    int startColumn = startCell % map.info.width;
    int goalColumn = goalCell % map.info.width;
    int xDist = std::abs(startColumn - goalColumn);

    int startRow = startCell / map.info.width;
    int goalRow = goalCell / map.info.width;
    int yDist = std::abs(startRow - goalRow);

    // find diagonals
    unsigned int diagonalMoves = std::min(xDist, yDist);
    double cost = diagonalMoves * diagonalCost;

    // find linear moves
    cost += linearCost * (std::max(xDist, yDist) - diagonalMoves);

    return cost;
}

void PathFinding::reorderQueue(std::priority_queue<CellData, std::vector<CellData>, Compare> &queue) {
    std::priority_queue<CellData, std::vector<CellData>, Compare> tempList;
    tempList.swap(queue);

    while (!tempList.empty())
    {
        queue.push(tempList.top());
        tempList.pop();
    }
}

geometry_msgs::msg::PoseArray PathFinding::AStar(const geometry_msgs::msg::Pose &startPose, const geometry_msgs::msg::Pose &goalPose)
{
    nav_msgs::msg::OccupancyGrid map = getOccupancyMap();
    nav_msgs::msg::OccupancyGrid hazard = getHazardMap();
    geometry_msgs::msg::PoseArray path;

    // get start and end cells
    unsigned int startCell = point2cell(startPose.position, map);
    unsigned int goalCell = point2cell(goalPose.position, map);

    // check if start and goal poses are valid
    if (startCell == -1)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Start cell is invalid");
        return path;
    }

    if (goalCell == -1)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "End cell is invalid");
        return path;
    }

    if (static_cast<int>(map.data[startCell]) > CELL_OCCUPIED_THRESHOLD)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Start cell is occupied");
        return path;
    }

    if (static_cast<int>(map.data[goalCell]) > CELL_OCCUPIED_THRESHOLD)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "goal cell is occupied");
        return path;
    }

    // check if start and goal cells are the same
    if (startCell == goalCell)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Start and goal poses are in the same cell");
        path.poses.push_back(goalPose);
        return path;
    }

    // generate new cell data
    std::vector<CellData> cellDataVector;
    for (unsigned int i = 0; i < map.data.size(); i++)
    {
        cellDataVector.emplace_back
                            (
                            i,                                      // index of cell
                            false,                                  // has this cell been visited (g, h and f cost calculated)
                            static_cast<int>(map.data.at(i)),       // occupancy value
                            0,                                      // hazard value (THIS WILL NEED TO BE LOOKED AT ONCE JAMES HAS FINISHED!!!)
                            UINT_MAX,                               // index of parent cell
                            INFINITY,                               // (cost to get there from start)
                            INFINITY,                               // (cost to get to goal from here)
                            INFINITY                                // (G cost + H cost + hazard value)
                            );
    }

    // set start cell data
    cellDataVector.at(startCell).parentCell = startCell;
    cellDataVector.at(startCell).visited = true;
    cellDataVector.at(startCell).gCost = 0;
    cellDataVector.at(startCell).hCost = findEuklidDistance(startCell, goalCell, map);
    cellDataVector.at(startCell).fCost = cellDataVector.at(startCell).gCost + cellDataVector.at(startCell).hCost;

    // list
    std::priority_queue<CellData, std::vector<CellData>, Compare> openList;
    openList.push(cellDataVector.at(startCell));
    std::vector<int> closedList(cellDataVector.size());

    bool goalFound = false;

    while (!openList.empty())
    {
        // new cycle
        CellData currentCell = openList.top();
        openList.pop();
        closedList.push_back(currentCell.index);
        bool updatedList = false;

        // look at surrounding cells
        std::array<int, 8> adjacentCells = findAdjacentCells(currentCell.index, map);
        for (int i = 0; i < 8; i++)
        {
            // validity check
            if (adjacentCells[i] == -1)
            {
                continue;
            }

            // check if goal
            if (static_cast<unsigned int>(adjacentCells[i]) == goalCell)
            {
                goalFound = true;
            }

            // check if cell is in closed list
            if (std::find(closedList.begin(), closedList.end(), adjacentCells[i]) != closedList.end())
            {
                continue;
            }

            // check if cell occupied
            if (cellDataVector.at(adjacentCells[i]).occupancy >= CELL_OCCUPIED_THRESHOLD)
            {
                closedList.push_back(adjacentCells[i]);
                continue;
            }

            // set costs
            if (cellDataVector.at(adjacentCells[i]).hCost == INFINITY)
            {   
                cellDataVector.at(adjacentCells[i]).hCost = findEuklidDistance(adjacentCells[i], goalCell, map);
            }

            double newGCost = findEuklidDistance(currentCell.index, adjacentCells[i], map);
            newGCost += currentCell.gCost;
            double newFCost = cellDataVector.at(adjacentCells[i]).hCost + newGCost;
            
            // make current cell new parent if new cost is lower
            if (newFCost < cellDataVector.at(adjacentCells[i]).fCost)
            {
                // update cell data
                cellDataVector.at(adjacentCells[i]).parentCell = currentCell.index;
                cellDataVector.at(adjacentCells[i]).gCost = newGCost;
                cellDataVector.at(adjacentCells[i]).fCost = newFCost;

                // flag for list update
                if (cellDataVector.at(adjacentCells[i]).visited)
                {
                    updatedList = true;
                }
            }

            // add to open list if not already visited
            if (!cellDataVector.at(adjacentCells[i]).visited)
            {
                cellDataVector.at(adjacentCells[i]).visited = true;
                openList.push(cellDataVector.at(adjacentCells[i]));
            }
        }

        // Recalculate list if data updated
        if (updatedList)
        {
            reorderQueue(openList);
        }
    
        // if goal found, clear open list to end loop
        if (goalFound)
        {
            // Clear the queue
            std::priority_queue<CellData, std::vector<CellData>, Compare>().swap(openList);
        }
    }

    // check if goal found
    if (!goalFound)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Cannot find path");
        return path;
    }

    // generate path (this is generating backwards)
    CellData currentCell = cellDataVector.at(goalCell);
    geometry_msgs::msg::Pose pose;
    pose.position = cell2Point(goalCell, map);

    while (currentCell.parentCell != startCell)
    {
        // yaw is angle from parent
        geometry_msgs::msg::Point parentPoint = cell2Point(currentCell.parentCell, map);
        double yaw = atan2(pose.position.y - parentPoint.y, pose.position.x - parentPoint.x);
        tf2::Quaternion quat;
        quat.setRPY(0, 0, yaw);
        pose.orientation = tf2::toMsg(quat);

        // add pose to path
        path.poses.push_back(pose);
        pose.position = parentPoint;
        currentCell = cellDataVector.at(currentCell.parentCell);
    }

    // filter out non-critical poses (probs should do this earlier)

    // reverse path
    std::reverse(path.poses.begin(), path.poses.end());

    // path smoothing?

    return path;
}