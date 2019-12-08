#ifndef ASTAR
#define ASTAR

class AStar {

    bool isUnBlocked(const IntPair &point);

    static bool isDestination(const IntPair &point, const IntPair &dest);

    static float calculateH(const IntPair &point, const IntPair &dest);

    static std::vector <AStarNode> makePath(std::map <IntPair, AStarNode> &allMap, const IntPair &dest);

    void fillOccupanyGrid();

    public:
        std::vector <AStarNode> findAStarPath(const IntPair &dest);

    #ifndef PRODUCTION
    void showPersonLoc(const followbot::Point2 &personLoc);
    #endif
}


#endif // ASTAR