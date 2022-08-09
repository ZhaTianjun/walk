#include "GridNode.h"
#include <vector>
#include <map>
#include <algorithm>
#include<iostream>
using namespace std;
class AstarPlanner{
private:
    vector<vector<int>>  data;//1 obs ,0 free
    GridNodePtr ** GridNodeMap;
    vector<pair<int,int>> final_path;
    int GLX_SIZE, GLY_SIZE;
    std::multimap<double, GridNodePtr> openSet;
    double getHeu(GridNodePtr node1, GridNodePtr node2);
    bool isOccupied(int x,int y);
    bool isFree(int x,int y);
public:
    AstarPlanner();
    AstarPlanner(vector<vector<int>>  data_);
    void initialize();
    void AstarGraphSearch(pair<int,int> start_,pair<int,int> goal_);
    void GetNeigborsets(GridNodePtr currentPtr,vector<GridNodePtr>& neighborPtrSets,
vector<double>& edgeCostSets);
    void DisplayPath( );
    bool isEmpty();
};