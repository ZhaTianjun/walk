#include "GridNode.h"
#include <vector>
#include <map>
#include <algorithm>
#include<iostream>
using namespace std;

class ARAstarPlanner{
private:
    vector<vector<int>>  data;//1 obs ,0 free
    GridNodePtr ** GridNodeMap;
    vector<pair<int,int>> final_path;
    int GLX_SIZE, GLY_SIZE;
    std::multimap<double, GridNodePtr> openSet;
    std::multimap<double, GridNodePtr> openSet2;
    vector<GridNodePtr> inconsSet;
    double getHeu(GridNodePtr node1, GridNodePtr node2, double epsilon);
    bool isOccupied(int x,int y);
    bool isFree(int x,int y);
    
public:
    ARAstarPlanner();
    ARAstarPlanner(vector<vector<int>>  data_);
    void initialize();
    void reInitialize(double &eps,double epsilon,GridNodePtr EndPtr);
    void ARAstarGraphSearch(pair<int,int> start_,pair<int,int> goal_,double allowed_time);
    void GetNeigborsets(GridNodePtr currentPtr,vector<GridNodePtr>& neighborPtrSets,
vector<double>& edgeCostSets);
    void DisplayPath( );
    bool isEmpty();
};