#ifndef SAMPLE_BASED
#define SAMPLE_BASED
#include "GridNode.h"
#include <vector>
#include <iostream>
using namespace std;
class SAMPLE_BASED{
    public:
        int GLX_SIZE, GLY_SIZE;
        vector<vector<int>>  data;
        bool isOccupied(int x,int y){
            return x>=0 && x<=GLX_SIZE-1 && y>=0 && y<=GLY_SIZE-1 && data[x][y]==1;
        }
        bool isFree(int x,int y){
            return x>=0 && x<=GLX_SIZE-1 && y>=0 && y<=GLY_SIZE-1 && data[x][y]==0;
        }
        virtual void GraphSearch(pair<int,int> start_,pair<int,int> goal_)=0;

};

#endif