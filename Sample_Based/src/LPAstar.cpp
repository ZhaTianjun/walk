#include "../include/LPAstar.h"
#include "math.h"
#include <chrono>
using namespace std;
LPAstarPlanner::LPAstarPlanner( ):GLX_SIZE(6),GLY_SIZE(7){
    data.clear();
    vector<int> temp(GLY_SIZE,0);
    data.resize(GLX_SIZE,temp);
    data[0][1]=1;
    data[0][2]=1;
    data[0][6]=1;
    data[1][4]=1;
    data[1][6]=1;
    data[2][1]=1;
    data[2][2]=1;
    data[2][3]=1;
    data[2][4]=1;
    data[2][6]=1;
    data[3][1]=1;
    data[3][2]=1;
    data[3][6]=1;
    data[4][1]=1;
    data[4][2]=1;
    data[4][4]=1;
    data[4][5]=1;
    data[4][6]=1;
    initialize();
}
LPAstarPlanner::LPAstarPlanner(vector<vector<int>>  data_ ){
    data = data_;
    GLY_SIZE=data_.size();
    GLX_SIZE=data_[0].size();
    initialize();
}
void LPAstarPlanner::initialize(){
    GridNodeMap =new GridNodePtr* [GLX_SIZE];
    for(int i=0;i<GLX_SIZE;i++){
        GridNodeMap[i] = new GridNodePtr[GLY_SIZE];
        for(int j=0;j<GLY_SIZE;j++){
            GridNodeMap[i][j]=new GridNode(i,j);
        }
    }
}
bool LPAstarPlanner::isOccupied(int x,int y){
    return x>=0 && x<=GLX_SIZE-1 && y>=0 && y<=GLY_SIZE-1 && data[x][y]==1;
}
bool LPAstarPlanner::isFree(int x,int y){
    return x>=0 && x<=GLX_SIZE-1 && y>=0 && y<=GLY_SIZE-1 && data[x][y]==0;
}
bool LPAstarPlanner::isEmpty()
{
    multimap<double,GridNodePtr>::reverse_iterator it=openSet.rbegin();
    while(it!=openSet.rend())
    {
        if(it->second->id==1)
            return 0;
        it++;
    }
    return 1;
}
double LPAstarPlanner::getHeu(GridNodePtr node1, GridNodePtr node2){
    double dx=fabs(node1->x - node2->x);
    double dy=fabs(node1->y - node2->y);
    return dx+dy+(sqrt(2)-2)*min(dx,dy);
}
void LPAstarPlanner::GetNeigborsets(GridNodePtr currentPtr,vector<GridNodePtr>& neighborPtrSets,
vector<double>& edgeCostSets){
    neighborPtrSets.clear();
    edgeCostSets.clear();
    for(int i=-1;i<=1;i++){
        for(int j=-1;j<=1;j++){
            if(i==0&&j==0) continue;
            int neighbor_x=i+currentPtr->x ;
            int neighbor_y=j+currentPtr->y ;
            if(neighbor_x<0||neighbor_x>=GLX_SIZE||
            neighbor_y<0||neighbor_y>=GLY_SIZE||
            isOccupied(neighbor_x,neighbor_y)||
            GridNodeMap[neighbor_x][neighbor_y]->id==-1)
                continue;
            GridNodePtr NeighborPtr=new GridNode(neighbor_x,neighbor_y);
            if(GridNodeMap[neighbor_x][neighbor_y]->id==1)
                NeighborPtr->id=1;
            double cost=sqrt(i*i+j*j);
            edgeCostSets.push_back(cost);
            neighborPtrSets.push_back(NeighborPtr);
        }
    }
}
void LPAstarPlanner::DisplayPath(){
    //vector<vector<char>>  data_dis;
    for(int j=0;j<GLY_SIZE;j++){
        for(int i=0;i<GLX_SIZE;i++){
            if(GridNodeMap[i][j]->id==-1)
                cout<<"* ";
            else
                cout<<data[i][j]<<" ";
        }
        cout<<endl;
    }
}
void LPAstarPlanner::AstarGraphSearch(pair<int,int> start_,pair<int,int> goal_){
    int start_x = start_.first;
    int start_y = start_.second;
    int goal_x=goal_.first;
    int goal_y=goal_.second;
    if(start_x<0||start_x>=GLX_SIZE||start_y<0||start_y>=GLY_SIZE||
    isOccupied(start_x,start_y)){
        cout<<"WRONG!!! Start point is unnormal!!!"<<endl;
        return;
    }
    if(goal_x<0||goal_x>=GLX_SIZE||goal_y<0||goal_y>=GLY_SIZE||
    isOccupied(goal_x,goal_y)){
        cout<<"WRONG!!! Goal point is unnormal!!!"<<endl;
        return;
    }
    openSet.clear();
    GridNodePtr StartPtr = new GridNode(start_x,start_y);
    GridNodePtr EndPtr = new GridNode(goal_x,goal_y);
    double val=getHeu(StartPtr,EndPtr);
    StartPtr->id=1;
    StartPtr->fScore=val;
    StartPtr->gScore=0;
    openSet.insert(make_pair(val,StartPtr));

    GridNodePtr currentPtr;
    vector<GridNodePtr> neighborPtrSets;
    vector<double> edgeCostSets;
    std::chrono::steady_clock::time_point _start_time=std::chrono::steady_clock::now();
    while(!isEmpty()){
        multimap<double,GridNodePtr>::iterator it=openSet.begin();
        while(it!=openSet.end()){
            if(it->second->id==1){
                 it->second->id = -1;
                currentPtr=it->second;
                GridNodeMap[currentPtr->x][currentPtr->y]->id=-1;
                break;
            }
            it++;
        }
        if(currentPtr->x==goal_x&&currentPtr->y==goal_y){
            std::chrono::steady_clock::time_point _end_time=std::chrono::steady_clock::now();
            std::chrono::duration<double> diff = _end_time-_start_time;
            cout<<"Success:A path is generated by Astar"<<endl;
            cout<<"Cost time is:"<<diff.count()*1000<<"ms"<<endl;
            cout<<"Path cost is:"<<currentPtr->gScore<<endl;
            DisplayPath( );
            return;
        }
        GetNeigborsets(currentPtr,neighborPtrSets,edgeCostSets);

        for(int i=0;i<neighborPtrSets.size();i++){
            GridNodePtr neighborPtr=neighborPtrSets[i];
            double cost=edgeCostSets[i];
            double value=currentPtr->gScore+cost+getHeu(neighborPtr,EndPtr);
            neighborPtr->gScore=currentPtr->gScore+cost;
            neighborPtr->fScore =value;
            if(neighborPtr->id==0){
                GridNodeMap[neighborPtr->x][neighborPtr->y]->id=1;
                neighborPtr->id=1;
                openSet.insert(make_pair(value,neighborPtr));
            }
            else if(neighborPtr->id==1){
                it=openSet.begin();
                while(it!=openSet.end()){
                    if(it->second->x==neighborPtr->x&&it->second->y==neighborPtr->y&&
                    it->second->gScore > neighborPtr->gScore){
                        openSet.erase(it);
                        openSet.insert(make_pair(value,neighborPtr));
                        break;
                    }
                    it++;
                }
            }
            else
            {
                    continue;
            }
            
        }

    }


}
