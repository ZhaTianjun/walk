#include "../include/ARAstar.h"
#include "math.h"
#include <chrono>
using namespace std;
ARAstarPlanner::ARAstarPlanner( ):GLX_SIZE(6),GLY_SIZE(7){
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
ARAstarPlanner::ARAstarPlanner(vector<vector<int>>  data_ ){
    data = data_;
    GLY_SIZE=data_.size();
    GLX_SIZE=data_[0].size();
    initialize();
}
void ARAstarPlanner::initialize(){
    GridNodeMap =new GridNodePtr* [GLX_SIZE];
    for(int i=0;i<GLX_SIZE;i++){
        GridNodeMap[i] = new GridNodePtr[GLY_SIZE];
        for(int j=0;j<GLY_SIZE;j++){
            GridNodeMap[i][j]=new GridNode(i,j);
        }
    }
}
bool ARAstarPlanner::isOccupied(int x,int y){
    return x>=0 && x<=GLX_SIZE-1 && y>=0 && y<=GLY_SIZE-1 && data[x][y]==1;
}
bool ARAstarPlanner::isFree(int x,int y){
    return x>=0 && x<=GLX_SIZE-1 && y>=0 && y<=GLY_SIZE-1 && data[x][y]==0;
}

double ARAstarPlanner::getHeu(GridNodePtr node1, GridNodePtr node2, double epsilon){
    double dx=fabs(node1->x - node2->x);
    double dy=fabs(node1->y - node2->y);
    //return epsilon*max(dx,dy);
    return epsilon*(dx+dy+(sqrt(2)-2)*min(dx,dy));
}
void ARAstarPlanner::GetNeigborsets(GridNodePtr currentPtr,vector<GridNodePtr>& neighborPtrSets,
vector<double>& edgeCostSets){
    neighborPtrSets.clear();
    edgeCostSets.clear();
    for(int i=1;i>=-1;i--){
        for(int j=1;j>=-1;j--){
            if(i==0&&j==0) continue;
            int neighbor_x=i+currentPtr->x ;
            int neighbor_y=j+currentPtr->y ;
            if(neighbor_x<0||neighbor_x>=GLX_SIZE||
            neighbor_y<0||neighbor_y>=GLY_SIZE||
            isOccupied(neighbor_x,neighbor_y))
                continue;
            GridNodePtr NeighborPtr=new GridNode(neighbor_x,neighbor_y);

            NeighborPtr->id=GridNodeMap[neighbor_x][neighbor_y]->id;
            double cost=sqrt(i*i+j*j);
            //double cost=1;//节点之间移动的代价值
            edgeCostSets.push_back(cost);
            neighborPtrSets.push_back(NeighborPtr);
        }
    }
}

void ARAstarPlanner::DisplayPath(){
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

void ARAstarPlanner::reInitialize(double& eps,double epsilon,GridNodePtr EndPtr){
     for(int j=0;j<GLY_SIZE;j++)
     {
        for(int i=0;i<GLX_SIZE;i++)
            GridNodeMap[i][j]->id=0;
    }

    //cout<<"111"<<endl;
    openSet2=openSet;
    openSet.clear();
    std::multimap<double, GridNodePtr>::iterator ite=openSet2.begin();
    //double max_eps=0;
    eps=0;
    while(ite!=openSet2.end()){
        //cout<<"111"<<endl;
        double value=ite->second->gScore+getHeu(ite->second,EndPtr,epsilon);
        eps=max(eps,EndPtr->fScore/(getHeu(ite->second,EndPtr,1)+ite->second->gScore));
        ite->second->fScore=value;
        if(ite->second->id==1)
        {
                GridNodeMap[ite->second->x][ite->second->y]->id = 1;
                cout<<ite->second->x<<","<<ite->second->y<<"   ";
                ite->second->id=1;
                openSet.insert(make_pair(value,ite->second));
        }
        ite++;
    }
    
    for(int i=0;i<inconsSet.size();i++){
        double value=inconsSet[i]->gScore+getHeu(inconsSet[i],EndPtr,epsilon);
        inconsSet[i]->fScore=value;
        openSet.insert(make_pair(value,inconsSet[i]));
        cout<<inconsSet[i]->x<<","<<inconsSet[i]->y<<"   ";
    }
    openSet2.clear();
    cout<<"is in openSet"<<endl;
    cout<<"eps is "<<eps<<endl;
    cout<<"openSet.size()="<<openSet.size()<<endl;
    //cout<<"open size "<<num<<endl;
    //cout<<"new close num="<<close_num<<endl;
    cout<<"_______________________"<<endl;
}
bool ARAstarPlanner::isEmpty()
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
void ARAstarPlanner::ARAstarGraphSearch(pair<int,int> start_,pair<int,int> goal_,double allowed_time){
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
    inconsSet.clear();
    double epsilon=2.5;//加权A*的初始权值
    GridNodePtr StartPtr = new GridNode(start_x,start_y);
    GridNodePtr EndPtr = new GridNode(goal_x,goal_y);
    double val=getHeu(StartPtr,EndPtr,epsilon);
    StartPtr->id=1;
    StartPtr->fScore=val;
    StartPtr->gScore=0;
    //EndPtr->fScore=inf;
    
    openSet.insert(make_pair(val,StartPtr));

    GridNodePtr currentPtr;
    vector<GridNodePtr> neighborPtrSets;
    vector<double> edgeCostSets;
    std::chrono::steady_clock::time_point _start_time=std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point _end_time=std::chrono::steady_clock::now();
    std::chrono::duration<double> diff = _end_time-_start_time;
    double epsilon_dot=epsilon;
    while(diff.count()*1000<allowed_time&&epsilon_dot>1&&epsilon>=1&&!isEmpty()){
        int close_num=0;
        while(!isEmpty()){
            //cout<<"111"<<endl;
            multimap<double,GridNodePtr>::iterator it=openSet.begin();
            while(it!=openSet.end()){
                if(it->second->id==1){
                        it->second->id = -1;
                        currentPtr=it->second;
                        //cout<<"Curent f and g  "<<it->second->fScore<<","<<it->second->gScore<<endl;
                        GridNodeMap[currentPtr->x][currentPtr->y]->id=-1;
                        break;
                }
                it++;
            }
            GetNeigborsets(currentPtr,neighborPtrSets,edgeCostSets);

            for(int i=0;i<neighborPtrSets.size();i++){
                GridNodePtr neighborPtr=neighborPtrSets[i];
                double cost=edgeCostSets[i];
                double value=currentPtr->gScore+cost+getHeu(neighborPtr,EndPtr,epsilon);
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
                    it=openSet.begin();
                    while(it!=openSet.end()){
                        if(it->second->x==neighborPtr->x&&it->second->y==neighborPtr->y&&
                        it->second->gScore > neighborPtr->gScore)
                        {
                            std::cout<<neighborPtr->x<<","<<neighborPtr->y<<"is in INCONS list"<<endl;
                            inconsSet.push_back(neighborPtr);
                            break;
                        }
                        it++;
                    }
                }
            }

            if(goal_x==currentPtr->x&&goal_y==currentPtr->y){
                EndPtr->fScore=currentPtr->fScore;
                cout<<"goal is expanded, cost is "<<EndPtr->fScore<<endl;
            }
            //close_num++;
            if(EndPtr->fScore  <=  currentPtr->fScore){
                _end_time=std::chrono::steady_clock::now();
                diff = _end_time-_start_time;
                cout<<"Success:A path is generated by Weighted Astar, epsilon="<<epsilon<<endl;
                cout<<"Cost time is:"<<diff.count()*1000<<"ms"<<endl;
                cout<<"Path cost is:"<<EndPtr->fScore<<endl;
                DisplayPath( );
                //return;
                epsilon-=0.5;
                reInitialize(epsilon_dot,epsilon,EndPtr);
                break;
            }
        }
    }
}
