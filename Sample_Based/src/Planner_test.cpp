#include<iostream>
#include "../include/Astar.h"
#include "../include/ARAstar.h"
using namespace std;
int main(){
    //随机生成地图
    int x=21,y=21;
    vector<int> temp(y,0);
    vector<vector<int>> data(x,temp);
    int obs_num=1;
    while(obs_num<=200){
        int dx=rand()%x;
        int dy=rand()%y;
        data[dx][dy]=1;
        obs_num++;
        //cout<<dx<<","<<dy<<endl;
    }
    data[0][0]=0;
    data[x-1][y-1]=0;


    ARAstarPlanner ARA(data);//设置ARA*算法的输入地图，如果没有输入则采用默认地图
    ARA.ARAstarGraphSearch(make_pair(0,0),make_pair(x-1,y-1),5);//设置起点、终点和允许的最大时间/ms
    cout<<"ARA* Planning Finished!!!"<<endl;
    AstarPlanner Astar(data);
    Astar.AstarGraphSearch(make_pair(0,0),make_pair(x-1,y-1));
    cout<<"A* Planning Finished"<<endl;
    return 0;
}