#ifndef GRIDNODE
#define GRIDNODE
#define inf 1000000000
struct GridNode;
typedef GridNode* GridNodePtr;
struct GridNode
{
    int id;//1 ,open_list;    -1,closed_list;    0,未扩展; 
    int x;
    int y;
    double fScore,gScore;
    double rhsScore;
    GridNode(int _x,int _y){
        id=0;
        x=_x;
        y=_y;
        fScore=inf;
        gScore=inf;
    }
} ;
#endif  
