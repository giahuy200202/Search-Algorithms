from inspect import stack
import numpy as np 
from math import sqrt

#------------Heuristic------------
def eclidean_distance(x1, x2):
    return sqrt((x2[0]-x1[0])**2 + (x2[1]-x1[1])**2)

#--------------DFS---------------
def DFS(matrix, start, end):

    path=[]; visit={}; back={}; stack=[]; temp=0; count=0; index=0
    stack.append(start)

    #Loop until stack is empty
    while(len(stack)!=0):
        temp=int(stack[-1])
        stack.pop()

        #Calculate previous node to choose
        while(bool(back)==True):          
            if(list(back.values())[-1]==0):
                if(list(back.keys())[-1]==start): break
                back.popitem()
            else:
                q=list(back.keys())[-1]; p=list(back.values())[-1]-1
                back[q]=p; break
        if(bool(back)==True): visit[temp]=list(back.keys())[-1]

        #Check goal, if is goal return visited, path
        if(end in visit):
            for x,y in reversed(visit.items()):
                if(index==0):
                    path.append(x); path.append(y)
                if(x==path[-1]): path.append(y)
                index+=1
            path.reverse()
            visited={start:start}; visited.update(visit)
            return visited, path
        
        #Add to stack
        count=0
        for i in range (len(matrix[temp])-1,-1,-1):
            check=True
            #Check if node is visited or not
            for j in visit.values():
                if(i==j): check=False
            if(check==True and int(matrix[temp][i])!=0): 
                stack.append(i); count+=1
        back[temp]=count



#--------------BFS---------------
def BFS(matrix, start, end):

    path=[]; visit={}; que=[]; temp=start; index=0;
    que.append(start)

    #Loop until queue is empty
    while(len(que)!=0):

        #Check goal
        if(end in que):
            for x,y in reversed(visit.items()):
                if(end==x):
                    path.append(x)
                    path.append(y)
                    
                if(len(path)!=0 and x==path[-1]): path.append(y)
                index+=1
            path.reverse()
            visited={start:start}; visited.update(visit)
            return visited, path   

        #Choose next node
        temp=int(que[0])

        #Delete node is chosen
        del que[0]

        #Check goal, if is goal return visited, path
        for i in range (0, len(matrix[temp])):
            check=True
            for j, z in visit.items():
                if(i==j or i==z): check=False
            if(check==True and int(matrix[temp][i])!=0):
                que.append(i)
                visit[i]=temp


#--------------UCS---------------
def UCS(matrix, start, end):

    path=[]; visitTemp={}; pathCost={}; visit={}; front={}; temp=start; minKey=10000000000; minValue=10000000000
    visitTemp[start]=0
    pathCost[start]=0

    #Loop until find goal
    while(True):   

        #Expand node and set cost
        for i in range (0, len(matrix[temp])):
            
            if(int(matrix[temp][i])!=0):
                if(i not in pathCost):
                    pathCost[i]=pathCost[temp]+int(matrix[temp][i])
                    visitTemp[i]=pathCost[i]
                    front[i]=temp
                elif( pathCost[i] > pathCost[temp]+int(matrix[temp][i]) ):
                    pathCost[i]=pathCost[temp]+int(matrix[temp][i])
                    visitTemp[i]=pathCost[i]
                    front[i]=temp

        #Choose min node by cost
        minKey=10000000000; minValue=10000000000
        for x,y in visitTemp.items():
            if(y!=0 and minValue>y):
                minKey=x; minValue=y
        if(len(visitTemp)==1): break
        visit[minKey]=front[minKey]
        del visitTemp[minKey]; del front[minKey]
        temp=minKey

        #Check goal, if is goal return visited, path
        if(end in visit.keys()):
            for x,y in reversed(visit.items()):
                if(end==x):
                    path.append(x)
                    path.append(y)
                elif(len(path)!=0 and x==path[-1]): path.append(y)
            path.reverse()
            visited={start:start}; visited.update(visit)
            return visited, path 


#--------------GBFS ( heuristic=path cost like UCS )---------------
def GBFS(matrix, start, end):

    path=[]; visitTemp={}; pathCost={}; visit={}; front={}; temp=start; minKey=10000000000; minValue=10000000000
    visitTemp[start]=0
    pathCost[start]=0

    #Loop until find goal
    while(True):   

        #Expand node and set cost
        for i in range (0, len(matrix[temp])):
            
            if(int(matrix[temp][i])!=0):
                if(i not in pathCost):
                    pathCost[i]=pathCost[temp]+int(matrix[temp][i])
                    visitTemp[i]=pathCost[i]
                    front[i]=temp
                elif( pathCost[i] > pathCost[temp]+int(matrix[temp][i]) ):
                    pathCost[i]=pathCost[temp]+int(matrix[temp][i])
                    visitTemp[i]=pathCost[i]
                    front[i]=temp

        #Choose min node by cost
        minKey=10000000000; minValue=10000000000
        for x,y in visitTemp.items():
            if(y!=0 and minValue>y):
                minKey=x; minValue=y
        if(len(visitTemp)==1): break
        visit[minKey]=front[minKey]
        del visitTemp[minKey]; del front[minKey]
        temp=minKey

        #Check goal, if is goal return visited, path
        if(end in visit.keys()):
            for x,y in reversed(visit.items()):
                if(end==x):
                    path.append(x)
                    path.append(y)
                elif(len(path)!=0 and x==path[-1]): path.append(y)
            path.reverse()
            visited={start:start}; visited.update(visit)
            return visited, path


#--------------A star---------------
def Astar(matrix, start, end, pos):

    path=[]; visitTemp={}; pathCost={}; visit={}; front={}; temp=start; minKey=10000000000; minValue=10000000000
    visitTemp[start]=0
    pathCost[start]=0

    #Loop until find goal
    while(True):

        #Expand node and set cost
        for i in range (0, len(matrix[temp])):
            
            if(int(matrix[temp][i])!=0):
                if(i not in pathCost):
                    pathCost[i]=pathCost[temp]+int(matrix[temp][i])
                    visitTemp[i]=pathCost[i]
                    front[i]=temp
                elif( pathCost[i]+eclidean_distance(pos[i],pos[end])> pathCost[temp]+int(matrix[temp][i])+eclidean_distance(pos[i],pos[end])):
                    pathCost[i]=pathCost[temp]+int(matrix[temp][i])
                    visitTemp[i]=pathCost[i]
                    front[i]=temp

        #Choose min node by path cost + heuristic function
        minKey=10000000000; minValue=10000000000
        for x,y in visitTemp.items():
            if(x!=start and minValue>y+eclidean_distance(pos[x],pos[end])):
                minKey=x; minValue=y
        if(len(visitTemp)==1): break
        visit[minKey]=front[minKey]
        del visitTemp[minKey]; del front[minKey]
        temp=minKey

        #Check goal, if is goal return visited, path
        if(end in visit.keys()):
            for x,y in reversed(visit.items()):
                if(end==x):
                    path.append(x)
                    path.append(y)
                elif(len(path)!=0 and x==path[-1]): path.append(y)
            path.reverse()
            visited={start:start}; visited.update(visit)
            return visited, path 
