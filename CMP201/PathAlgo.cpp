#include"PathAlgo.h"
using namespace std;
#define ROW 100
#define COL 100

//Astar start
typedef pair<int, int> Couple;



typedef pair<double, pair<int, int> > PairoPair;

//defining a structure for storing parameters
struct ele
{
    //indices of parent element
    int par_i, par_j;

    //f(n)=g(n)+h(n)
    double f, g, h;
};

//Function checking legitmacy of element of maze
bool Validity(int col, int row)
{

    return (row >= 0) && (row < ROW) &&
        (col >= 0) && (col < COL);
}

//Function to check if element can be used as a path or not
bool UnBlocked(int maze[][COL], int row, int col)
{

    if (maze[row][col] == 1)
        return (true);
    else
        return (false);
}


//Function to check whther the input coordinates have been reached or not
bool Destination(int col, int row, Couple dest)
{
    if (row == dest.first && col == dest.second)
        return (true);
    else
        return (false);
}

//Function to calculate heuristics
double HValue(int row, int col, Couple dest)
{
    //returns distance by this formula. This formula is for the euclidean distance
    return ((double)sqrt((row - dest.first) * (row - dest.first)
        + (col - dest.second) * (col - dest.second)));
}

//Function for tracing the way from start to end point
void PathTracer(ele eleDetails[][COL], Couple dest)
{
    stack<Couple> Way;
    int row = dest.first;
    int col = dest.second;

    

    while (!(eleDetails[row][col].par_i == row
        && eleDetails[row][col].par_j == col))
    {
        Way.push(make_pair(row, col));
        int swap_row = eleDetails[row][col].par_i;
        int swap_col = eleDetails[row][col].par_j;
        row = swap_row;
        col = swap_col;
    }

    //this assists in printing out the path taken by a star algorithm
    Way.push(make_pair(row, col));
    cout << "Shortest Path is: " << Way.size() - 1 << "\n";
    while (!Way.empty())
    {
        pair<int, int> p = Way.top();
        Way.pop();
        cout << "-> {" << p.first << "," << p.second << "}";
    }

    return;
}

//This is the start of the function of a* algorithm
void aSearch(int maze[][COL], Couple startpt, Couple endpt)
{
   
    //This checks the legtimacy of inputted start point
    if (Validity(startpt.first, startpt.second) == false)
    {
        cout << "Source doesn't exist\n";
        return;
    }

    
     //This checks the legitamacy of inputted end point
    if (Validity(endpt.first, endpt.second) == false)
    {
        cout << "Destination doesn't exist\n";
        return;
    }

    
   
    //This checks whether the the start and end point of the maze are 0 or 1
    if (UnBlocked(maze, startpt.first, startpt.second) == false ||
        UnBlocked(maze, endpt.first, endpt.second) == false)
    {
        cout << "Source or the destination is blocked\n";
        return;
    }

    //This checks if the start point is the same as the end point
    if (Destination(startpt.first, startpt.second, endpt) == true)
    {
        cout << "We are already at the destination\n";
        return;
    }

    //Creating a closed list then initialising it to false so no element has been included yet
    bool ListClose[ROW][COL];
    memset(ListClose, false, sizeof(ListClose));

    //this 2d array of the structure stores the indices of the inputted element
    //It is filled using a nested for loop
    ele eleDetails[ROW][COL];

    int i, j;

    for (i = 0; i < ROW; i++)
    {
        for (j = 0; j < COL; j++)
        {
            eleDetails[i][j].f = FLT_MAX; // FLT_MAX is being used so that it can have the highest value of float
            eleDetails[i][j].g = FLT_MAX;
            eleDetails[i][j].h = FLT_MAX;
            eleDetails[i][j].par_i = -1;
            eleDetails[i][j].par_j = -1;
        }
    }

    //This gives us the details of the initial element of the maze
    i = startpt.first, j = startpt.second;
    eleDetails[i][j].f = 0.0;
    eleDetails[i][j].g = 0.0;
    eleDetails[i][j].h = 0.0;
    eleDetails[i][j].par_i = i;
    eleDetails[i][j].par_j = j;

    //An openlist is created where f=g+h
    //i,j represent row and column index of the element of maze
    set<PairoPair> ListOpen;

    //the starting cell is on the open list and it's f is set to 0
    ListOpen.insert(make_pair(0.0, make_pair(i, j)));

   

    //this bool value is initialized as false because the destination is not yet reached
    bool DestReach = false;



    while (!ListOpen.empty())
    {
        PairoPair p = *ListOpen.begin();

        //This vertex is removed from the open list
        ListOpen.erase(ListOpen.begin());

        //this vertex is then added to the closed list
        i = p.second.first;
        j = p.second.second;
        ListClose[i][j] = true;
        /*
         the following syntax helps in generating the 4 possible ways the algorithm can traverse
         these are :
         North (i-1, j)

         South (i+1, j)

         East (i, j+1)

         West (i, j-1)


         (The element which is popped has indices i,j)
       */
        //These variables help in storing the f(n),g(n) and h(n) of the 4 paths that can be travelled
        double gN, hN, fN;

        //North
         //The following statement checks whether the element entered is legitimate or not otherwise it won't run further
        if (Validity(i - 1, j) == true)
        {
            //This helps us to know if the next element(Successor) is the same as the end point(Destination)
            if (Destination(i - 1, j, endpt) == true)
            {

                eleDetails[i - 1][j].par_i = i;
                eleDetails[i - 1][j].par_j = j;
                cout << "The destination cell is found\n";
                PathTracer(eleDetails, endpt);
                DestReach = true;
                return;
            }

            //  Ignore successor if already on closed list or end point blocked
            // else do this
            else if (ListClose[i - 1][j] == false &&
                UnBlocked(maze, i - 1, j) == true)
            {
                gN = eleDetails[i][j].g + 1.0;
                hN = HValue(i - 1, j, endpt);
                fN = gN + hN;


                

                if (eleDetails[i - 1][j].f == FLT_MAX ||
                    eleDetails[i - 1][j].f > fN)
                {
                    ListOpen.insert(make_pair(fN,
                        make_pair(i - 1, j)));

                    //update details of the cell
                    eleDetails[i - 1][j].f = fN;
                    eleDetails[i - 1][j].g = gN;
                    eleDetails[i - 1][j].h = hN;
                    eleDetails[i - 1][j].par_i = i;
                    eleDetails[i - 1][j].par_j = j;
                }
            }
        }

        //South
         //The following statement checks whether the element entered is legitimate or not otherwise it won't run further
        if (Validity(i + 1, j) == true)
        {
            //This helps us to know if the next element(Successor) is the same as the end point(Destination)
            if (Destination(i + 1, j, endpt) == true)
            {

                eleDetails[i + 1][j].par_i = i;
                eleDetails[i + 1][j].par_j = j;
                cout << "The destination cell is found\n";
                PathTracer(eleDetails, endpt);
                DestReach = true;
                return;
            }

            //  Ignore successor if already on closed list or end point blocked
            // else do this  
            

            else if (ListClose[i + 1][j] == false &&
                UnBlocked(maze, i + 1, j) == true)
            {
                gN = eleDetails[i][j].g + 1.0;
                hN = HValue(i + 1, j, endpt);
                fN = gN + hN;


                

                if (eleDetails[i + 1][j].f == FLT_MAX ||
                    eleDetails[i + 1][j].f > fN)
                {
                    ListOpen.insert(make_pair(fN, make_pair(i + 1, j)));
                    // Update the details of this cell 
                    eleDetails[i + 1][j].f = fN;
                    eleDetails[i + 1][j].g = gN;
                    eleDetails[i + 1][j].h = hN;
                    eleDetails[i + 1][j].par_i = i;
                    eleDetails[i + 1][j].par_j = j;
                }
            }
        }

        //East
         //The following statement checks whether the element entered is legitimate or not otherwise it won't run further
        if (Validity(i, j + 1) == true)
        {
            //This helps us to know if the next element(Successor) is the same as the end point(Destination)
            if (Destination(i, j + 1, endpt) == true)
            {

                eleDetails[i][j + 1].par_i = i;
                eleDetails[i][j + 1].par_j = j;
                cout << "The destination cell is found\n";
                PathTracer(eleDetails, endpt);
                DestReach = true;
                return;
            }

            //  Ignore successor if already on closed list or end point blocked
            // else do this
            else if (ListClose[i][j + 1] == false &&
                UnBlocked(maze, i, j + 1) == true)
            {
                gN = eleDetails[i][j].g + 1.0;
                hN = HValue(i, j + 1, endpt);
                fN = gN + hN;

              

                if (eleDetails[i][j + 1].f == FLT_MAX ||
                    eleDetails[i][j + 1].f > fN)
                {
                    ListOpen.insert(make_pair(fN,
                        make_pair(i, j + 1)));

                    //update the details of the cell
                    eleDetails[i][j + 1].f = fN;
                    eleDetails[i][j + 1].g = gN;
                    eleDetails[i][j + 1].h = hN;
                    eleDetails[i][j + 1].par_i = i;
                    eleDetails[i][j + 1].par_j = j;
                }
            }
        }

        //West
         //The following statement checks whether the element entered is legitimate or not otherwise it won't run further
        if (Validity(i, j - 1) == true)
        {
            //This helps us to know if the next element(Successor) is the same as the end point(Destination)
            if (Destination(i, j - 1, endpt) == true)
            {

                eleDetails[i][j - 1].par_i = i;
                eleDetails[i][j - 1].par_j = j;
                cout << "The destination cell is found\n";
                PathTracer(eleDetails, endpt);
                DestReach = true;
                return;
            }
            //  Ignore successor if already on closed list or end point blocked
            // else do this

            else if (ListClose[i][j - 1] == false &&
                UnBlocked(maze, i, j - 1) == true)
            {
                gN = eleDetails[i][j].g + 1.0;
                hN = HValue(i, j - 1, endpt);
                fN = gN + hN;

                

                if (eleDetails[i][j - 1].f == FLT_MAX ||
                    eleDetails[i][j - 1].f > fN)
                {
                    ListOpen.insert(make_pair(fN,
                        make_pair(i, j - 1)));

                    //update details of the cell
                    eleDetails[i][j - 1].f = fN;
                    eleDetails[i][j - 1].g = gN;
                    eleDetails[i][j - 1].h = hN;
                    eleDetails[i][j - 1].par_i = i;
                    eleDetails[i][j - 1].par_j = j;
                }
            }
        }


        
    }

    /*
       The algorithm fails to reach the end goal when the end point of the maze is not found and the open list is empty
       This can happen when the algorithm encounters too many blockages and there is no feasible path to the end point/destination
    */

    if (DestReach == false)
        cout << "Failed to find the Destination Cell\n";

    return;
}

//Lee start
struct Init
{
    int x;
    int y;
};

struct queueNode
{
    Init pt;
    int dist;
};

//These arrays tell all posssible movements from the cell
int rowNum[] = { -1, 0, 0, 1 };
int colNum[] = { 0, -1, 1, 0 };

//Function to find shortest possible route from start to end point 
int BFS(int mat[][COL], Init src, Init dest)
{
    //condition to check wheter source and destination are equal or not
    if (!mat[src.x][src.y] || !mat[dest.x][dest.y])
        return -1;

    //This matrix helps keep record of reached cells
    bool reached[ROW][COL];

    //all elements are unreached initially
    memset(reached, false, sizeof reached);

    reached[src.x][src.y] = true;

    //creating an empty queue
    queue<queueNode> q;

    queueNode s = { src, 0 };
    q.push(s);

    while (!q.empty())
    {
        //element of the queue moves to the front
        queueNode curr = q.front();
        Init pt = curr.pt;
        cout << "->" << pt.x << "," << pt.y;
        if (pt.x == dest.x && pt.y == dest.y)
            return curr.dist;

        q.pop();

        //checking all 4 possible movements from initial cell
        for (int i = 0; i < 4; i++)
        {
            int row = pt.x + rowNum[i];
            int col = pt.y + colNum[i];

            if (Validity(row, col) && mat[row][col] &&
                !reached[row][col])
            {
                reached[row][col] = true;
                queueNode Adjcell = { {row, col},
                                      curr.dist + 1 };
                q.push(Adjcell);
            }
        }
    }

    return -1;
}



int main()
{
    ofstream Lee;
    ofstream Astar;
    //csv file to store 100 iterations of lee algorithm
    Lee.open("test_lee.csv");
    //csv file to store 100 iterations of Astar algorithm
    Astar.open("test_astar.csv");
    //variables for handling inputs
    int mat[ROW][ROW], n, i, j, choice, p = 1;
    int s_x = 0, s_y = 0, e_x = 0, e_y = 0;
    double time_taken;
    int dist;
    //clock for timing how much time it's taking for each code
    clock_t time_start, time_stop;

    //input the matrix to calculate the shortest path

    cout << "Enter side length of square maze(Less than 100):";
    cin >> n;
    cout << "\nEnter the maze:\n";
    for (i = 0; i < n; i++)
    {
        for (j = 0; j < n; j++)
            cin >> mat[i][j];
    }
    cout << "\nEnter Start Point: ";
    cin >> s_x >> s_y;
    cout << "\nEnter End Point: ";
    cin >> e_x >> e_y;

    Couple src = make_pair(s_x, s_y);
    Couple destn = make_pair(e_x, e_y);
    Init source = { s_x, s_y };
    Init dest = { e_x, e_y };


    //Menu for easy navigation through the application
    while (p)
    {
        cout << "\n           MENU                      \n";
        cout << "\n 1.Lee Algorithm    \n";
        cout << "\n 2.A* Algorithm   \n";
        cout << "\n 3.Gather time data over 1000 iterations   \n";
        cout << "\n 4.Exit            \n";
        cout << "\n Enter Your Choice: \n";
        cin >> choice;

        switch (choice)
        {
        case(1):
            //Lee
            time_start = clock();

            dist = BFS(mat, source, dest);
            time_stop = clock();

            time_taken = ((double)time_stop - time_start) / ((clock_t)1000);
            if (dist != -1)
            {
                cout << "\nNodes checked by Lee are " << dist;
                cout << "\nLee Algorithm Took " << time_taken << "s to navigate maze\n";
            }
            else
                cout << "\nPath doesn't exist";
            break;
        case(2):
            //A*
            time_start = clock();
            aSearch(mat, src, destn);
            time_stop = clock();
            time_taken = ((double)time_stop - time_start) / ((clock_t)1000);
            cout << "\nA* Algorithm Took " << time_taken << "s to navigate maze\n";
            break;
        case(3):
            for (int r = 0; r < 100; r++)
            {
                //Lee
                time_start = clock();

                dist = BFS(mat, source, dest);
                time_stop = clock();

                time_taken = ((double)time_stop - time_start) / ((clock_t)1000);
              //if (dist != -1)
              //{
              //    cout << "\nNodes checked by Lee are " << dist;
              //    cout << "\nLee Algorithm Took " << time_taken << "s to navigate maze\n";
              //}
              //else
              //    cout << "\nPath doesn't exist";
                Lee << time_taken << endl;
            }
            for (int r = 0; r < 100; r++)
            {
                //A*
                time_start = clock();
                aSearch(mat, src, destn);
                time_stop = clock();
                time_taken = ((double)time_stop - time_start) / ((clock_t)1000);
               // cout << "\nA* Algorithm Took " << time_taken << "s to navigate maze\n";
                Astar << time_taken << "," << endl;
            }
        case(4):
            p = 0;
            break;
        default:
            cout << "\nInvalid Choice\n";
        }

    }
    return 0;
}