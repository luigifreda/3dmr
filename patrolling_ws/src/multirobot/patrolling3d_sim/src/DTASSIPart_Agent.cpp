#include "SSIPatrolAgent.h"




//Sequential Single Item Auction with dynamic compact partition of the environment

class DTASSIPart_Agent : public SSIPatrolAgent
{
protected:
    //center location given current tasks: task location that is at minimum path distance from all other locations. This is always a task location. 
    size_t current_center_location;

    //compute minimum path cost considering all tasks (tasks) and the next vertex (nv). 
    //The first room is always the current goal (if any), then rooms are visited in decreasing order of utility. 
    //The path cost is sum of travel cost given the order. 
    double compute_bid(int nv);

    //update tasks setting to true only the vertices for which this robot has the current highest bid
    //based on the array bids	
    void update_tasks();

    //compute center point given current tasks	
    void compute_center_location();

    double compute_sum_distance(int cv);



public:

    DTASSIPart_Agent()
    {
    }

    void init(int argc, char** argv);


};

void DTASSIPart_Agent::init(int argc, char** argv)
{

    //    logfile = fopen("DTASSIOut.log","w");	

    //    fprintf(logfile,"INITIALIZING \n");
    //    fflush(logfile);	

    SSIPatrolAgent::init(argc, argv);

    //    fprintf(logfile,"INITIALIZING 2 \n");
    //    fflush(logfile);	

    //set current center location to current vertex	    
    current_center_location = current_vertex_;

    //    fprintf(logfile,"initialised current center location to: %d \n",current_center_location);

    //initialize parameters

}

double DTASSIPart_Agent::compute_bid(int nv)
{

    /*printf("computing bid for vertex %d (using dynamic partition) \n ",nv);
    printf("current tasks = ");
    for (size_t i = 0; i<dimension;i++){
        printf(" %d, ",tasks[i]);	
    }
    printf("] \n");*/

    if (nv == next_vertex_ || nv == next_next_vertex)
    {
        // printf("already going to %d sending 0 (current target: %d)",nv,next_vertex);
        return 0.;
    }

    size_t num_tasks = 1;
    for (size_t i = 0; i < graph_dimension_; i++)
    {
        if (tasks[i])
        {
            num_tasks++;
        }
    }

    //	size_t cv = current_vertex;
    //	if (next_vertex >= 0 && next_vertex <dimension){
    //		cv = next_vertex;
    //	}

    double bid_value = compute_cost(nv, current_center_location) * num_tasks;
    //printf("bid for %d (current center %zu, num task %zu): %.2f \n",nv,current_center_location,num_tasks,bid_value);

    return bid_value;
}

void DTASSIPart_Agent::compute_center_location()
{
    size_t min = current_vertex_;
    //	printf("compute center:: min: %d current center: %d \n",min,current_center_location);
    double min_dist = compute_sum_distance(min);
    //	printf("compute center:: min dist: %.2f \n",min_dist);
    for (size_t i = 0; i < graph_dimension_; i++)
    {
        if (i != current_center_location && tasks[i])
        {
            double dist = compute_sum_distance(i);
            //			printf("compute center:: current dist: %.2f, min dist: %.2f, current min: %d, current point: %d \n",dist,min_dist,min,i);
            if (dist < min_dist)
            {
                min = i;
                min_dist = dist;
            }
        }
    }
    current_center_location = min;

}

double DTASSIPart_Agent::compute_sum_distance(int cv)
{
    if (cv < 0 || cv >= graph_dimension_)
    {
        //		printf("return big number: cv = %d",cv);
        return BIG_NUMBER;
    }
    double sum = 0.;
    for (size_t i = 0; i < graph_dimension_; i++)
    {
        if (tasks[i])
        {
            //			printf("sum: %2.f \n",sum);
            sum += compute_cost(cv, i);
        }
    }
    return sum;
}

void DTASSIPart_Agent::update_tasks()
{

    /*debug print
            printf("updating tasks: \n tasks before[");
            for (size_t i = 0; i<dimension; i++){
                printf(" %d, ",tasks[i]);	
            }
            printf("] \n"); 

            printf("bids [");
            for (size_t i = 0; i<dimension; i++){
                printf(" <%.2f,%d>, ",bids[i].bidValue,bids[i].robotId);	
            }
            printf("] \n"); 

            printf("center location before %d \n",current_center_location);

    ------------*/

    nactivetasks = 0;
    bool changed = false;
    for (size_t i = 0; i < graph_dimension_; i++)
    {
        if (!changed && tasks[i] != (bids[i].robotId == ID_ROBOT_))
        {
            changed = true;
        }
        tasks[i] = (bids[i].robotId == ID_ROBOT_);
        if (tasks[i]) nactivetasks++;
    }

    if (changed)
    {
        compute_center_location();
    }

#if DEBUG_PRINT

    printf("DTAP current center location: %lu\n", current_center_location);
    printf("DTAP: Active Tasks %d [", nactivetasks);
    for (size_t i = 0; i < graph_dimension_; i++)
    {
        if (tasks[i]) printf("%lu ", i);
    }
    printf("] \n");



#endif   
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "patrol_agent"); // will be replaced by __name:=XXXXXX
    
    DTASSIPart_Agent agent;
    agent.init(argc, argv);
    agent.run();

    return 0;
}

