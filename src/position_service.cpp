/**
* \file position_service.cpp
* \brief This file is for computing a random value between a min and a max one
* \author Maria Luisa Aiachini
*
* \details
*
* Services: <BR>
* 	°/position_server
*
* Description:
*
* This node is used to compute a random position in between a minumum and a maximum one.
* When the service is required, it receives a request with a minimum value and a
* maximum one. It then generates a random position between the two.
*
*/

#include "ros/ros.h"
#include "rt2_assignment1/RandomPosition.h"

/**
* \brief
*
* \param
* \param
*
* \return
*
*
*/
double randMToN(double M, double N)
{     return M + (rand() / ( RAND_MAX / (N-M) ) ) ; }

/**
* \brief
*
* \param
* \param
*
* \return
*
*
*/
bool myrandom (rt2_assignment1::RandomPosition::Request &req, rt2_assignment1::RandomPosition::Response &res){
    res.x = randMToN(req.x_min, req.x_max);
    res.y = randMToN(req.y_min, req.y_max);
    res.theta = randMToN(-3.14, 3.14);
    return true;
}

/**
* \brief
*
* \param
* \param
*
* \return
*
*
*/
int main(int argc, char **argv)
{
   ros::init(argc, argv, "random_position_server");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/position_server", myrandom);
   ros::spin();

   return 0;
}
