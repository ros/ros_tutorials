/*
 * Astar.cpp
 *
 *  Created on: Sep 25, 2016
 *      Author: ZX
 */
#include <cmath>
#include <stdlib.h>
#include <limits.h>
#include <vector>
#include <algorithm>
#include <iostream>
#include <queue>
#include <fstream>
#include <stdlib.h>
#include <stack>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/rate.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <turtlesim/Pose.h>
#include "ros/ros.h"
#define PI 3.1415926
using namespace std;
ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;
turtlesim::Pose turtlesim_pose;
void move(double speed, double dist, bool isForward);
void rotate(double angle,double angle_tol);
void turn(double angle,double angle_tol);
double degree2radian(double degree);
void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);
void moveGoal(turtlesim::Pose goal_pose, double tol);
double getDist(turtlesim::Pose goal_pose);

void turn(double angle,double angle_tol){
	geometry_msgs::Twist vel_msg;
	double initial_pose_theta=turtlesim_pose.theta;
	double moved_angle=0;
	ros::Rate loop_rate(10);
	do{
		vel_msg.angular.z=2*(angle-moved_angle);
		velocity_publisher.publish(vel_msg);
		ros::spinOnce();
		loop_rate.sleep();
		moved_angle=turtlesim_pose.theta-initial_pose_theta;
	}while(abs(angle-moved_angle)>angle_tol);
	vel_msg.angular.z=0;
	velocity_publisher.publish(vel_msg);
}

void move(double speed, double dist, bool isForward)
{
	geometry_msgs::Twist  vel_msg;
	if(isForward)
		vel_msg.linear.x=speed;
	else
		vel_msg.linear.x=-speed;
	vel_msg.linear.y=0;
	vel_msg.linear.z=0;
	vel_msg.angular.x=0;
	vel_msg.angular.y=0;
	vel_msg.angular.z=0;
	double t0=ros::Time::now().toSec();
	double current_dist=0;
	double t1;
	ros::Rate loop_rate(10);
	while(current_dist<dist)
	{
		velocity_publisher.publish(vel_msg);
		t1=ros::Time::now().toSec();
		current_dist=speed*(t1-t0);
		loop_rate.sleep();
		ros::spinOnce();
	}

	vel_msg.linear.x=0;
	velocity_publisher.publish(vel_msg);
}

void rotate(double angle,double angle_tol)
{
	geometry_msgs::Twist vel_msg;
	vel_msg.linear.x=0;
	vel_msg.linear.y=0;
	vel_msg.linear.z=0;
	vel_msg.angular.x=0;
	vel_msg.angular.y=0;
	ros::Rate loop_rate(10);
	do{
		vel_msg.angular.z=2*(angle-turtlesim_pose.theta);
		velocity_publisher.publish(vel_msg);
		ros::spinOnce();
		loop_rate.sleep();
	}while(abs(angle-turtlesim_pose.theta)>angle_tol);
	vel_msg.angular.z=0;
	velocity_publisher.publish(vel_msg);
}

double degree2radian(double degree)
{
	return degree*PI/180;
}

void poseCallback(const turtlesim::Pose::ConstPtr & pose_message)
{
	turtlesim_pose.x=pose_message->x;
	turtlesim_pose.y=pose_message->y;
	turtlesim_pose.theta=pose_message->theta;
}

void moveGoal(turtlesim::Pose goal_pose, double tol)
{
	geometry_msgs::Twist vel_msg;
	vel_msg.linear.y=0;
	vel_msg.linear.z=0;
	vel_msg.angular.x=0;
	vel_msg.angular.y=0;
	ros::Rate loop_rate(10);
	do{
		vel_msg.linear.x=1.5*getDist(goal_pose);
		vel_msg.angular.z=4*(atan2(goal_pose.y-turtlesim_pose.y,goal_pose.x-turtlesim_pose.x)-turtlesim_pose.theta);
		velocity_publisher.publish(vel_msg);
		ros::spinOnce();
		loop_rate.sleep();
	}while(getDist(goal_pose)>tol);
	//	vel_msg.linear.x=0;
	//	vel_msg.angular.z=0;
	//	velocity_publisher.publish(vel_msg);
}

double getDist(turtlesim::Pose goal_pose)
{
	return sqrt(pow(goal_pose.x-turtlesim_pose.x,2)+pow(goal_pose.y-turtlesim_pose.y,2));
}

/**
 * define a class reprenting the point in a grid
 * final_cost: total cost
 * g_cost: the cost to come from initial point
 * point_cost: the cost of every point
 * heuristic_cost: the estimated cost to reach goal from current point
 * from_x, from_y: record the parent of current point
 */
class Cell
{
public:
	int x;
	int y;
	double final_cost;
	double g_cost;
	double heuristic_cost;
	double point_cost;
	int from_x;
	int from_y;
	bool is_blocked;
	Cell(int x,int y);
	Cell();
	Cell(int pos_x,int pos_y,double f);
	int getX();
	int getY();
	double getF();
	double getH();
	void print();
	double calculate_heuristic(Cell c);
	bool operator==(const Cell&c){
		if(this->x==c.x && this->y==c.y)
			return true;
		return false;
	}
};
/*
 * compare class for the priority queue
 */
class Compare
{
public:
	bool operator() (Cell c1, Cell c2)
	{
		return (c1.final_cost>c2.final_cost);
	}
};
/*
 * class for astar search
 * step_cost: the unit cost to go to neighbor;
 * cost_map: a collection of points representing the grid
 * open: the points which haven't been searched
 * closed: the points which have been searched
 * initial_cell: initial point
 * goal_cell: goal point
 * height, width: size of the grid, defined by user
 */
class Astar {
public:
	double step_cost;
	std::vector< vector<Cell> > cost_map;
	std::priority_queue<Cell,std::vector<Cell>,Compare> open;
	std::vector<Cell> closed;
	Cell initial_cell;
	Cell goal_cell;
	int height;
	int width;
	Astar(double step_cost,Cell goal_cell,Cell initial_cell,int w,int h,std::string filename);
	void update_cost(Cell current_cell,int i,int j,int flag);
	void set_obstacle(std::string obstacle_file,double eps);
	void read_point_cost(std::string filename);
	void print_map(char c);
	bool open_contain_cell(Cell c);
	void get_h_cost();
	bool search();
	void print_solution();
	void print_path();
	std::vector<Cell> get_path();
};

int main(int argc,char**argv){
	ros::init(argc,argv,"avoid_obstacle_turtle");
	ros::NodeHandle nh;
	velocity_publisher=nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",10);
	pose_subscriber=nh.subscribe("/turtle1/pose",10,poseCallback);
	double step_cost=1.0;
	int width=10;
	int height=10;
	Cell initial_cell(1,1,0.0);
	Cell goal_cell(9,6,0.0);
	double tol=0.005;
	double eps=1;
	std::string cost_map,obstacle_file;
	cout<<"\n please enter the path of cost map file: \n"<<endl;
	cin>>cost_map;
	cout<<"\n please enter the path of obstacle file: \n"<<endl;
	cin>>obstacle_file;
	Astar astar(step_cost,goal_cell,initial_cell,width,height,cost_map);
	astar.get_h_cost();
	std::vector<Cell> obstacle;
	int n=0;
	astar.set_obstacle(obstacle_file,eps);
	cout<<"\n the initial map: B indicates that the the point is blocked."<<endl;
	astar.print_map('b');
	bool success=astar.search();
	if(success)
		cout<<"\n path found! move along it!"<<endl;
	else
	{
		cout<<"\n no path found!"<<endl;
		return 0;
	}
	std::vector<Cell> final_path=astar.get_path();
	cout<<"\n the optimal path is: "<<endl;
	for(int i=0;i<final_path.size();i++)
		final_path[i].print();
	turtlesim::Pose turtle_pose;
	turtle_pose.x=initial_cell.x;
	turtle_pose.y=initial_cell.y;
	moveGoal(turtle_pose,tol);
	for(int i=0;i<final_path.size();i++){
		turtle_pose.x=final_path[i].x*eps;
		turtle_pose.y=final_path[i].y*eps;
		moveGoal(turtle_pose,tol);
	}
	return 0;
}

/*
 * constructor
 * user have to define the step cost, goal point, initial point, the size of the grid
 * and the filepath for the point cost
 */
Astar::Astar(double step_cost,Cell goal_cell,Cell initial_cell,int w, int h,std::string filename){
	this->step_cost=step_cost;
	this->goal_cell=goal_cell;
	this->initial_cell=initial_cell;
	this->closed.push_back(initial_cell);
	this->open.push(initial_cell);
	this->width=w;
	this->height=h;
	for(int i=0;i<this->height;i++){
		std::vector<Cell> vc;
		for(int j=0;j<this->width;j++){
			Cell c(i,j);
			vc.push_back(c);
		}
		this->cost_map.push_back(vc);
	}
	this->cost_map[initial_cell.x][initial_cell.y].g_cost=0;
	this->cost_map[initial_cell.x][initial_cell.y].final_cost
	=this->cost_map[initial_cell.x][initial_cell.y].calculate_heuristic(this->goal_cell);
	read_point_cost(filename);
}
/*
 * determine whether a point is in the priority queue or not
 * traverse the priority queue and compare
 */
bool Astar::open_contain_cell(Cell c){
	std::priority_queue<Cell,std::vector<Cell>,Compare> q=this->open;
	while(!q.empty()){
		Cell top_cell=q.top();
		if(top_cell==c)
			return true;
		q.pop();
	}
	return false;
}
/*
 * read the point_cost of every point in the grid from the given filepath
 */
void Astar::read_point_cost(std::string filename){
	ifstream fin(filename.c_str());
	std::string str;
	if(fin.is_open()){
		for(int i=0;i<height;i++)
			for(int j=0;j<width;j++){
				fin>>str;
				if(fin.eof())
					return;
				this->cost_map[i][j].point_cost=atof(str.c_str());
			}
	}
	fin.close();
}
/*
 * set the obstacles in the grid given the points vector
 */
void  Astar::set_obstacle(std::string obstacle_file,double eps){
	std::ifstream fin(obstacle_file.c_str());
	while(!fin.eof()){
		std::string x1_coor ;
		std::string y1_coor;
		std::string x2_coor ;
		std::string y2_coor;
		fin>>x1_coor;
		fin>>y1_coor;
		fin>>x2_coor;
		fin>>y2_coor;
		int x1=atof(x1_coor.c_str())/eps;
		int y1=atof(y1_coor.c_str())/eps;
		int x2=atof(x2_coor.c_str())/eps;
		int y2=atof(y2_coor.c_str())/eps;
		int x_max=(x1>x2)?x1:x2;
		int y_max=(y1>y2)?y1:y2;
		int x_min=(x1>x2)?x2:x1;
		int y_min=(y1>y2)?y2:y1;
		for(int i=x_min;i<=x_max;i++)
			for(int j=y_min;j<=y_max;j++)
				this->cost_map[i][j].is_blocked=true;
	}
	fin.close();
}
/*
 * calculate the heuristic_cost of every point in the grid
 */
void Astar::get_h_cost(){
	for(int i=0;i<this->height;i++)
		for(int j=0;j<this->width;j++){
			this->cost_map[i][j].heuristic_cost=this->cost_map[i][j].calculate_heuristic(this->goal_cell);
		}
}
/*
 * print out the solution
 */
void Astar::print_solution(){
	std::cout<<"the final cost map is:"<<std::endl;
	for(int i=0;i<this->height;i++){
		for(int j=0;j<this->width;j++)
			cout<<this->cost_map[i][j].final_cost<<"\t";
		std::cout<<"\n";
	}
	std::cout<<"the path is:"<<std::endl;
	Cell current=this->cost_map[this->goal_cell.x][this->goal_cell.y];
	while(true){
		current.print();
		if(current.from_x==this->initial_cell.x && current.from_y==this->initial_cell.y)
			break;
		current=this->cost_map[current.from_x][current.from_y];
	}
	this->initial_cell.print();
}
/*
 * update the cost of a point which is under search
 * if the point is already in the closed vector, then return
 * if it's g_cost plus the step cost from its neighboor
 * is lower than its current g_cost then update.
 * if the point is not in open, then push it in open
 */
void Astar:: update_cost(Cell current_cell,int i,int j,int flag){
	Cell c=this->cost_map[i][j];
	if(std::find(this->closed.begin(), this->closed.end(), c) !=
			this->closed.end()|| c.is_blocked==true)
		return;
	double tentative_g_cost;
	if(flag==0)
		tentative_g_cost=current_cell.g_cost+this->step_cost+c.point_cost;
	else
		tentative_g_cost=current_cell.g_cost+sqrt(2)*this->step_cost+c.point_cost;
	bool inOpen = this->open_contain_cell(c);
	if(tentative_g_cost<c.g_cost){
		c.from_x = current_cell.x;
		c.from_y=current_cell.y;
		c.g_cost=tentative_g_cost;
		c.final_cost=c.g_cost+c.heuristic_cost+c.point_cost;
	}
	if(!inOpen)
		this->open.push(c);
	this->cost_map[i][j]=c;
	return;
}
/*
 * search for path
 *pop out the first point in the priority queue
 *search its neighboors and update
 *put the current point in the closed vector
 */
bool Astar::search(){
	if(this->initial_cell.is_blocked==true)
		return false;
	Cell current;
	while(true){
		if(open.empty())
			return false;
		current = open.top();
		open.pop();
		closed.push_back(current);
		if(current == this->goal_cell)
			return true;
		if(current.x-1>=0)
			update_cost(current,current.x-1,current.y,0);
		if(current.x-1>=0 && current.y-1>=0)
			update_cost(current,current.x-1,current.y-1,1);
		if(current.x+1<this->width && current.y+1<this->height)
			update_cost(current,current.x+1,current.y+1,1);
		if(current.x-1>=0 && current.y+1<this->height)
			update_cost(current,current.x-1,current.y+1,1);
		if(current.x+1<this->width && current.y-1>=0)
			update_cost(current,current.x+1,current.y-1,1);
		if(current.y-1>=0)
			update_cost(current,current.x,current.y-1,0);
		if(current.y+1<this->width)
			update_cost(current,current.x,current.y+1,0);
		if(current.x+1<this->height)
			update_cost(current,current.x+1,current.y,0);
	}
	return false;
}
/*
 * print cost_map for debug use
 */
void Astar::print_map(char c){
	if(c=='b')
		for(int i=0;i<this->height;i++){
			for(int j=0;j<this->width;j++){
				if(this->cost_map[i][j].is_blocked==true)
					cout<<"B"<<"\t";
				else
					cout<<this->cost_map[i][j].final_cost<<"\t";
			}
			cout<<endl;
		}
	else if(c=='g')
		for(int i=0;i<this->height;i++){
			for(int j=0;j<this->width;j++)
				cout<<this->cost_map[i][j].g_cost<<"\t";
			cout<<endl;
		}
	else
		for(int i=0;i<this->height;i++){
			for(int j=0;j<this->width;j++)
				cout<<this->cost_map[i][j].heuristic_cost<<"\t";
			cout<<endl;
		}
}
std::vector<Cell> Astar::get_path(){
	std::stack<Cell> path;
	Cell current=this->cost_map[this->goal_cell.x][this->goal_cell.y];
	while(true){
		path.push(current);
		if(current.from_x==this->initial_cell.x && current.from_y==this->initial_cell.y)
			break;
		current=this->cost_map[current.from_x][current.from_y];
	}
	path.push(this->initial_cell);
	std::vector<Cell> final_path;
	while(!path.empty()){
		final_path.push_back(path.top());
		path.pop();
	}
	return final_path;
}

void Astar::print_path(){
	std::cout<<"the path is:"<<std::endl;
	Cell current=this->cost_map[this->goal_cell.x][this->goal_cell.y];
	while(true){
		current.print();
		if(current.from_x==this->initial_cell.x && current.from_y==this->initial_cell.y)
			break;
		current=this->cost_map[current.from_x][current.from_y];
	}
	this->initial_cell.print();
}

Cell::Cell(){
	this->x=0;
	this->y=0;
	this->heuristic_cost=0.0;
	this->final_cost=0.0;
	this->g_cost=0.0;
	this->from_x=0;
	this->from_y=0;
	this->point_cost=0.0;
	this->is_blocked=false;
}
/*
 * constructor
 * the final_cost and g_cost of points in the grid have to be initialized to be infinity
 */
Cell::Cell(int x,int y){
	this->x=x;
	this->y=y;
	this->heuristic_cost=0.0;
	this->final_cost=(double) INT_MAX;
	this->g_cost=(double) INT_MAX;
	this->from_x=0;
	this->from_y=0;
	this->point_cost=0.0;
	this->is_blocked=false;
}

Cell::Cell(int pos_x,int pos_y,double f){
	this->x=pos_x;
	this->y=pos_y;
	this->final_cost=f;
	this->heuristic_cost=0.0;
	this->g_cost=0.0;
	this->from_x=0;
	this->from_y=0;
	this->point_cost=0.0;
	this->is_blocked=false;
}
/*
 * print out the coordinate of a point
 */
void Cell::print(){
	std::cout<<"["<<this->x<<","<<this->y<<"]-->";
}
/*
 * calculate the heuristic_cost of a point
 * use can define other heuristic function
 */
double Cell::calculate_heuristic(Cell goal_cell){
	return abs(this->x-goal_cell.x)+abs(this->y-goal_cell.y);
}




