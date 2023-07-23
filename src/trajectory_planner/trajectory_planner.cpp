#include "trajectory_planner.hpp"
#include <stdexcept>
#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.h>
#include "conversions.hpp"
#include <kal2_trajectory_planner/internal/utils.hpp>


namespace kal2_trajectory_planner_ros_tool{

/**
 * Initialization
 */
  TrajectoryPlanner_Node::TrajectoryPlanner_Node(const ros::NodeHandle& nhPrivate)
        : interface_{nhPrivate}, reconfigureServer_{nhPrivate} {

    interface_.fromParamServer();
    reconfigureServer_.setCallback(boost::bind(&TrajectoryPlanner_Node::reconfigureCallback, this, _1, _2));

    interface_.cones_subscriber->registerCallback(&TrajectoryPlanner_Node::conesCallback, this);

    pathGenerationTimer_ =
        nhPrivate.createTimer(ros::Rate(interface_.timer_rate), &TrajectoryPlanner_Node::pathCallback, this);


    interface_.showNodeInfo();
    interface_.logDebug("Node initialized.");


    }



void TrajectoryPlanner_Node::conesCallback(const nav_msgs::Path::ConstPtr& cones){
    std::vector<Point> conesposition = conversions::depthmsgtoconesposition(cones);

    if(conesposition.size() < 3){
        std::cout << "conesanzahl ist klein________ _________ _________ als 3 "<<std::endl;
    }

    Pose vehiclePose;
    if (!currentVehiclePose(vehiclePose)) {
        interface_.logWarn("Cannot compute trajectory because vehicle pose was not found in tf tree.");
        return;
    }

    std::cout << "vehiclePose: " << vehiclePose.translation().x() << vehiclePose.translation().y() << std::endl;
    // Point bezugpoint = {vehiclePose.translation().x(),vehiclePose.translation().y()};
    Point bezugpoint = {0,0};    // jetzt in vehicle
    std::sort(conesposition.begin(), conesposition.end(), [&bezugpoint](const Point& p1, const Point& p2) {
        return utils::compareByDistance(p1, p2, bezugpoint);
    });

    static std::vector<Point> erstreihe;
    static std::vector<Point> zweireihe;
    std::vector<Point> mapcones;
    Point erstmittelpoint;

    vehicletomap(conesposition, mapcones);
    utils::conessortieren(mapcones, erstreihe, zweireihe);

    a_star_.erstundzweireihe(erstreihe, zweireihe);
    conesInitialized_ = true;

    // std::cout << "mittelpoint.size: " << mittelpoints.size();
    // for (const Point& Position : mittelpoints) {
    //         std::cout << " (" << Position.x << ", " << Position.y << ")" ;
    //     }
    // std::cout<< std::endl;

// -----------------------------------------------------------------------------------
    // static std::vector<Point> mittelpoints;
    // utils::calculateMittelpoint(mittelpoints, erstreihe, zweireihe);

    // Eigen::VectorXd koeff = utils::fitPolynomial(mittelpoints, 2);

    // std::vector<Position> interpolar;
    // if (mittelpoints[0].x < mittelpoints[mittelpoints.size()-1].x){
    //     for (double i= mittelpoints[0].x; i < mittelpoints[mittelpoints.size()-1].x; i= i + 0.02 ){
    //         double poly_y = utils::polynomial_center(koeff,i);
    //         interpolar.push_back(Position(i, poly_y));
    //     }
    // }else{
    //     for (double i= mittelpoints[0].x; i > mittelpoints[mittelpoints.size()-1].x; i= i - 0.02 ){
    //         double poly_y = utils::polynomial_center(koeff,i);
    //         interpolar.push_back(Position(i, poly_y));
    //     }
    // }


    // std::cout << "interpolar.size: " << interpolar.size();
 //   utils::print_position(interpolar);






    // -------------------------------------------------------
    // std::vector<Point> linkcones;
    // std::vector<Point> rightcones;
    // Point currentPoint= {0,0};
    // utils::linkrightcones(conesposition, currentPoint,linkcones,rightcones);

    // std::vector<Point> maplinkcones;
    // std::vector<Point> maprightcones;
    // vehicletomap(rightcones, maprightcones);
    // vehicletomap(linkcones,maplinkcones);

    // a_star_.linkandrightcones(maplinkcones,maprightcones);


    // a_star_.cones(conesposition);       // conesposition_ = conesposition
    // conesInitialized_ = true;

}

void TrajectoryPlanner_Node::reconfigureCallback(const Interface::Config& config, uint32_t /*level*/) {
    interface_.fromConfig(config);
    // trajectoryPlanner_.setPolynomialDegree(interface_.polynomial_degree);
  }


void TrajectoryPlanner_Node::pathCallback(const ros::TimerEvent& /*timer*/) {
    if (!conesInitialized_) {
        interface_.logWarn("Cannot compute trajectory because no cones has been set yet.");
        return;
    }

    Pose vehiclePose;
    if (!currentVehiclePose(vehiclePose)) {
        interface_.logWarn("Cannot compute trajectory because vehicle pose was not found in tf tree.");
        return;
    }
    // Node startpoint(startpoint.x,startpoint.y );
    // Node targetpoint(targetpoint.x,targetpoint.y);
    // std::vector<Node*> Node_ = a_star_.aStarSearch(startpoint, targetpoint, linkcones, rightcones);


    // Pose vehiclePose;
    // if (!currentVehiclePose(vehiclePose)) {
    //     interface_.logWarn("Cannot compute trajectory because vehicle pose was not found in tf tree.");
    //     return;
    // }

// ----------------------------------------------------------------------------------------------------------------
    // Pose transform;       //    currentPosition
    // transform.translation() = Eigen::Vector2d(0.0, 0.0); // 设置平移向量 (1.0, 2.0)

    // const std::vector<Point>& points = a_star_.getConesPosition();
    // int num = points.size();
    // double distancetocones = conversions::findClosestDistance(points);

    // for (const Point& Position : points) {

    //     std::cout << "conesPoint (" << Position.x << ", " << Position.y << ")" << std::endl;
    // }


    // std::cout << "------------------" << std::endl;
// -----------------------------------------------------------------------------------------------------------
    // if (distancetocones < 0.1 && num >1 ){
    //     std::vector<Node*> Node_ = a_star_.aStar(transform);
    //     std::vector<Position> eigenVectors = conversions::convertToEigenVector(Node_);
    //     Trajectory trajectory;
    //     conversions::vectortotrajectory(eigenVectors, trajectory);
    //     interface_.path_publisher.publish(conversions::trajectoryToPathMsg(trajectory, interface_.map_frame));
    // }else{

    // }



    // if( a_star_.getConesPosition().empty()){
    //     std::cout << std::endl << "         cones anzahl ist null "<< std::endl;
    //     return;
    // }


    // std::vector<Node*> Node_ = a_star_.aStar(vehiclePose);
    // if (Node_.empty()){
    //     return;
    // }
    // conversions::print_vector(Node_);
    // std::vector<Position> eigenVectors = conversions::convertToEigenVector(Node_);
    // std::cout << std::endl << "eigenVectors anzahl " << eigenVectors.size() << std::endl;



    // std::vector<Position> eigenVectors = a_star_.fitPolynom(vehiclePose);
    std::cout << std::endl << "----------123 __________ _________ "  << std::endl;
    std::vector<Position> eigenVectors;



  //  a_star_.fitmittel(vehiclePose, eigenVectors);
     a_star_.fitcones(eigenVectors);


    std::cout << std::endl << "eigenVectors anzahl  __________ _________ " << eigenVectors.size() << std::endl;

    utils::print_position(eigenVectors);

    Trajectory trajectory;

     if(eigenVectors.empty()){
        std::cout << std::endl << "trajectory anzahl " << trajectory.size() << std::endl;
        return;
     }
    conversions::vectortotrajectory(eigenVectors, trajectory);
    std::cout << std::endl << "trajectory anzahl " << trajectory.size() << std::endl;

    if(trajectory.empty()){
        return;
    }   else{

        interface_.path_publisher.publish(conversions::trajectoryToPathMsg(trajectory, interface_.map_frame));

    }


}




bool TrajectoryPlanner_Node::currentVehiclePose(Pose& pose) const {

    // Find vehicle position in tf tree
    geometry_msgs::TransformStamped vehiclePoseRos;
    Eigen::Isometry3d vehiclePose3d;
    try {
        vehiclePoseRos = tfBuffer_.lookupTransform(interface_.map_frame, interface_.vehicle_frame, ros::Time(0));

        std::cout << "vehicle_frame zu map_frame : x = " << vehiclePoseRos.transform.translation.x << ", y = "
        << vehiclePoseRos.transform.translation.y << ", z = " << vehiclePoseRos.transform.translation.z << std::endl;


    } catch (const tf2::TransformException& e) {
        return false;
    }

    // Transform to Eigen type
    vehiclePose3d = tf2::transformToEigen(vehiclePoseRos);

    // Convert to two dimensions
    pose = conversions::isometry2dFromIsometry3d(vehiclePose3d);

    return true;
}


void TrajectoryPlanner_Node::vehicletomap( std::vector<Point>& cones, std::vector<Point>& mapposition){

    utils::removeAllZeroPoints(cones);
    for (const Point& Position : cones) {

        std::cout << std::endl << "conesPoint (" << Position.x << ", " << Position.y << ")" << std::endl;

        try
        {

            geometry_msgs::PointStamped ps;
            ps.header.frame_id = interface_.vehicle_frame;
            // ps.header.stamp = ros::Time::now();
            ps.point.x = Position.y;
            ps.point.y = -Position.x;
            ps.point.z = 0.0;

            geometry_msgs::PointStamped psAtSon2;
            psAtSon2 = tfBuffer_.transform(ps,interface_.map_frame);
            ROS_INFO("  in  map_frame :x=%.3f,y=%.3f,z=%.3f",
                    psAtSon2.point.x,
                    psAtSon2.point.y,
                    psAtSon2.point.z
                    );
            mapposition.push_back(Point{psAtSon2.point.x, psAtSon2.point.y});


        }
        catch(const std::exception& e)
        {
            // std::cerr << e.what() << '\n';
            ROS_INFO("异常信息:%s",e.what());

        }
    }

    std::cout << "------------------" << std::endl;
}



}