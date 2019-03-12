/* @info: Links:
 * rviz plugin create: http://docs.ros.org/kinetic/api/rviz_plugin_tutorials/html/display_plugin_tutorial.html
 *
 * @info: Plotting in rviz:
 * gps data (as red points)
 * on top of a map: https://github.com/gareth-cross/rviz_satellite
 *
 * edited 2017.Mai: Markus Lamprecht
 *
 *@TODO:
 * MarkerList for gps points does not work... (if uncheck check marker sollen alle marker wieder erscheinen!)
 * erstelle eigenes rviz plugin wie das AerialMapDisplay plugin !
 *  in diesem Plugin mach es möglich über extra read only schaltflächen daten zu markern anzuzeigen!
 *  readonly field: lat
 *  readonly field: long, height,velocity, etc.
 * */

#include <ros/ros.h>
#include <cmath>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

/* Messages: */
#include <rviz_maps/rviz_scale.h>      // listen to point scaling

// requires Marker for rviz visualization of solution points.
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

// Required to convert lat long to UTM coordinates: UTM = planar
#include <geodesy/wgs84.h>  // sudo apt-get install ros-kinetic-geodesy
#include <geodesy/utm.h>    //sudo apt-get install ros-kinetic-geodesy

// incomming data that does not need to be stored:
// values are updated when first message comes in!
double current_scale1=1;
double current_center_lat =49.00;
double current_center_long=9.8;


// incomming data that is stored:
double lat1_array[15000];
double long1_array[15000];
double alt1_array[15000];
double status1_array[15000];

//storing current distance to origin:
double x1_array[15000];
double y1_array[15000];
double z1_array[15000];

// count number of points:
int  point1_nu =0;

//required to plot the lineal
int id_counter_arrow;

// Method to convert a lat long point to a utm point
geodesy::UTMPoint latlong2utm(double lat, double lon){
    geographic_msgs::GeoPoint geo_point;
    geo_point.latitude = lat;
    geo_point.longitude = lon;
    geo_point.altitude = 0;

    geodesy::UTMPoint utm_point;
    geodesy::fromMsg(geo_point, utm_point);
    return utm_point;
}

//undefined reference to `geodesy::fromMsg(
//geographic_msgs::GeoPoint_<std::allocator<void> > const&, geodesy::UTMPoint&)'
//fromMsg (const geographic_msgs::GeoPoint &from, UTMPoint &to)

// Method to convert:
// 1. start point to utm point
// 2. reference point to utm point
// 3. calculate the difference between start and end point.
// 4. Return difference point in utm coordinates (this points will then be plotted!)
geometry_msgs::Point latlong2coordinates(double ref_point_lat,double ref_point_long,double start_point_lat,double start_point_long){
    geodesy::UTMPoint ref_point_utm = latlong2utm(ref_point_lat,ref_point_long);
    geodesy::UTMPoint start_point_utm = latlong2utm(start_point_lat,start_point_long);
    geometry_msgs::Point difference;
    difference.x=  ( (ref_point_utm.easting)-(start_point_utm.easting));
    difference.y = ( (ref_point_utm.northing)-(start_point_utm.northing) );
    difference.z = 0;
    return difference;
}


//double getColor(std::string color){
//    if(solution_status_array[num_pos_deg-1] >3.0) // Fix -> green
//    {
//        if(color=="red" || color == "blue"){
//            return 0.0f;
//        }
//        else{
//            return 1.0;
//        }
//    }
//    if(solution_status_array[num_pos_deg-1] == 0) // Single -> red
//    {
//        if(color=="green" || color == "blue"){
//            return 0.0f;
//        }
//        else{
//            return 1.0;
//        }
//    }
//    if(solution_status_array[num_pos_deg-1] < 3.0) // float -> yellow
//    {
//        if(color == "blue"){
//            return 0.0f;
//        }
//        else{
//            return 1.0;
//        }
//    }
//    return 0; // error;
//}

void plot_points1(ros::Publisher publisher_name){
  // function is executed with 10 Hz.
  //ROS_INFO("Plot points in plot_points1");
    // TODO: just draws last point at the moment.
    // coordinate points:
    visualization_msgs::Marker marker_points1;
    // visualization_msgs::MarkerArray utm_bias_array;
    marker_points1.header.frame_id ="/map";
    marker_points1.header.stamp = ros::Time::now();
    marker_points1.ns ="points1";
    marker_points1.action =visualization_msgs::Marker::ADD;
    marker_points1.pose.orientation.x = 0.0;
    marker_points1.pose.orientation.y = 0.0;
    marker_points1.pose.orientation.z = 0.0;
    marker_points1.pose.orientation.w = 1.0;
    marker_points1.pose.position.x =x1_array[point1_nu-1];
    //std::cout<<x1_array[point1_nu]<<"  "<<x1_array[0]<<std::endl;
    marker_points1.pose.position.y =y1_array[point1_nu-1];
    marker_points1.pose.position.z =0;

    // wenn type: POINT dann so machen und ohne utm_bias.pose.position....
    // geometry_msgs::Point p;
    // for(int i=0;i<num_pos_deg;i++){
    //    p.x = x_array[i];
    //     p.y = y_array[i];
    //     p.z = 0;//z_array[num_pos_deg-1];
    //    utm_bias.points.push_back(p);  // MarkerArray ma; ma.markers.push_back(mark
    //}

    //Marker array....?!
    // utm_bias_array.markers.push_back(utm_bias);

    marker_points1.id = point1_nu;
    marker_points1.type = visualization_msgs::Marker::SPHERE;
    marker_points1.scale.x = current_scale1;
    marker_points1.scale.y = current_scale1;
    marker_points1.scale.z = current_scale1;
    marker_points1.color.r = 0;//getColor("red");
    marker_points1.color.g =1.0;//getColor("green");
    marker_points1.color.b = 0;//getColor("blue");
    marker_points1.color.a = 1.0;

    publisher_name.publish(marker_points1);

//    /* Create a transformation and its broadcaster (rosrun rqt_tf_tree rqt_tf_tree)
//     * http://wiki.ros.org/tf/Tutorials/Introduction%20to%20tf */
//    tf::Transform transform;
//    tf::TransformBroadcaster br;
//    tf2_ros::StaticTransformBroadcaster transformBroadcast;

//    // publish the transform
//    transform.setOrigin(tf::Vector3(utm_bias.pose.position.x, utm_bias.pose.position.y, 0.0));
//    tf::Quaternion q;
//    q.setRPY(0, 0, 0);
//    transform.setRotation(q);
//    geometry_msgs::TransformStamped tf;
//    tf::transformTFToMsg(transform, tf.transform);
//    tf.header.frame_id = "/frames/map";
//    tf.child_frame_id = "/frames/rtkrcv_raw";
//    tf.header.stamp = ros::Time::now();
//    br.sendTransform(tf);
//    transformBroadcast.sendTransform(tf);

//    static tf::TransformBroadcaster br;
//    tf::Transform transform;
//    transform.setOrigin( tf::Vector3(utm_bias.pose.position.x, utm_bias.pose.position.y, 0.0) );
//    tf::Quaternion q;
//    q.setRPY(0, 0, 0);
//    transform.setRotation(q);
//    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/frames/map", "/frames/rtkrcv_raw"));

    //ROS_INFO_STREAM("Publishing frame: " << utm_bias.pose.position.x << " : " << utm_bias.pose.position.y);
    //ROS_INFO_STREAM("tf frame: " << tf::Vector3(utm_bias.pose.position.x, utm_bias.pose.position.y, 0.0) );

}


// prints points in height in green - very precise!
//void plot_up_point(ros::Publisher vis_point_pub){
//    // coordinate points:
//    visualization_msgs::Marker utm_bias;
//    // visualization_msgs::MarkerArray utm_bias_array;
//    utm_bias.header.frame_id ="/frames/map";
//    utm_bias.header.stamp = ros::Time::now();
//    utm_bias.ns ="points";
//    utm_bias.action =visualization_msgs::Marker::ADD;
//    utm_bias.pose.orientation.x = 0.0;
//    utm_bias.pose.orientation.y = 0.0;
//    utm_bias.pose.orientation.z = 0.0;
//    utm_bias.pose.orientation.w = 1.0;


//    //wenn type: POINT dann so machen und ohne utm_bias.pose.position....

//    for(int i=0;i<num_pos_deg;i++){
//        geometry_msgs::Point p;
//        p.x = x_array[i];
//        p.y = y_array[i];
//        p.z = z_array[i];
//        // std::cout<<"x_array[i]: "<<x_array[i]<<std::endl;
//        //std::cout<<"y_array[i]: "<<y_array[i]<<std::endl;
//        // std::cout<<"z_array[i]: "<<z_array[i]<<std::endl;
//        utm_bias.points.push_back(p);  // MarkerArray ma; ma.markers.push_back(mark
//    }

//    //Marker array....?!
//    // utm_bias_array.markers.push_back(utm_bias);

//    utm_bias.id = 5;
//    utm_bias.type = visualization_msgs::Marker::POINTS;
//    utm_bias.lifetime = ros::Duration();
//    utm_bias.scale.x = scale_up_point; // print it very precise
//    utm_bias.scale.y = scale_up_point;
//    utm_bias.scale.z = 0;
//    utm_bias.color.r = getColor("red");
//    utm_bias.color.g =getColor("green");
//    utm_bias.color.b = getColor("blue");
//    utm_bias.color.a = 1.0;
//    vis_point_pub.publish(utm_bias);
//}

//int i = 0;
//void last_point_highlight_ground(ros::Publisher vis_last_point_highlight_pub){
//    // coordinate points:
//    visualization_msgs::Marker utm_bias;
//    // visualization_msgs::MarkerArray utm_bias_array;
//    if(i==0){
//        utm_bias.header.frame_id ="/frames/map";
//        utm_bias.header.stamp = ros::Time::now();
//        utm_bias.ns ="points";
//        utm_bias.action =visualization_msgs::Marker::ADD;
//        utm_bias.pose.orientation.x = 0.0;
//        utm_bias.pose.orientation.y = 0.0;
//        utm_bias.pose.orientation.z = 0.0;
//        utm_bias.pose.orientation.w = 1.0;
//        utm_bias.pose.position.x =x_array[num_pos_deg-1];
//        utm_bias.pose.position.y =y_array[num_pos_deg-1];
//        utm_bias.pose.position.z =0;

//        utm_bias.id = 5000000;
//        utm_bias.type = visualization_msgs::Marker::SPHERE;
//        utm_bias.scale.x = scale_ground_point*2;
//        utm_bias.scale.y = scale_ground_point*2;
//        utm_bias.scale.z = scale_ground_point*2;
//        utm_bias.color.r = getColor("red");
//        utm_bias.color.g =getColor("green");
//        utm_bias.color.b = getColor("blue");
//        utm_bias.color.a = 1.0;
//        i=1;
//    }
//    else if(i==1){
//        utm_bias.header.frame_id ="/frames/map";
//        utm_bias.header.stamp = ros::Time::now();
//        utm_bias.ns ="points";
//        utm_bias.action =visualization_msgs::Marker::ADD;
//        utm_bias.pose.orientation.x = 0.0;
//        utm_bias.pose.orientation.y = 0.0;
//        utm_bias.pose.orientation.z = 0.0;
//        utm_bias.pose.orientation.w = 1.0;
//        utm_bias.pose.position.x =x_array[num_pos_deg-1];
//        utm_bias.pose.position.y =y_array[num_pos_deg-1];
//        utm_bias.pose.position.z =0;

//        utm_bias.id = 5000000;
//        utm_bias.type = visualization_msgs::Marker::SPHERE;
//        utm_bias.scale.x = scale_ground_point*2;
//        utm_bias.scale.y = scale_ground_point*2;
//        utm_bias.scale.z = scale_ground_point*2;
//        utm_bias.color.r = 0.0;
//        utm_bias.color.g = 0.0;
//        utm_bias.color.b = 0.0;
//        utm_bias.color.a = 1.0;
//        i=0;
//    }
//    vis_last_point_highlight_pub.publish(utm_bias);
//}

void lineal(ros::Publisher vis_ks,int arrow_length,double	color_r,double color_g,double color_b,double intensity_a){
    // lineal - dimension:

    visualization_msgs::Marker arrow_x_10, arrow_y_10,arrow_z_10;
    arrow_x_10.header.frame_id = arrow_y_10.header.frame_id= arrow_z_10.header.frame_id="/map";
    arrow_x_10.header.stamp =arrow_y_10.header.stamp =arrow_z_10.header.stamp = ros::Time::now();
    arrow_x_10.ns = arrow_y_10.ns=arrow_z_10.ns="lineal_arrows";
    arrow_x_10.action =arrow_y_10.action = arrow_z_10.action =visualization_msgs::Marker::ADD;
    arrow_x_10.pose.orientation.w =arrow_y_10.pose.orientation.w= arrow_z_10.pose.orientation.w = 1.0;

    geometry_msgs::Point p_start; p_start.x=0;p_start.y=0;p_start.z=1;
    geometry_msgs::Point p_end_x_10; p_end_x_10.x=arrow_length;p_end_x_10.y=0;p_end_x_10.z=1;
    geometry_msgs::Point p_end_y_10; p_end_y_10.x=0;p_end_y_10.y=arrow_length;p_end_y_10.z=1;
    geometry_msgs::Point p_end_z_10; p_end_z_10.x=0;p_end_z_10.y=0;p_end_z_10.z=1+arrow_length;

    arrow_x_10.points.push_back(p_start);
    arrow_x_10.points.push_back(p_end_x_10);
    arrow_y_10.points.push_back(p_start);
    arrow_y_10.points.push_back(p_end_y_10);
    arrow_z_10.points.push_back(p_start);
    arrow_z_10.points.push_back(p_end_z_10);

    arrow_x_10.id=id_counter_arrow; id_counter_arrow++;
    arrow_y_10.id=id_counter_arrow; id_counter_arrow++;
    arrow_z_10.id=id_counter_arrow; id_counter_arrow++;
    arrow_x_10.type =  arrow_y_10.type=  arrow_z_10.type = visualization_msgs::Marker::ARROW;
    arrow_x_10.scale.x = arrow_y_10.scale.x =arrow_z_10.scale.x  =0.25;
    arrow_x_10.scale.y =arrow_y_10.scale.y= arrow_z_10.scale.y=(int)(arrow_length/10);
    arrow_x_10.scale.z = arrow_y_10.scale.z=arrow_z_10.scale.z=0.0;
    arrow_x_10.color.r =arrow_y_10.color.r = arrow_z_10.color.r =color_r;
    arrow_x_10.color.g =arrow_y_10.color.g =arrow_z_10.color.g=color_g;
    arrow_x_10.color.b = arrow_y_10.color.b =arrow_z_10.color.b=color_b ;
    arrow_x_10.color.a =arrow_y_10.color.a =arrow_z_10.color.a =intensity_a;


    // text_x
    visualization_msgs::Marker text_x_10;
    text_x_10.header.frame_id ="/map";
    text_x_10.header.stamp = ros::Time::now();
    text_x_10.ns ="lineal_text";
    text_x_10.action = visualization_msgs::Marker::ADD;
    text_x_10.pose.orientation.x = 0.0;
    text_x_10.pose.orientation.y = 0.0;
    text_x_10.pose.orientation.z = 0.0;
    text_x_10.pose.orientation.w = 0.0;
    text_x_10.pose.position.x = (int) (arrow_length/3);
    text_x_10.pose.position.y=  (int)   (arrow_length/3);
    text_x_10.pose.position.z=1;

    text_x_10.id=id_counter_arrow;  id_counter_arrow++;
    text_x_10.text =std::to_string(arrow_length)+"m";
    text_x_10.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_x_10.scale.x = 0;
    text_x_10.scale.y = 0;
    text_x_10.scale.z = 4;
    text_x_10.color.r = 0.0f;
    text_x_10.color.g =0.0f;
    text_x_10.color.b = 0.0f;
    text_x_10.color.a = 1.0;

    vis_ks.publish(text_x_10);
    vis_ks.publish(arrow_x_10);
    vis_ks.publish(arrow_y_10);
    vis_ks.publish(arrow_z_10);
}

void distance_to_origin_in_UTM_m(double latitude_center,double longitude_center){

  geometry_msgs::Point distance_point;

  // for every new message calculate all points again. This is in particular important if a new origin is set!
  for (int i=0;i<point1_nu;i++){
    distance_point = latlong2coordinates(lat1_array[i],long1_array[i],current_center_lat,current_center_long);
    x1_array[i] = distance_point.x;
    y1_array[i] = distance_point.y;
  }
  z1_array[point1_nu] = alt1_array[point1_nu]; //altitude does not have to be calculated
}

void points1_Callback(const rviz_maps::rviz_scale::ConstPtr& msg)
{
  lat1_array[point1_nu]=msg->latitude;
  long1_array[point1_nu]=msg->longitude;
  alt1_array[point1_nu]=msg->altitude;
  status1_array[point1_nu]=msg->status;

  current_scale1=msg->scale;
  current_center_lat=msg->lat_center;
  current_center_long=msg->long_center;

  point1_nu++;
  distance_to_origin_in_UTM_m(current_center_lat,current_center_long);
  ROS_INFO("point comming in");
}

int main(int argc, char** argv)
{
    ROS_INFO_ONCE("[rviz_sub.cpp] [int main(..)] : started rviz plotting points");
    ros::init(argc, argv, "rviz_listener");
    ros::NodeHandle nh;

    //Points:
    ros::Publisher pub_points1 = nh.advertise<visualization_msgs::Marker>( "points1", 10 );
    //ros::Publisher vis_last_point_highlight_pub = nh.advertise<visualization_msgs::Marker>( "topic_last_point_highlight", 10 );

    //Koordinate Systems:
    ros::Publisher vis_ks_10= nh.advertise<visualization_msgs::Marker>( "topic_ks_10", 10 );
    ros::Publisher vis_ks_50= nh.advertise<visualization_msgs::Marker>( "topic_ks_50", 10 );
    ros::Publisher vis_ks_100= nh.advertise<visualization_msgs::Marker>( "topic_ks_100", 10 );
    ros::Publisher vis_ks_500= nh.advertise<visualization_msgs::Marker>( "topic_ks_500", 10 );
    ros::Publisher vis_ks_1000= nh.advertise<visualization_msgs::Marker>( "topic_ks_1000", 10 );
    ros::Publisher vis_ks_5000= nh.advertise<visualization_msgs::Marker>( "topic_ks_5000", 10 );

    ros::Subscriber sub_points1 = nh.subscribe("/points1_publish", 10, points1_Callback);


    ros::Rate r(10); // points should not come in faster than 10Hz. // TODO.
    while (ros::ok()) {

        plot_points1(pub_points1);
        //last_point_highlight_ground(vis_last_point_highlight_pub);

        lineal(vis_ks_10,10,0.47,0.47,0.47,1);
        lineal(vis_ks_50,50,0.47,0.47,0.47,1);
        lineal(vis_ks_100,100,0.47,0.47,0.47,1);
        lineal(vis_ks_500,500,0.47,0.47,0.47,1);
        lineal(vis_ks_1000,1000,0.47,0.47,0.47,1);
        lineal(vis_ks_5000,5000,0.47,0.47,0.47,1);

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}

