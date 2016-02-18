#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "tekscan_client/GetPressureMap.h"

#include <iostream>
#include <fstream>
#include <math.h>
using namespace std;



int main(int argc, char** argv){
  ros::init(argc, argv, "shadow_tf_listener");
  ros::NodeHandle node;
  
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  
  // >> PRESSURE MAP SERVICE
  ros::ServiceClient pressure_client = node.serviceClient<tekscan_client::GetPressureMap>("GetPressureMap");
  tekscan_client::GetPressureMap srv_pressure;
  srv_pressure.request.num = 0; 
    
  ros::Rate rate(10.0);
  
  ofstream pose_file;
  ofstream tactil_file;
  pose_file.open ("/home/aurova/Desktop/pruebas/resultados/positions.txt");
  if(pose_file.is_open())
    ROS_INFO("Archivo posiciones abierto");
   
  tactil_file.open ("/home/aurova/Desktop/pruebas/resultados/pressure.txt");
  if(tactil_file.is_open())
    ROS_INFO("Archivo presion abierto");
  
  int iteration = 0;
  double displacement = 0;

  while (node.ok()){
    geometry_msgs::TransformStamped transformSt;
    // get timestamp
    try{
      // obtener posiciones tips: 
      
      // transform -> get pose ff
      transformSt = tfBuffer.lookupTransform("forearm", "thtip",
                               ros::Time(0));      
      pose_file << transformSt.transform.translation.x << " " << transformSt.transform.translation.y << " " << transformSt.transform.translation.z << " ";
       
      // transform -> get pose ff
      transformSt = tfBuffer.lookupTransform("forearm", "fftip",
                               ros::Time(0));
      pose_file << transformSt.transform.translation.x << " " << transformSt.transform.translation.y << " " << transformSt.transform.translation.z << " ";

      // transform -> get pose mf
      transformSt = tfBuffer.lookupTransform("forearm", "mftip",
                               ros::Time(0));      
      pose_file << transformSt.transform.translation.x << " " << transformSt.transform.translation.y << " " << transformSt.transform.translation.z << " ";

      // transform -> get pose rf
      transformSt = tfBuffer.lookupTransform("forearm", "rftip",
                               ros::Time(0));      
      pose_file << transformSt.transform.translation.x << " " << transformSt.transform.translation.y << " " << transformSt.transform.translation.z << " ";

      // transform -> get pose lf
      transformSt = tfBuffer.lookupTransform("forearm", "lftip",
                               ros::Time(0));
      pose_file << transformSt.transform.translation.x << " " << transformSt.transform.translation.y << " " << transformSt.transform.translation.z << " ";

      pose_file << iteration << "\n";
      
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    
    
    // Obtener mapa de presión
    if (pressure_client.call(srv_pressure))
    {
        // guardar en archivos fuerza/presión
  	  tactil_file << srv_pressure.response.applied_force[0] << " " << srv_pressure.response.applied_force[1] << " " << srv_pressure.response.applied_force[2] << " " 
                  << srv_pressure.response.applied_force[3] << " " << srv_pressure.response.applied_force[4] << " ";

  	  tactil_file << srv_pressure.response.force_deviation[0] << " " << srv_pressure.response.force_deviation[1] << " " << srv_pressure.response.force_deviation[2] << " "
                  << srv_pressure.response.force_deviation[3] << " " << srv_pressure.response.force_deviation[4] << " ";

  	  tactil_file << iteration << "\n";

    }
    else
    {
      ROS_ERROR("Failed to call service pressure");
      return 1;
    }
    iteration++;
    rate.sleep();
  }
  
  pose_file.close();
  tactil_file.close();

  ROS_INFO("Archivo cerrado");
  return 0;
};
