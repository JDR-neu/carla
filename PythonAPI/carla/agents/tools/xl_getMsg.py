#!/usr/bin/env python
import carla
import random
import numpy as np
import math
import transforms3d
import sys
import os
from environment_model_msgs.msg import *
from ego_motion_msgs.msg import EgoVehicleMotion
from sensor_fusion_msgs.msg import ObjFusionInfo

def Message(player,map,world):
    # try:
    player_location = player.get_location()
    player_velocity = player.get_velocity()
    player_angular_velocity = player.get_angular_velocity()
    player_acceleration = player.get_acceleration()

    ego_vehicle_waypoint = map.get_waypoint(player_location, True)
    # print('player_location %s'%player_location)

    actor_list = world.get_actors()
    vehicle_list = actor_list.filter("*vehicle*")
    # print(vehicle_list)
        # get all vehicles and lights    
    obj_in_ego_lane = []
    obj_in_left_lane = []
    obj_in_right_lane = []

    #get obj in ego/left/right
    if vehicle_list is not None:
        
        for target_vehicle in vehicle_list:
            # do not account for the ego vehicle
            if target_vehicle.id == player.id:
                continue
            # if the object is not in our lane it's not an obstacle
            target_vehicle_waypoint = map.get_waypoint(target_vehicle.get_location(), True)
            # print("target location ", (target_vehicle.get_location().x,target_vehicle.get_location().y,target_vehicle.get_location().z), "ego location ", (player_location.x,player_location.y,player_location.z))
            # # print("target wp ",(target_vehicle_waypoint.transform.location.x,  target_vehicle_waypoint.transform.location.y, target_vehicle_waypoint.transform.location.z), "ego wp ", ego_vehicle_waypoint)
            # print("target waypoint id ", target_vehicle_waypoint.id, "ego waypoint id ", ego_vehicle_waypoint.id)
            # print("target road id ", target_vehicle_waypoint.road_id, "ego road id ", ego_vehicle_waypoint.road_id)
            # print("target lane id ", target_vehicle_waypoint.lane_id, "ego lane id ", ego_vehicle_waypoint.lane_id) 
            # print("target section id ", target_vehicle_waypoint.section_id, "ego section id ", ego_vehicle_waypoint.section_id)   
            if target_vehicle_waypoint.is_intersection:
                # print("========================is junction!!!!!!!!!=====================")
                targetLaneIDToUse = abs(target_vehicle_waypoint.lane_id)
            else:
                targetLaneIDToUse = target_vehicle_waypoint.lane_id
            
            if ego_vehicle_waypoint.is_intersection:
                egoLaneIDToUse = abs(ego_vehicle_waypoint.lane_id)
            else:
                egoLaneIDToUse = ego_vehicle_waypoint.lane_id
            
            if ego_vehicle_waypoint.get_left_lane() != None:
                if ego_vehicle_waypoint.get_left_lane().is_intersection:
                    egoLeftLaneIDToUse = abs(ego_vehicle_waypoint.get_left_lane().lane_id)
                else:
                    egoLeftLaneIDToUse = ego_vehicle_waypoint.get_left_lane().lane_id
            
            if ego_vehicle_waypoint.get_right_lane() != None:
                if ego_vehicle_waypoint.get_right_lane().is_intersection:
                    egoRightLaneIDToUse = abs(ego_vehicle_waypoint.get_right_lane().lane_id)
                else:
                    egoRightLaneIDToUse = ego_vehicle_waypoint.get_right_lane().lane_id
                
            if targetLaneIDToUse * egoLaneIDToUse < 0: 
                continue
            else:         
                if targetLaneIDToUse == egoLaneIDToUse:     
                    obj_in_ego_lane.append(target_vehicle)
                elif ego_vehicle_waypoint.get_left_lane() != None and targetLaneIDToUse == egoLeftLaneIDToUse:                  
                    obj_in_left_lane.append(target_vehicle)
                elif ego_vehicle_waypoint.get_right_lane() != None and targetLaneIDToUse == egoRightLaneIDToUse:
                    obj_in_right_lane.append(target_vehicle)
                # else:
                #     print("don't care about the cars in other lanes")
    else :
        print("==================no car====================")           

    obj_all = [obj_in_left_lane,obj_in_ego_lane,obj_in_right_lane]
    # print('obj_all %s' %obj_all)

    # translate matrix R
    pitch_player = math.radians(player.get_transform().rotation.pitch)
    roll_player = - math.radians(player.get_transform().rotation.roll)
    yaw_player =  - math.radians(player.get_transform().rotation.yaw) 
    # print('pitch %s' %pitch)
    # print('roll %s' %roll)
    # print('yaw_radians %s' %yaw)

    R = transforms3d.euler.euler2mat(roll_player,pitch_player,yaw_player).T
    # print('translation matrix %s' %R)
    environmentModel = EnvironmentModel()
    egoVehicleMotion = EgoVehicleMotion()   

    wayPoints = [ego_vehicle_waypoint.get_left_lane(),ego_vehicle_waypoint,ego_vehicle_waypoint.get_right_lane()]

    i = 0
    for laneIndex in range(3):
        player_loc = [0,0,0]
        player_v = [0,0,0]
        player_angularV = [0,0,0]
        player_a = [0,0,0]
        player_loc[0] = player_location.x
        player_loc[1] = - player_location.y
        player_loc[2] = player_location.z
        player_v[0] = player_velocity.x
        player_v[1] = -player_velocity.y
        player_v[2] = player_velocity.z
        player_angularV[0] = player_angular_velocity.x
        player_angularV[1] = -player_angular_velocity.y
        player_angularV[2] = player_angular_velocity.z
        player_a[0] = player_acceleration.x
        player_a[1] = -player_acceleration.y
        player_a[2] = player_acceleration.z
        
        # print("layer1 is running!!!!!!!!!!!!")

        if laneIndex == i:
            if wayPoints[i] == None:
                environmentModel.Lanes[i].IsValid_b = False
                i = i+1 
                # print("+++++++++++++++++++lane is not valid++++++++++++++++++++++++++")           
                continue
            else:
                environmentModel.Lanes[i].IsValid_b = True    
                # print("lane is valid")
                next_waypoints = []
                next_waypoints.append(wayPoints[i])

                for _ in range(100):
                    tmpwp = next_waypoints[-1]
                    nextwps = list(tmpwp.next(1))
                    if len(nextwps) > 0:
                        nextwp = nextwps[0]
                        next_waypoints.append(nextwp)   
                    else: continue
                # print('+++++++++++++lane ',laneIndex, 'has ', len(next_waypoints), 'waypoints++++++++++')
                if len(next_waypoints) > 0:    
                    # print("layer2 is running!!!!!!!!!!!!")   
                # if len(wayPoints[i].next(4.0)) > 0:
                #     next_waypoints.append(wayPoints[i].next(4.0)[0])
                #     if len(wayPoints[i].next(4.0)[0].next(4.0)) > 0:
                #         next_waypoints.append(wayPoints[i].next(4.0)[0].next(4.0)[0])
                #         if len(wayPoints[i].next(4.0)[0].next(4.0)[0].next(4.0)) > 0:
                #             next_waypoints.append(wayPoints[i].next(4.0)[0].next(4.0)[0].next(4.0)[0])

                    waypoint_loc_relative_x = []
                    waypoint_loc_relative_y = []
                    waypoint_loc_relative = []
                    for waypoint in next_waypoints:
                        x_rel = waypoint.transform.location.x
                        y_rel = -waypoint.transform.location.y
                        z_rel = waypoint.transform.location.z
                        rel = [x_rel,y_rel,z_rel]
                        # print('rel %s' %y_rel)
                        # print('player_loc %s'%player_loc)   
                        x_rel -= player_loc[0]
                        y_rel -= player_loc[1]
                        z_rel -= player_loc[2]
                        rel = [x_rel,y_rel,z_rel]

                        waypoint_loc_relative.append([x_rel,y_rel,z_rel])
                        # print('waypoint_loc_relative %s' %waypoint_loc_relative)
                    for waypoint in waypoint_loc_relative:
                        waypoint_loc_relative_x.append(np.dot(R,waypoint)[0]) 
                        waypoint_loc_relative_y.append(np.dot(R,waypoint)[1]) 

                    # print('obj_loc_relative_x %s' %obj_loc_relative_x)                  
                    # print('obj_loc_relative_y %s' %obj_loc_relative_y) 
                    p = np.poly1d(np.polyfit(waypoint_loc_relative_x,waypoint_loc_relative_y,3))
                    # print(p)            #Cubic polynomial                                
                    # print(p.coeffs)
                    if len(p.coeffs) == 4 :            #Cubic polynomial
                        lanePolyY0 = p.coeffs[3]
                        lanePolyPsi = math.atan(p.coeffs[2])
                        lanePolyC0 = (2 * p.coeffs[1])
                        lanePolyC1 = (6 * p.coeffs[0])
                    elif len(p.coeffs) == 3 :
                        lanePolyY0 = 0
                        lanePolyPsi = math.atan(p.coeffs[2])
                        lanePolyC0 = (2 * p.coeffs[1])
                        lanePolyC1 = (6 * p.coeffs[0])
                    # else:
                        # print("polynomial in one variable ")
                    # print("get central line!!!")
                    environmentModel.Lanes[i].Geometry_st.dY0_f = lanePolyY0
                    environmentModel.Lanes[i].Geometry_st.alpPsi_f = lanePolyPsi
                    environmentModel.Lanes[i].Geometry_st.kapC0_f = lanePolyC0
                    environmentModel.Lanes[i].Geometry_st.kapDxC1_f = lanePolyC1
                # print('waypoint1 %s'%next_waypoints[0])
                # print('waypoint2 %s'%next_waypoints[1])
                # print('waypoint3 %s'%next_waypoints[2])
                # print('waypoint4 %s'%next_waypoints[3])

                # print('waypoint_loc_relative_x %s'%waypoint_loc_relative_x)
                # print('waypoint_loc_relative_y %s'%waypoint_loc_relative_y)
                # print('waypoint_loc_relative %s'%waypoint_loc_relative)
                


                if len(obj_all[i]) != 0:
                    # print('obj_all %s'%obj_all[0])
                    # print('obj_all %s'%obj_all[1][0])
                    # print('obj_all %s'%obj_all[2][0])
                    # print("layer3 is running!!!!!!!!!!!!")
                    obj_list = []
                    for vehicle in obj_all[i]:
                        obj_id = vehicle.id
                        # print("target id is {}".format(obj_id))
                        x_pos_actor = vehicle.get_location().x
                        y_pos_actor = -vehicle.get_location().y
                        z_pos_actor = vehicle.get_location().z
                        x_v_actor = vehicle.get_velocity().x
                        y_v_actor = -vehicle.get_velocity().y
                        z_v_actor = vehicle.get_velocity().z
                        x_angular_v_actor = vehicle.get_angular_velocity().x
                        y_angular_v_actor = -vehicle.get_angular_velocity().y
                        z_angular_v_actor = vehicle.get_angular_velocity().z
                        x_a_actor = vehicle.get_acceleration().x
                        y_a_actor = -vehicle.get_acceleration().y
                        z_a_actor = vehicle.get_acceleration().z
                        roll_actor = - math.radians(vehicle.get_transform().rotation.roll)
                        pitch_actor =  math.radians(vehicle.get_transform().rotation.pitch)
                        yaw_actor =  - math.radians(vehicle.get_transform().rotation.yaw) 

                        dx = x_pos_actor - player_loc[0]
                        dy = y_pos_actor - player_loc[1]
                        dz = z_pos_actor - player_loc[2]
                        vx = x_v_actor
                        vy = y_v_actor
                        vz = z_v_actor
                        angularVx = x_angular_v_actor
                        angularVy = y_angular_v_actor
                        angularVz = z_angular_v_actor
                        ax = x_a_actor
                        ay = y_a_actor
                        az = z_a_actor
                        
                        headAngle = yaw_actor - yaw_player
                        
                        if headAngle > math.pi:
                            headAngle = headAngle - 2 * math.pi
                        if headAngle < -math.pi:
                            headAngle = headAngle + 2 * math.pi
                        
                        dx_dy_dz = [dx,dy,dz]
                        vx_vy_vz = [vx,vy,vz]
                        angularVx_vy_vz = [angularVx,angularVy,angularVz]
                        ax_ay_az = [ax,ay,az]

                        dx_dy_dz2 = np.dot(R,dx_dy_dz)
                        vx_vy_vz2 = np.dot(R,vx_vy_vz)
                        ax_ay_az2 = np.dot(R,ax_ay_az)
                        # print("layer4 is running!!!!!!!!!!!!")
                        # angularVx_vy_vz2 = np.dot(R,angularVx_vy_vz)
                        angularVx_vy_vz2 = [angularVx,angularVy,angularVz]

                        obj_list.append([obj_id,dx_dy_dz2[0],dx_dy_dz2[1],dx_dy_dz2[2],vx_vy_vz2[0],\
                                        vx_vy_vz2[1],vx_vy_vz2[2],angularVx_vy_vz2[0],angularVx_vy_vz2[1],\
                                        angularVx_vy_vz2[2],ax_ay_az2[0],ax_ay_az2[1],ax_ay_az2[2],\
                                        headAngle])
                    obj_list_filter = list(filter(lambda x:(x[1] < 250 or x[1] > -250 ),obj_list))

                    # print('obj_list %s'%obj_list)
                    
                    obj_sorted = sorted(obj_list_filter,key=lambda x:x[1]) #sort by dx
                    environmentModel.Lanes[i].ObjectNum_ub = len(obj_sorted)

                    # print('obj_sorted %s'%obj_sorted)
                    
                    if len(obj_sorted) != 0:
                        # print("layer5 is running!!!!!!!!!!!!")
                        for j in range(len(obj_sorted)):
                            # print(type(environmentModel.Lanes[i].ObjectsinLane_ast))
                            # print(len(environmentModel.Lanes[i].ObjectsinLane_ast))
                            environmentModel.Lanes[i].ObjectsinLane_ast.append(ObjFusionInfo())
                            # print('dxv %.2f'%obj_sorted[j][1])
                            dxv = min(obj_sorted[j][1],250)
                            dxv = max(dxv,-250)
                            dyv = min(obj_sorted[j][2],250)
                            dyv = max(dyv,-250)
                            environmentModel.Lanes[i].ObjectsinLane_ast[j].dxv_sw = int(dxv * 128)
                            environmentModel.Lanes[i].ObjectsinLane_ast[j].dyv_sw = int(dyv * 128)
                            vxv = min(obj_sorted[j][4],125)
                            vxv = max(vxv,-125)
                            vyv = min(obj_sorted[j][5],125)
                            vyv = max(vyv,-125)
                            environmentModel.Lanes[i].ObjectsinLane_ast[j].vxv_sw = int(vxv * 256)
                            environmentModel.Lanes[i].ObjectsinLane_ast[j].vyv_sw = int(vyv * 256)
                            angularVxv = min(obj_sorted[j][7] * math.pi / 180,1.9)
                            angularVxv = max(angularVxv,-1.9)
                            angularVyv = min(obj_sorted[j][7] * math.pi / 180,1.9)
                            angularVyv = max(angularVyv,-1.9)
                            angularVzv = min(obj_sorted[j][7] * math.pi / 180,1.9)
                            angularVzv = max(angularVzv,-1.9)
                            
                            environmentModel.Lanes[i].ObjectsinLane_ast[j].angularVxv_sw = int(angularVxv * 16384)
                            environmentModel.Lanes[i].ObjectsinLane_ast[j].angularVyv_sw = int(angularVyv * 16384)
                            environmentModel.Lanes[i].ObjectsinLane_ast[j].angularVzv_sw = int(angularVzv * 16384)
                            axv = min(obj_sorted[j][10],15)
                            axv = max(axv,-15)
                            ayv = min(obj_sorted[j][10],15)
                            ayv = max(ayv,-15)

                            environmentModel.Lanes[i].ObjectsinLane_ast[j].axv_sw = int(axv * 2048)
                            environmentModel.Lanes[i].ObjectsinLane_ast[j].ayv_sw = int(ayv * 2048)

                            # Todo: add object bounding box size
                            environmentModel.Lanes[i].ObjectsinLane_ast[j].length_uw = int(4.0 * 128)
                            environmentModel.Lanes[i].ObjectsinLane_ast[j].width_uw = int(2.0 * 128)
                            environmentModel.Lanes[i].ObjectsinLane_ast[j].headAngle_f = headAngle
                            
                            # print("id is {}".format(obj_sorted[j][0]), "type is ", type(environmentModel.Lanes[i].ObjectNum_ub))
                            environmentModel.Lanes[i].ObjectIdxinLane_aub[j] = obj_sorted[j][0]
                            # print("id after sorted is {}".format(int(obj_sorted[j][0])))
                            
                        
                        obj_filter = filter(lambda x:x[1] > 0,obj_sorted)
                        new1 = list(obj_filter) 
                    
                        if len(new1) != 0:
                            # print("-----------len(frontVehicle) is ",len(new1),"\n")
                            firstObjId = new1[0] 
                            # print("++++++++++++++++++++++++++You can see car in front++++++++++++++++++++++++++++++++")
                            environmentModel.Lanes[i].FirstFrontObjInLaneIdx_ub = obj_sorted.index(firstObjId)

                    else:
                        environmentModel.Lanes[i].FirstFrontObjInLaneIdx_ub = 128
                    # print('index %d'%environmentModel.Lanes[i].FirstFrontObjInLaneIdx_ub)
                    
                else:
                    environmentModel.Lanes[i].ObjectsinLane_ast = []
                    environmentModel.Lanes[i].FirstFrontObjInLaneIdx_ub = 128
                
                if wayPoints[i].left_lane_marking.type == carla.LaneMarkingType.Solid:
                    LeftBoundary = 1
                elif wayPoints[i].left_lane_marking.type == carla.LaneMarkingType.BottsDots:
                    LeftBoundary = 5
                else:
                    LeftBoundary = 3

                if wayPoints[i].right_lane_marking.type == carla.LaneMarkingType.Solid:
                    RightBoundary = 1
                elif wayPoints[i].right_lane_marking.type == carla.LaneMarkingType.BottsDots:
                    RightBoundary = 5
                else:
                    RightBoundary = 3

                if wayPoints[i].lane_type == carla.LaneType.OnRamp:
                    lane_type = 4
                elif wayPoints[i].lane_type == carla.LaneType.OffRamp:
                    lane_type = 5
                else:
                    lane_type = 0
                
                environmentModel.Lanes[i].LeftBoundaryType_ub = LeftBoundary
                environmentModel.Lanes[i].RightBoundaryType_ub = RightBoundary   
                environmentModel.Lanes[i].lane_type = lane_type
                environmentModel.Lanes[i].Geometry_st.dWidth_f = wayPoints[i].lane_width
                environmentModel.Lanes[i].ObjectIdxinLane_aub = [0] * 64
                # print("layer6 is running!!!!!!!!!!!!")
    
            i +=1  

    player_a2 = np.dot(R,player_a)
    player_angularV2 = np.dot(R,player_angularV)
    player_v2 = np.dot(R,player_v)


    player_a2[0] = min(player_a2[0],15)
    player_a2[0] = max(player_a2[0],-15)
    player_a2[1] = min(player_a2[1],15)
    player_a2[1] = max(player_a2[1],-15)
    player_angularV2[1] = player_angularV2[1] * math.pi/ 180
    player_angularV2[1] = min(player_angularV2[1],1.9)
    player_angularV2[1] = max(player_angularV2[1],-1.9)
    # print("egoVehicleMotion.vxvRef_sw is ",player_v[0],"egoVehicleMotion.psiDtOpt_sw is ",player_angularV[1],"\n")
    # print("egoVehicleMotion.axvRef_sw is ",player_a[0],"egoVehicleMotion.ayvRef_sw is ",player_a[1],"\n")
    egoVehicleMotion.vxvRef_sw = int(player_v2[0] * 256)
    egoVehicleMotion.axvRef_sw = int(player_a2[0] * 2048)
    egoVehicleMotion.ayvRef_sw = int(player_a2[1] * 2048)
    egoVehicleMotion.psiDtOpt_sw = int( player_angularV2[1] * 16384)

    egoVehicleMotion.carla_pos_x = player_loc[0]
    egoVehicleMotion.carla_pos_y = player_loc[1]
    egoVehicleMotion.carla_heading = yaw_player

    return environmentModel, egoVehicleMotion
    # finally:
    #     print('\ndestroying %d actors' % len(actors))
    #     for actor in actors:
    #         actor.destroy()


# if __name__ == '__main__':
#     Message()
