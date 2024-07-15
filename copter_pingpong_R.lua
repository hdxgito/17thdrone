-- This script makes the drone go forward and backward at a defined distance and number of times.
-- The stages are:
-- 0) Change to Guided mode
-- 1) Takeoff to the height defined by takeoff_alt
-- 2) Wait until reaching the takeoff altitude
-- 3) Go forward to the defined distance
-- 4) Go back to the initial position
-- 5) Change to Land mode

local takeoff_alt = 3         -- Takeoff height
local copter_guided_mode_num = 4
--local copter_althold_mode_num = 2
local copter_land_mode_num = 9
local stage = 0
local count = 0               -- Number of times the drone has gone forward
local max_count = 2           -- Maximum number of times the drone should go forward
local ping_pong_distance = 10 -- Distance up to which the drone should go forward (m)
local vel = 1                 -- Drone velocity (m/s)
local yaw_setpoint = 0


function update()
    -- Checking if the drone is armed
    if not arming:is_armed() then
         -- Reset state when disarmed
        stage = 0
        gcs:send_text(6, "Arming")
    else
        if stage == 0 then      -- Stage0: Change to guided mode
              if vehicle:set_mode(copter_guided_mode_num) then -- Change to Guided mode
                  stage = stage + 1
              end

        elseif stage == 1 then -- Stage1: Takeoff
        gcs:send_text(6, "Taking off")
            if vehicle:start_takeoff(takeoff_alt) then
                stage = stage + 1
            end

            local current_yaw = math.deg(ahrs:get_yaw())
            if vehicle.do_set_yaw then
                if not vehicle:do_set_yaw(current_yaw, 0, 0, 0) then
                    gcs:send_text(6, "Failed to set yaw using do_set_yaw")
                end
            else
                gcs:send_text(6, "do_set_yaw not available")
            end 

        elseif stage == 2 then -- Stage2: Check if the vehicle has reached the target altitude  
            local home = ahrs:get_home()  
            local curr_loc = ahrs:get_position()  
            if home and curr_loc then   
                local vec_from_home = home:get_distance_NED(curr_loc)  
                gcs:send_text(6, "Altitude above home: " .. tostring(math.floor(-vec_from_home:z())))  
                if math.abs(takeoff_alt + vec_from_home:z()) < 1 then  
                    -- Get the current yaw angle when the drone reaches the target altitude  
                    current_yaw = ahrs:get_yaw()  
                    stage = stage + 1  
                end  
            end  

        elseif stage == 3 then -- Stage3: Moving Forward
        -- If the number of times is exceeded, switch to stage5
            --if count >= max_count then
                --stage = stage + 2
            --end

                -- Calculate velocity vector
                local target_vel = Vector3f()
                target_vel:x(vel)
                target_vel:y(0)
                target_vel:z(0)
          
                -- Send velocity request
                if not vehicle:set_target_velocity_NED(target_vel) then
                    gcs:send_text(6, "Failed to execute velocity command")
                end

                -- Checking if the stop point is reached
                local home = ahrs:get_home()
                local curr_loc = ahrs:get_position()
                if home and curr_loc then 
                    local vec_from_home = home:get_distance_NED(curr_loc)
                    gcs:send_text(6, "Distance from home: " .. tostring(math.floor(vec_from_home:x())))
                    if math.abs(ping_pong_distance - vec_from_home:x()) < 1 then
                        --count = count + 1
                        stage = stage + 1
                    end
                end
            
            elseif stage == 4 then -- Stage4: Moving Back  

                -- Send MAV_CMD_CONDITION_YAW message to maintain current yaw  
                local msg = {  
                    -- Parameters for the MAVLink message  
                    param1 = 0,  
                    param2 = 0,  
                    param3 = 0,  
                    param4 = 0,  
                    param5 = 0,  
                    param6 = 0,  
                    param7 = 0,  
                    command = mavlink.MAV_CMD_NAV_WAYPOINT,  
                    target_system = 1,  
                    target_component = 1  
                }  
                  
                -- Send the MAVLink message  
                mavlink:send(mavlink.MAVLINK_MSG_ID_COMMAND_LONG, msg)  
             
                -- Calculate velocity vector  
                local target_vel = Vector3f()  
                target_vel:x(-vel)  
                target_vel:y(0)  
                target_vel:z(0)  
              
                -- Send velocity request  
                if not vehicle:set_target_velocity_NED(target_vel) then  
                    gcs:send_text(6, "Failed to execute velocity command")  
                end  
                  
                -- Checking if the stop point is reached  
                local home = ahrs:get_home()  
                local curr_loc = ahrs:get_position()  
                if home and curr_loc then   
                    local vec_from_home = home:get_distance_NED(curr_loc)  
                    gcs:send_text(6, "Distance from home: " .. tostring(math.floor(vec_from_home:x())))  
                    if math.abs(vec_from_home:x()) < 1 then  
                        stage = stage - 1  
                    end  
                end  

        elseif stage == 5 then -- Stage5: Moving right
            -- If the number of times is exceeded, switch to stage5
                --if count >= max_count then
                    --stage = stage + 2
                --end
    
                    -- Calculate velocity vector
                    local target_vel = Vector3f()
                    target_vel:x(0)
                    target_vel:y(vel)
                    target_vel:z(0)
    
                    -- Send velocity request
                    if not vehicle:set_target_velocity_NED(target_vel) then
                        gcs:send_text(6, "Failed to execute velocity command")
                    end
                    
                    -- Checking if the stop point is reached
                    local home = ahrs:get_home()
                    local curr_loc = ahrs:get_position()
                    if home and curr_loc then 
                        local vec_from_home = home:get_distance_NED(curr_loc)
                        gcs:send_text(6, "Distance from home: " .. tostring(math.floor(vec_from_home:x())))
                        if math.abs(ping_pong_distance - vec_from_home:y()) < 1 then
                            --count = count + 1
                            stage = stage + 1
                        end
                    end
                
            elseif stage == 6 then -- Stage6: Moving Back
                -- Calculate velocity vector
                local target_vel = Vector3f()
                target_vel:x(0)
                target_vel:y(-vel)
                target_vel:z(0)
    
                -- Send velocity request
                if not vehicle:set_target_velocity_NED(target_vel) then
                    gcs:send_text(6, "Failed to execute velocity command")
                end
                
                -- Checking if the stop point is reached
                local home = ahrs:get_home()
                local curr_loc = ahrs:get_position()
                if home and curr_loc then 
                    local vec_from_home = home:get_distance_NED(curr_loc)
                    gcs:send_text(6, "Distance from home: " .. tostring(math.floor(vec_from_home:x())))
                    if math.abs(vec_from_home:y()) < 1 then
                        stage = stage + 1
                    end
                end
            
          elseif stage == 7 then -- Stage7: Change to land mode
              vehicle:set_mode(copter_land_mode_num)
              stage = stage + 1
              gcs:send_text(6, "Finished pingpong, switching to land")
          end
        end
    return update, 100
end

return update()