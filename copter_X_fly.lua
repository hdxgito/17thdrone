-- This script makes the drone go forward and backward at a defined distance and number of times.
-- 各ステージは:
-- -1) 機首の向きを変えないようにするため、WP_YAW_BEHAVIOR の値を 0　に設定
-- 0) Guided modeに変更
-- 1) 離陸して設定した高さまで上昇
-- 2) 高度が設定した高さになるまで待ち
-- 3-10) 機首の向きを変えず前後左右に移動
-- 11) Land modeに変更して着陸
-- 12) WP_YAW_BEHAVIORの値を元の値に戻す

local takeoff_alt = 3         -- 離陸後の飛行高さを指定
local copter_guided_mode_num = 4    --Guided_modeにするパラメータ
local copter_land_mode_num = 9  --lamd_modeにするパラメータ
local stage = -1              -- ステージの初期値 -1
local ping_pong_distance = 5  -- ドローンが移動する距離を指定（ｍ）
local vel = 1                 -- ドローンの速度を指定 (m/s)
local original_yaw_behavior   -- To store the original WP_YAW_BEHAVIOR value

function update()
    -- アームされているか確認
    if not arming:is_armed() then
         -- Reset state when disarmed
        stage = -1
        gcs:send_text(6, "Arming")
    else
        if stage == -1 then     -- stage-1: WP_YAW_BEHAVIORの値を確認・記録してから0（機首の向きを変えない）に設定する
            original_yaw_behavior = param:get('WP_YAW_BEHAVIOR')
            if param:set('WP_YAW_BEHAVIOR', 0) then
                gcs:send_text(6, "Set WP_YAW_BEHAVIOR to 0")
                stage = stage + 1
            else
                gcs:send_text(6, "Failed to set WP_YAW_BEHAVIOR")
            end

        elseif stage == 0 then      -- Stage0: guided modeに変更
              if vehicle:set_mode(copter_guided_mode_num) then
                  stage = stage + 1
              end

        elseif stage == 1 then -- Stage1: Takeoff
        gcs:send_text(6, "Taking off")
            if vehicle:start_takeoff(takeoff_alt) then
                stage = stage + 1
            end
            
        elseif stage == 2 then -- Stage2: 機体が設定した高さに到達するまで待ち
            local home = ahrs:get_home()
            local curr_loc = ahrs:get_position()
            if home and curr_loc then 
                local vec_from_home = home:get_distance_NED(curr_loc)
                gcs:send_text(6, "Altitude above home: " .. tostring(math.floor(-vec_from_home:z())))
                if math.abs(takeoff_alt + vec_from_home:z()) < 1 then
                    stage = stage + 1
                end
            end
            
        elseif stage == 3 then -- Stage3: 前進する
                -- Calculate velocity vector
                local target_vel = Vector3f()
                target_vel:x(vel)   --X方向の正（北）に進むよう速度を指定
                target_vel:y(0)
                target_vel:z(0)

                -- 移動速度を指示する
                if not vehicle:set_target_velocity_NED(target_vel) then
                    gcs:send_text(6, "Failed to execute velocity command")
                end
                
                -- Checking if the stop point is reached
                local home = ahrs:get_home()
                local curr_loc = ahrs:get_position()
                if home and curr_loc then 
                    local vec_from_home = home:get_distance_NED(curr_loc)
                    gcs:send_text(6, "Distance from home: " .. tostring(math.floor(vec_from_home:x())))
                    if math.abs(ping_pong_distance - vec_from_home:x()) < 1 then    --設定した移動距離とhomeから実際に移動した距離を比較し1以下になったら目標地点に到達したと判断
                        stage = stage + 1
                    end
                end
            
        elseif stage == 4 then -- Stage4: homeに戻る
                -- Calculate velocity vector
                local target_vel = Vector3f()
                target_vel:x(-vel)  --X方向の負（南）に進むよう速度を指定
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
                    if math.abs(vec_from_home:x()) < 1 then --homeからの移動距離が1以下になったらhomeに到着したと判断
                        stage = stage + 1
                    end
                end

        elseif stage == 5 then -- Stage5: 後退する
                -- Calculate velocity vector
                local target_vel = Vector3f()
                target_vel:x(-vel)--X方向の負（南）に進むよう速度を指定
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
                    if math.abs(ping_pong_distance + vec_from_home:x()) < 1 then    --homeからの移動距離が負になるため比較の演算子を＋にする
                        stage = stage + 1
                    end
                end 
        
        elseif stage == 6 then -- Stage6: homeに戻る
                -- Calculate velocity vector
                local target_vel = Vector3f()
                target_vel:x(vel)    --X方向の正（北）に進むよう速度を指定
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
                    if math.abs(vec_from_home:x()) < 1 then --homeからの移動距離が1以下になったらhomeに到着したと判断
                        stage = stage + 1
                    end
                end               

       elseif stage == 7 then -- Stage7: 右側に移動する
                -- Calculate velocity vector
                local target_vel = Vector3f()
                target_vel:x(0)
                target_vel:y(vel)   --Y方向の正（東）に進むよう速度を指定
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
                    gcs:send_text(6, "Distance from home: " .. tostring(math.floor(vec_from_home:y())))
                    if math.abs(ping_pong_distance - vec_from_home:y()) < 1 then
                        stage = stage + 1
                    end
                end
            
        elseif stage == 8 then -- Stage8: homeに戻る
                -- Calculate velocity vector
                local target_vel = Vector3f()
                target_vel:x(0)
                target_vel:y(-vel)  --y方向の正（東）に進むよう速度を指定
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
                    gcs:send_text(6, "Distance from home: " .. tostring(math.floor(vec_from_home:y())))
                    if math.abs(vec_from_home:y()) < 1 then
                        stage = stage + 1
                    end
                end

        elseif stage == 9 then -- Stage9: 左側に移動する
                -- Calculate velocity vector
                local target_vel = Vector3f()
                target_vel:x(0)
                target_vel:y(-vel)  --Y方向の負（西）に進むよう速度を指定
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
                    gcs:send_text(6, "Distance from home: " .. tostring(math.floor(vec_from_home:y())))
                    if math.abs(ping_pong_distance + vec_from_home:y()) < 1 then    --homeからの移動距離が負になるため比較の演算子を＋にする
                        stage = stage + 1
                    end
                end
                
        elseif stage == 10 then -- Stage10: homeに戻る
                -- Calculate velocity vector
                local target_vel = Vector3f()
                target_vel:x(0)
                target_vel:y(vel)   --Y方向の正（東）に進むよう速度を指定
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
                    gcs:send_text(6, "Distance from home: " .. tostring(math.floor(vec_from_home:y())))
                    if math.abs(vec_from_home:y()) < 1 then
                        stage = stage + 1
                    end
                end
                
        elseif stage == 11 then -- Stage11: 着陸モードに切り替え着陸する
            vehicle:set_mode(copter_land_mode_num)
            stage = stage + 1
            gcs:send_text(6, "Finished movement pattern, switching to land")

        elseif stage == 12 then -- stage12: WP_YAW_BEHAVIORの値を元の値に戻す
            if param:set('WP_YAW_BEHAVIOR', original_yaw_behavior) then
                gcs:send_text(6, "Restored WP_YAW_BEHAVIOR to original value")
                stage = stage + 1
            else
                gcs:send_text(6, "Failed to restore WP_YAW_BEHAVIOR")
            end
        end
    end
    return update, 100
end

return update()