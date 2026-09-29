# CP and VFH Project Guide

Tai lieu nay mo ta source hien tai cua du an Collision Prevention (CP) va VFH
Avoidance trong PX4-Autopilot. Muc dich la de nguoi doc co the theo duoi du
lieu tu LiDAR, hieu tung lop xu ly, xem code thay doi o dau, chay test va tiep
tuc phat trien.

## 0. Pham vi va trang thai

- Source goc: /home/nhut/PX4-Autopilot.
- Phan du an: CollisionPrevention, ObstacleMath, VFH library,
  FlightTaskAuto, GZ LiDAR bridge va cac test lien quan.
- Lan tao tai lieu nay chi bo sung tai lieu, khong sua logic C++ hay tham so bay.
- Worktree co cac thay doi khac cua nguoi dung o GZ submodule, driver TFNova,
  EKF study, Zenoh, file backup va toolchain. Cac thay doi do khong thuoc pham
  vi CP/VFH va khong duoc tu y sua.
- Cac dong PX4 khong thay doi khong duoc copy het. Moi khoi thay doi quan trong
  duoc trich lai va giai thich theo thu tu cau lenh. Muc 15 co lenh in full
  patch khi can xem tung dong.

## 1. Buc tranh tong the

Du an co hai muc tieu:

1. Stop an toan: neu obstacle nam tren huong bay, CP tinh van toc an toan theo
   khoang phanh. Neu sensor mat, Auto bi ep van toc XY bang 0 va sau khoang mot
   giay yeu cau Hold.
2. VFH Avoid tuy chon: neu CP_AUTO_MODE=1, VFH chon mot sector an toan, gioi
   han toc do theo corridor va tao van toc XY moi. Neu khong co path hoac qua
   gan nguong emergency, he thong Stop.

CP_AUTO_EN la cong tac tong. CP_AUTO_MODE=0 la Stop mac dinh. Chi gia tri
CP_AUTO_MODE=1 moi bat VFH; gia tri khac khong mo VFH.

VFH hien la local planner cho obstacle tinh trong tam LiDAR. No khong thay
Navigator, khong sua current_seq, khong chen waypoint, khong lam SLAM va
khong co global replanning.

## 2. So do kien truc PX4

~~~text
Gazebo/hardware LiDAR
        |
        v
GZBridge hoac sensor driver
        |
        | obstacle_distance, BODY_FRD, 72 sector
        v
CollisionPrevention
   |                    \
   | obstacle map         \ safe velocity / Stop
   v                       v
VFHObstacleAdapter       FlightTaskAuto
   |                       ^
   v                       |
VFHPlanner            VFH velocity override
   |                       |
   v                       |
DirectionalSpeedLimiter   |
   |                       |
   v                       |
VFHVelocitySetpoint -------+
              |
              v
       PositionSmoothing
              |
              v
      trajectory_setpoint
              |
              v
        mc_pos_control

Navigator Mission ---> FlightTaskAuto va VFH target waypoint
~~~

Quyen han:

| Lop | So huu viec gi | Khong lam viec gi |
| --- | --- | --- |
| GZBridge | Doc LaserScan, gom sector, publish uORB | Khong chon huong bay |
| CollisionPrevention | Fusion sensor, stale, frame, tilt, obstacle map | Khong lam global planner |
| ObstacleMath | Toan chieu tia va goc | Khong publish topic |
| VFHObstacleAdapter | Doi cm/deg BODY_FRD sang m/rad local NED | Khong chon sector |
| VFHPlanner | Histogram, hysteresis, chon huong | Khong dieu khien motor |
| DirectionalSpeedLimiter | Tinh clearance corridor va toc do | Khong thay the planner |
| ControlPolicy | State active, clear, no-path | Khong doc sensor |
| FlightTaskAuto | Chon Stop/VFH, sensor loss, override | Khong sua Navigator current_seq |
| PositionSmoothing | Lam muot trajectory | Khong biet y nghia obstacle |
| mc_pos_control | Dieu khien cuoi | Khong tinh VFH |

## 3. Flow runtime

### 3.1 Tu LiDAR den obstacle map

1. Gazebo tao LaserScan.
2. GZBridge gom tia vao sector 5 do.
3. Trong mot sector, lay khoang cach nho nhat de khong bo sot vat can mong.
4. Scan FLU cua Gazebo duoc dao mang de phu hop BODY_FRD cua PX4.
5. Bridge publish obstacle_distance toi da 72 sector.
6. CollisionPrevention cap nhat attitude va obstacle map.
7. BODY_FRD ray duoc chieu len mat phang ngang bang quaternion roll/pitch/yaw.
8. Cac sector chieu chong nhau duoc gop; reading gan nhat duoc giu lai.
9. Map fused duoc publish thanh obstacle_distance_fused.

### 3.2 Auto Stop

1. FlightTaskAuto::update() goi _updateCollisionPreventionConstraints().
2. Neu CP Auto tat, waypoint type khong phu hop hoac dang RTL, ham return.
3. Neu khong co data, force zero XY, set brake acceleration va speed limit 0.
4. Warning critical duoc throttle mot lan moi giay.
5. Neu mat data hon mot giay va khong phai Auto Loiter, gui Hold.
6. Neu co data, CP tinh SafeVelocityResult theo huong toi waypoint.
7. Toc do va acceleration cua PositionSmoothing bi gioi han.
8. Khi critical, force zero velocity. Logic stop mode cu van duoc giu.

### 3.3 Auto VFH Avoid

1. CP_AUTO_MODE=1 bat nhanh VFH.
2. Huong muc tieu la vector tu UAV den _triplet_target, waypoint hien tai cua
   Navigator.
3. Adapter kiem tra timestamp/frame/resolution va doi geometry sang local NED.
4. Planner mo rong obstacle theo vehicle radius + safety margin, hysteresis va
   tinh cost.
5. Sector co cost thap nhat va nam trong goc cho phep duoc chon.
6. Limiter tinh clearance chieu, tru khoang delay va tinh van toc phanh.
7. Neu corridor <= nguong dung, VFH tra emergency stop.
8. Neu co path va policy active, VFH tao velocity XY moi.
9. Khi override, XY waypoint trong mang smoothing dat NAN de khong cong them
   feed-forward huong waypoint cu. Z van position-controlled.
10. Khi obstacle het, corridor phai clear lien tuc mot giay roi VFH release.
11. Sau release, Auto tiep tuc toi waypoint hien tai; smoothing tu keo UAV ve line.

## 4. Quy uoc du lieu

obstacle_distance:
- distances[72]: centimet.
- increment va angle_offset: degree.
- frame VFH yeu cau MAV_FRAME_BODY_FRD.
- BODY_FRD: X tien, Y phai, Z xuong.
- UINT16_MAX: unknown/not observed.
- max_distance + 1: observed nhung khong thay obstacle.
- 0: observed nhung obstacle gan hon minimum range; khong phai free.

VFH:
- Khoang cach: meter.
- Goc: radian, wrap trong [-pi, pi].
- 72 bin, moi bin 5 do, du 360 do.
- Input sau adapter dung local NED: X North, Y East.

Doi huong:
~~~text
direction_local = atan2(East, North)
direction_body  = wrap_pi(direction_local - vehicle_yaw)
~~~

Yaw doi frame mot lan tai adapter. Roll/pitch thay doi thanh phan ngang cua ray,
vi vay phai dung quaternion de sua ca range va bearing truoc khi gan sector.

## 5. Danh sach file va thay doi

### 5.1 Collision Prevention

- src/lib/collision_prevention/CollisionPrevention.hpp
  - them getSensorDelay() va getObstacleMapBodyFrame();
  - them overload calculateSafeVelocity co min_distance_to_keep;
  - _addObstacleSensorData() nhan attitude;
  - luu attitude timestamp.
- CollisionPrevention.cpp
  - update() cap nhat attitude;
  - Auto dung CP_AUTO_DIST, Manual van dung CP_DIST;
  - attitude phai gan timestamp lidar trong 100 ms;
  - chieu tung ray BODY_FRD;
  - merge overlap sector, giu reading gan nhat;
  - giu semantics zero, unknown, clear sentinel.
- ObstacleMath.hpp/.cpp them project_ray_on_horizontal_plane().
- collisionprevention_params.c them parameter Auto/VFH/brake tilt.
- CollisionPreventionTest.cpp va ObstacleMathTest.cpp them regression cho
  tilt, yaw, overlap, sentinel, resolution va safe velocity.

### 5.2 VFH library

- VFHPlanner.hpp/.cpp: config, histogram, hysteresis, cost va selected direction.
- VFHObstacleAdapter.hpp/.cpp: freshness, frame, don vi va geometry.
- VFHVelocitySetpoint.hpp/.cpp: selected angle thanh velocity XY.
- VFHDirectionalSpeedLimiter.hpp/.cpp: clearance corridor va speed governor.
- VFHControlPolicy.hpp/.cpp: Stop/VFH state, clear timeout, no-path timeout.
- CMakeLists.txt build library va 6 unit test executable.
- src/lib/CMakeLists.txt add subdirectory vfh_avoidance.

### 5.3 Auto va SITL

- FlightTaskAuto.hpp/.cpp: chon Stop/VFH, sensor-loss, no-path, emergency,
  velocity override.
- Auto/CMakeLists.txt: link VFH library vao Auto task.
- GZBridge.cpp: gom scan 5 do, lay min return va SIM_GZ_LIDAR_F.
- Tools/vfh_avoidance/analyze_vfh_ulog.py: ULog sang HTML.
- Tools/simulation/gz/worlds/vfh_obstacle_course.sdf: block, wall, L, cot,
  gate va U dead end.

## 6. Cac khoi code quan trong

Phan nay trich lai cac khoi code quan trong. Mot so khoi duoc rut gon voi `...`
chi de thay doi thu tu tham so; dung `nl -ba` va `git diff` de xem code day du.

### 6.1 Parameter

File: src/lib/collision_prevention/collisionprevention_params.c.

~~~c
PARAM_DEFINE_INT32(CP_AUTO_EN, 0);
PARAM_DEFINE_FLOAT(CP_AUTO_DIST, 6.0f);
PARAM_DEFINE_INT32(CP_AUTO_MODE, 0);
PARAM_DEFINE_FLOAT(CP_VFH_MAX_VEL, 3.0f);
PARAM_DEFINE_FLOAT(CP_VFH_STOP_DIST, 1.5f);
PARAM_DEFINE_FLOAT(CP_MAX_ANGLE, 10.0f);
PARAM_DEFINE_FLOAT(CP_BRAKE_ANGLE, 30.0f);
~~~

- CP_AUTO_EN: master switch CP Auto.
- CP_AUTO_DIST: Auto stopping envelope va moc slowdown, hien 6 m.
- CP_AUTO_MODE: 0 Stop, 1 VFH Avoid.
- CP_VFH_MAX_VEL: can tren toc do do VFH tao ra, hien 3 m/s.
- CP_VFH_STOP_DIST: nguong emergency corridor, hien 1.5 m.
- CP_MAX_ANGLE: acceleration binh thuong qua tilt 10 do.
- CP_BRAKE_ANGLE: acceleration phanh/no-data qua tilt 30 do.
- Day la gioi han acceleration setpoint; hard tilt limit cuoi cung van nam o
  mc_pos_control.

### 6.2 Safe velocity va CP API

File: CollisionPrevention.hpp:90-108 va CollisionPrevention.cpp:122-146.

~~~cpp
float getSensorDelay() const { return _param_cp_delay.get(); }
const obstacle_distance_s &getObstacleMapBodyFrame() const
{
    return _obstacle_map_body_frame;
}

SafeVelocityResult calculateSafeVelocity(...)
{
    return calculateSafeVelocity(..., _param_cp_dist.get());
}

SafeVelocityResult calculateSafeVelocity(..., float min_distance_to_keep)
{
    SafeVelocityResult result{};
    const Vector2f travel_dir = travel_direction.unit_or_zero();

    if (!_obstacle_data_present || !travel_dir.isAllFinite()
        || !measured_velocity.isAllFinite()
        || max_jerk <= FLT_EPSILON
        || max_braking_accel <= FLT_EPSILON) {
        return result;
    }

    const float effective_min_distance_to_keep = math::max(
        _obstacle_map_body_frame.min_distance * 0.01f,
        min_distance_to_keep);
~~~

Theo thu tu:

1. Getter cho VFH dung map CP va CP_DELAY, khong tao subscription copy rieng.
2. Overload cu giu hanh vi Manual bang CP_DIST.
3. Overload moi cho Auto truyen CP_AUTO_DIST.
4. Input khong hop le tra valid=false.
5. Minimum su dung la max(sensor minimum, parameter minimum).

CP tinh:
~~~text
stopping_distance = obstacle_distance
                  - effective_min_distance_to_keep
                  - delay_distance
max_velocity_towards_obstacle = f(max_jerk, max_braking_accel,
                                  stopping_distance)
safe_velocity = max_velocity_towards_obstacle / travel_projection
~~~

delay_distance gom measured_velocity * CP_DELAY va tuoi data neu bin dang co
obstacle. stopping_distance <= 0 tao critical va safe velocity 0.

### 6.3 Tilt compensation

File: ObstacleMath.cpp:57-80.

~~~cpp
const Vector3f ray_body{cosf(ray_angle_body), sinf(ray_angle_body), 0.f};
const Vector3f ray_world = q_world_vehicle.rotateVector(ray_body);

horizontal_scale = ray_world.xy().norm();
if (!PX4_ISFINITE(horizontal_scale)
    || horizontal_scale <= FLT_EPSILON) {
    return false;
}

horizontal_scale = math::constrain(horizontal_scale, FLT_EPSILON, 1.f);
const float horizontal_angle_world = atan2f(ray_world(1), ray_world(0));
horizontal_angle_body = matrix::wrap_pi(
    horizontal_angle_world - vehicle_yaw);
~~~

- Tao ray ngang BODY_FRD.
- Quaternion dua ray vao NED.
- norm XY la he so slant-to-horizontal.
- atan2 tinh bearing NED; tru yaw dua ve body-horizontal.
- Ba tia center/lower/upper deu duoc chieu. Scale nho nhat duoc dung de
  conservative.
- Attitude lech lidar khong qua 100 ms moi duoc dung. Quaternion loi thi bo
  qua reading in-range; sentinel van giu semantics fail-safe.
- Neu hardware da tu bu tilt, khong duoc bu lan hai.

Cong thuc:
~~~text
ray_body = [cos(theta), sin(theta), 0]
ray_ned  = R(q) * ray_body
scale    = norm(ray_ned.x, ray_ned.y)
d_horizontal = d_slant * scale
angle_horizontal_body = wrap_pi(atan2(ray_ned.y, ray_ned.x) - yaw)
~~~

### 6.4 BODY_FRD map merge

File: CollisionPrevention.cpp:376-521.

~~~cpp
const int source_bin_count =
    static_cast<int>(roundf(360.f / obstacle.increment));
const int source_distance_count =
    sizeof(obstacle.distances) / sizeof(obstacle.distances[0]);

if (source_bin_count < 1
    || source_bin_count > source_distance_count
    || fabsf(360.f / obstacle.increment - source_bin_count) > 1e-3f) {
    return;
}

for (int j = 0; j < source_bin_count; j++) {
    if (obstacle.distances[j] == UINT16_MAX) {
        continue;
    }

    // Chieu tam va hai bien sector.
    // Tim fused bin ma interval ray phu len.
    // Gop reading gan nhat truoc _enterData().
}
~~~

- Resolution phai chia tron 360 do va khong duoc doc qua 72 phan tu.
- Unknown khong ghi de du lieu.
- Center va hai bien sector giu overlap sau roll/pitch.
- Overlap helper xu ly ca interval cat qua 0/360.
- Nhieu tia cung fused bin: reading gan nhat duoc giu.
- _enterData() tiep tuc xu ly priority giua cac sensor.

### 6.5 GZBridge

File: GZBridge.cpp, laserScanCallback().

~~~cpp
const float sector_width_rad = math::radians(5.f);
const int samples_per_sector = math::max(
    1, static_cast<int>(roundf(msg.angle_step() / sector_width_rad)));

for (int sector = 0; sector < sector_count; ++sector) {
    uint16_t closest_distance = range_max_cm + 1;

    for (int sample = 0; sample < samples_per_sector; ++sample) {
        const float range = msg.ranges(sector * samples_per_sector + sample);
        if (PX4_ISFINITE(range) && range >= msg.range_min()
            && range <= msg.range_max()) {
            closest_distance = math::min(
                closest_distance,
                static_cast<uint16_t>(roundf(range * 100.f)));
        }
    }

    ds_array[sector] = closest_distance;
}
~~~

- Moi sector rong 5 do.
- Lay min return de vat can mong khong bi average che mat.
- Return loi dung clear sentinel.
- Dao mang FLU cua Gazebo sang BODY_FRD PX4.
- SIM_GZ_LIDAR_F dung de test dropout.

### 6.6 VFHPlanner histogram

File: VFHPlanner.cpp:15-145.

~~~cpp
if (!input.observed[bin]
    || !PX4_ISFINITE(input.distances[bin])
    || input.distances[bin] <= FLT_EPSILON) {
    primary_histogram[bin] = 1.f;
    continue;
}

const float distance = input.distances[bin];
if (distance >= _config.influence_distance) {
    continue;
}

const float density = math::constrain(
    1.f - distance / _config.influence_distance, 0.f, 1.f);
const float protected_radius = getProtectedRadius();
const float enlargement_angle = asinf(
    math::constrain(protected_radius / distance, 0.f, 1.f));
const int enlargement_bins = static_cast<int>(
    ceilf(enlargement_angle / input.angle_increment));

for (int offset = -enlargement_bins;
     offset <= enlargement_bins; ++offset) {
    const int affected_bin = wrapBin(static_cast<int>(bin) + offset);
    primary_histogram[affected_bin] = math::max(
        primary_histogram[affected_bin], density);
}
~~~

- Unknown, NaN, zero la occupied, khong phai free.
- Obstacle xa hon influence_distance bo qua.
- density lon khi vat can gan.
- protected_radius = vehicle_radius + safety_margin.
- asin(protected_radius/distance) la nua goc mo rong obstacle.
- ceilf va wrapBin bao phu du bin, ke ca bien 0/360.

Hysteresis:
~~~cpp
if (primary_histogram[bin] > threshold_high) {
    _blocked[bin] = true;
} else if (primary_histogram[bin] < threshold_low) {
    _blocked[bin] = false;
}
~~~

Vung giua low/high giu trang thai cu, tranh bat/tat do noise.

### 6.7 Cost va chon sector

~~~cpp
if (_blocked[bin]) {
    continue;
}

const float candidate_direction = matrix::wrap_pi(
    input.angle_offset + static_cast<float>(bin) * input.angle_increment);

if (angleDifference(candidate_direction, input.target_direction)
    > _config.max_target_direction_change) {
    continue;
}

float cost = _config.target_weight
           * angleDifference(candidate_direction, input.target_direction)
           + _config.current_weight
           * angleDifference(candidate_direction, input.current_direction);

if (_has_previous_direction) {
    cost += _config.previous_weight
          * angleDifference(candidate_direction, _previous_direction);
}
~~~

- Blocked sector bi loai ngay.
- Sector nam qua xa waypoint bi loai; day la ly do chu U co the valid=false.
- target_weight uu tien tien do mission.
- current_weight giam doi huong dot ngot.
- previous_weight giam dao dong trai/phai.
- Cost nho nhat duoc chon va luu cho chu ky sau.

Cong thuc:
~~~text
J = Wt * angle(target, candidate)
  + Wc * angle(current, candidate)
  + Wp * angle(previous, candidate)
~~~

### 6.8 Adapter

File: VFHObstacleAdapter.cpp:16-55.

~~~cpp
if (obstacle_map.frame != obstacle_distance_s::MAV_FRAME_BODY_FRD
    || obstacle_map.timestamp == 0
    || now - obstacle_map.timestamp > timeout
    || fabsf(obstacle_map.increment - 5.f) > 0.01f) {
    return false;
}

converted.angle_offset = matrix::wrap_pi(
    math::radians(obstacle_map.angle_offset) + frame_rotation);
converted.angle_increment = math::radians(obstacle_map.increment);
converted.target_direction = target_direction;
converted.current_direction = current_direction;
~~~

Frame sai, stale, resolution sai thi fail. cm doi sang m. UINT16_MAX va vuot
clear sentinel la unknown. Target/current da o local NED nen khong xoay them.

Trong FlightTaskAuto, frame_rotation la yaw. Neu yaw sai, vector selected co the
sai theo local NED; day la ly do can yaw-quality gate truoc hardware flight toc
do cao.

### 6.9 Directional speed limiter

File: VFHDirectionalSpeedLimiter.cpp:16-101.

~~~cpp
for (size_t bin = 0; bin < VFHPlanner::BIN_COUNT; ++bin) {
    const float sector_angle = input.angle_offset
        + static_cast<float>(bin) * input.angle_increment;
    const float angle_difference = fabsf(
        matrix::wrap_pi(sector_angle - selected_direction));

    if (angle_difference > CORRIDOR_HALF_ANGLE) {
        continue;
    }

    if (!input.observed[bin] || !PX4_ISFINITE(input.distances[bin])
        || input.distances[bin] <= FLT_EPSILON) {
        return output;
    }

    const float projected_distance = input.distances[bin]
        * cosf(angle_difference);
    nearest_projected_distance = math::min(
        nearest_projected_distance, projected_distance);
}
~~~

- Chi corridor +/-15 do quanh selected direction duoc dung de tinh clearance.
- Unknown trong corridor lam output invalid, fail-safe ve Stop.
- d*cos(delta) la khoang cach chieu theo velocity.
- Obstacle ngoai corridor hien co the khong lam giam toc. Do do code chua dam
  bao clearance 1.5 m tren moi huong.

Phan phanh:
~~~cpp
const float required_stop_distance = math::max(
    emergency_stop_distance, protected_radius);

if (nearest_projected_distance <= required_stop_distance) {
    output.emergency_stop = true;
    return output;
}

const float distance_after_delay = nearest_projected_distance
    - required_stop_distance - current_speed * delay;

const float physics_speed_limit = sqrtf(
    math::max(0.f, 2.f * brake_acceleration * distance_after_delay));
~~~

- required_stop_distance lon hon giua CP_VFH_STOP_DIST va protected radius.
- Neu da vao nguong, tra emergency, khong ep van toc toi thieu 0.1 m/s.
- Tru sensor/controller delay.
- Dung v^2 = 2*a*d.
- CP_AUTO_DIST la slowdown distance.

### 6.10 Policy

File: VFHControlPolicy.cpp:9-84.

~~~cpp
if (!input.path_available) {
    if (_avoidance_active) {
        if (_no_path_start_us == 0) {
            _no_path_start_us = input.now_us;
        } else if (input.now_us - _no_path_start_us
                   > NO_PATH_TIMEOUT_US) {
            _no_path_stop = true;
            _avoidance_active = false;
            output.no_path_event = true;
        }
    }
} else {
    _no_path_start_us = 0;

    if (_no_path_stop) {
        _no_path_stop = false;
        _avoidance_active = false;
    }
}
~~~

- Stop mode reset policy.
- No-path qua 500 ms khi avoidance active tao event va latch stop.
- Path quay lai xoa latch trong cung chu ky; VFH co the resume.
- Target corridor clear lien tuc 1 giay moi release avoidance trong phan con lai.
- Day la hysteresis thoi gian, khac histogram hysteresis.

### 6.11 FlightTaskAuto

File: FlightTaskAuto.cpp:872-1145.

~~~cpp
bool vfh_velocity_override = false;
_updateCollisionPreventionConstraints(
    force_zero_velocity_setpoint, vfh_velocity_override);

if (vfh_velocity_override) {
    for (Vector3f &waypoint : waypoints) {
        waypoint(0) = NAN;
        waypoint(1) = NAN;
    }
}
~~~

Output override bao cho caller biet VFH da ghi de XY. Dat XY waypoint bang NAN
de smoothing khong cong them huong waypoint cu; Z van duoc giu.

Chon mode:
~~~cpp
const bool vfh_avoidance_enabled = _param_cp_auto_mode.get()
    == static_cast<int32_t>(CollisionPreventionAutoMode::vfh_avoid);

if (vfh_avoidance_enabled) {
    vfh_path_available = _updateVFH(
        travel_direction, vfh_velocity_setpoint,
        avoidance_required, brake_accel_xy, vfh_emergency_stop);
} else {
    _resetVFH();
}
~~~

Stop mode dung safe velocity cu. VFH mode goi Adapter -> Planner -> Limiter ->
VelocitySetpoint.

Emergency:
~~~cpp
if (vfh_emergency_stop) {
    force_zero_velocity_setpoint = true;
    _position_smoothing.setMaxAccelerationXY(brake_accel_xy);
    _position_smoothing.setCruiseSpeed(0.f);
    _position_smoothing.setMaxVelocityXY(0.f);
    return;
}
~~~

Emergency VFH chi dung; khong tao vector escape ngang. Day la chu y an toan hien
tai, khong phai logic repulsive vector.

Toc do:
~~~cpp
const float vfh_speed_limit = math::min(
    math::min(math::min(_mc_cruise_speed,
                        _param_mpc_xy_vel_max.get()),
              _param_cp_vfh_max_vel.get()),
    directional_speed);

_position_smoothing.setCruiseSpeed(vfh_speed_limit);
_position_smoothing.setMaxVelocityXY(vfh_speed_limit);
_velocity_setpoint.xy() = vfh_velocity_setpoint;
vfh_velocity_override = true;
~~~

Toc do cuoi la min cua mission speed, PX4 max speed, VFH max speed va
directional speed. Smoothing van bao ve acceleration/jerk.

Sensor loss:
~~~cpp
if (!_collision_prevention.obstacleDataPresent()) {
    _resetVFH();
    force_zero_velocity_setpoint = true;
    _position_smoothing.setMaxAccelerationXY(brake_accel_xy);
    _position_smoothing.setCruiseSpeed(0.f);
    _position_smoothing.setMaxVelocityXY(0.f);
    return;
}
~~~

Sau do code giu warning 1 Hz va request Hold sau 1 s. Mat data luon uu tien hon
VFH.

## 7. Hanh vi theo mode

| Tinh huong | Stop mode | VFH Avoid mode |
| --- | --- | --- |
| Khong obstacle | Bay Auto binh thuong | VFH co the khong active |
| Obstacle tren huong | Giam safe velocity | Chon corridor free |
| Obstacle ngoai corridor | CP theo huong mission | Limiter hien co the bo qua |
| Corridor <= 1.5 m | CP co the critical | VFH emergency stop |
| Khong free sector | Dung theo CP | No-path policy va dung |
| Mat sensor | Zero XY, warning, Hold sau 1 s | Zero XY, reset, warning, Hold |
| RTL | Auto CP loai khoi nhanh nay | VFH khong can thiep RTL |

CP_DIST la Manual CP. CP_AUTO_DIST la Auto. CP_VFH_STOP_DIST la threshold
emergency cua VFH, khac stopping envelope legacy.

## 8. Waypoint va line mission

Khi UAV dang toi WP2:

1. _triplet_target la WP2.
2. Obstacle lam VFH chon vector lech nhung cost van huong toi WP2.
3. Corridor clear mot giay thi VFH release.
4. Auto van dung WP2; PositionSmoothing tao vector cheo ve WP2/line.
5. Neu Navigator da xac nhan WP2 reached, Navigator tang current_seq sang WP3.

VFH khong tu skip waypoint. Viec ve line nhanh hay cham phu smoothing, acceptance
radius va off-track logic cua PX4. Vi vay hien tuong UAV bay ngang ve line khong
phai VFH tu chen waypoint.

## 9. Parameter hien tai

| Parameter | Gia tri source | Tac dung |
| --- | ---: | --- |
| CP_AUTO_EN | 0 | Bat/tat CP Auto |
| CP_AUTO_MODE | 0 | 0 Stop, 1 VFH Avoid |
| CP_DIST | -1 mac dinh PX4 | Manual CP |
| CP_AUTO_DIST | 6.0 m | Auto slowdown/stopping envelope |
| CP_VFH_MAX_VEL | 3.0 m/s | Can tren VFH speed |
| CP_VFH_STOP_DIST | 1.5 m | Emergency corridor |
| CP_MAX_ANGLE | 10 deg | Tilt acceleration binh thuong |
| CP_BRAKE_ANGLE | 30 deg | Tilt acceleration phanh |
| CP_DELAY | theo PX4 | Delay sensor/controller |

Cac so nay la diem test SITL, khong phai chung nhan hardware. Clearance con phu
thuoc brake acceleration, latency, noise, goc scan, hinh dang obstacle va pose.

## 10. Test hien co

CMake dang ky 6 VFH executable:

1. unit-VFHPlanner
2. unit-VFHArchitecture
3. unit-VFHObstacleAdapter
4. unit-VFHVelocitySetpoint
5. unit-VFHDirectionalSpeedLimiter
6. unit-VFHControlPolicy

Coverage: 72 sector, resolution loi, obstacle rong, U/no-free-sector, wrap
0/360, unknown/zero/stale/dropout, frame rotation, tilt, yaw cancellation,
overlap merge, corridor speed, delay, 1.5 m, 0.79 m, mode policy va reset.

Lenh:
~~~bash
cd /home/nhut/PX4-Autopilot
make tests TESTFILTER=VFH
make tests TESTFILTER=CollisionPrevention
ctest --test-dir build/px4_sitl_test --output-on-failure \
  -R 'VFH|CollisionPrevention|ObstacleMath|VelocitySmoothing|PositionSmoothing'
git diff --check
~~~

Ket qua da ghi nhan:

- CollisionPrevention: 31/31 pass.
- ObstacleMath: pass.
- VFH: source hien co 6 executable; README ghi lan moi nhat 6/6 pass.
- Regression gan nhat sau tilt compensation: 10/10 pass.
- CCACHE_DISABLE=1 make px4_sitl: build/link pass.

Lan tao guide nay chi sua tai lieu, khong tu chay lai test. Source thay doi tiep
thi phai chay lai lenh truoc khi ket luan pass moi.

## 11. SITL va debug

Chay:
~~~bash
make px4_sitl gz_x500_lidar_2d
make px4_sitl gz_x500_lidar_2d_vfh_obstacle_course
~~~

Listener:
~~~bash
listener obstacle_distance 1
listener obstacle_distance_fused 1
listener vehicle_attitude 1
listener vehicle_local_position 1
listener vehicle_status 1
~~~

Can doi chieu raw/fused frame, offset, increment, timestamp; roll/pitch; velocity;
mode transition; va VFH log path/avoid/target/selected/speed/dist/free.

Bat SIM_GZ_LIDAR_F de test:
1. Dropout ngan hon freshness timeout: khong Hold.
2. Data stale: zero XY va warning.
3. Mat data hon 1 s: request Hold.
4. Chuyen lai Mission khi sensor van tat: lai bi chan.

## 12. ULog va HTML

~~~bash
cd /home/nhut/PX4-Autopilot
find build -type f -name '*.ulg' -printf '%T@ %p\n' \
  | sort -nr | head
python3 Tools/vfh_avoidance/analyze_vfh_ulog.py \
  /absolute/path/to/flight.ulg \
  --output /absolute/path/to/flight.vfh.html
~~~

Khong dung /path/to/flight.ulg literal; thay bang duong dan that.

Nen xem vehicle_local_position, vehicle_local_position_setpoint,
obstacle_distance, obstacle_distance_fused, collision_constraints,
vehicle_attitude, vehicle_status va position_setpoint_triplet.

HTML co the danh dau speed vuot VFH max, clearance thap, dropout dai, tilt cao
va topic thieu. VFH control hien la terminal text, khong phai custom ULog topic;
vi vay HTML chua co selected_direction/cost/free_sector_count theo moi sample.

## 13. Ket qua log quan trong

Log: build/px4_sitl_default/rootfs/log/2026-09-10/09_12_18.ulg.

- Raw/fused deu 72 sector, increment 5 do, BODY_FRD.
- Mot tia raw 1074 cm sau tilt con khoang 1039 cm.
- Minimum fused gan nhat khoang 81 cm.
- Van toc nguy hiem khoang 3.15 m/s; obstacle body khoang 350 do trong khi
  selected direction cach gan 97 do.
- Limiter chi xem +/-15 do quanh selected, nen obstacle ngoai corridor khong lam
  speed giam nhu ky vong.

Ket luan: tilt compensation dang chay, nhung limiter chua bao phu 360 do. Khong
duoc ket luan rang VFH giu 1.5 m moi huong chi tu min_distance.

## 14. Gioi han va phan chua lam

1. DirectionalSpeedLimiter chi tinh clearance trong corridor da chon.
2. Chua co repulsive vector truc tiep vao velocity.
3. Chua co CP_VFH_CLEARANCE rieng cho muc tieu 1.5 m moi huong.
4. Chua co second safe-velocity governor chay lai CP tren selected VFH direction.
5. Adapter cho phep map tuoi 500 ms, limiter moi dung CP_DELAY; tuoi tung sector
   chua duoc cong day du.
6. Chua co yaw-quality gate rieng.
7. Chua co custom uORB VFH debug topic.
8. Chua hardware flight test.
9. RTL dang duoc loai khoi CP Auto path.

Cac muc tren khong phai thay doi dang co. Neu muon lam tiep, phai them unit test
truoc, sau do moi sua limiter va FlightTaskAuto.

## 15. In full patch va xem tung dong

~~~bash
cd /home/nhut/PX4-Autopilot
git status --short
git diff --name-only
git diff -- \
  src/lib/CMakeLists.txt \
  src/lib/collision_prevention \
  src/modules/flight_mode_manager/tasks/Auto \
  src/modules/simulation/gz_bridge/GZBridge.cpp
~~~

File VFH moi dang untracked, xem bang:
~~~bash
nl -ba src/lib/vfh_avoidance/VFHPlanner.cpp
nl -ba src/lib/vfh_avoidance/VFHObstacleAdapter.cpp
nl -ba src/lib/vfh_avoidance/VFHDirectionalSpeedLimiter.cpp
nl -ba src/lib/vfh_avoidance/VFHControlPolicy.cpp
nl -ba src/modules/flight_mode_manager/tasks/Auto/FlightTaskAuto.cpp \
  | sed -n '872,1145p'
~~~

Dung git diff de xem file tracked va nl -ba de xem file moi. Khi file moi duoc
git add, git diff --cached se in ca patch cua no.

## 16. Quy trinh cap nhat

Sau moi step da chot:

1. Cap nhat flow va bang file.
2. Ghi parameter/uORB/frame moi.
3. Them unit test truoc khi sua behavior.
4. Chay VFH, CollisionPrevention va regression.
5. Chay SITL voi toc do thap roi tang dan.
6. Luu ULog, mo HTML, doi chieu raw/fused/attitude/velocity/mode.
7. Ghi limitation va dieu kien cho step ke tiep.
8. Chi danh dau hoan thanh khi code, test va validation deu dat.

## 17. Tai lieu lien quan

- src/lib/vfh_avoidance/README.md
- STEP4_TEST_REPORT.md
- docs/collision_prevention_safety_layer_report.md
- Tools/vfh_avoidance/analyze_vfh_ulog.py

Lich su: 2026-09-13 tao guide tong hop source, flow, code excerpts, test, log,
backlog va bo sung English comments trong cac khoi CP/VFH quan trong.
