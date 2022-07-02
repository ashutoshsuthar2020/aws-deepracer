from numpy import array
import math
def reward_function(params):
    # Example of penalize steering, which helps mitigate zig-zag behaviors
    # Read input parameters
    x = params['x']
    y = params['y']
    waypoints = params['waypoints']
    closest_waypoints = params['closest_waypoints']
    heading = params['heading']
    steps = params['steps']
    progress = params['progress']
    all_wheels_on_track = params['all_wheels_on_track']
    is_reversed = params['is_reversed']
    distance_from_center = params['distance_from_center']
    track_width = params['track_width']
    steering_angle = params['steering_angle']
    speed = params['speed']
    def direction_checker(waypoints,closest_waypoints,heading):
        reward =1
        #check direction - refer aws pdf
        next_point = waypoints[closest_waypoints[1]]
        prev_point = waypoints[closest_waypoints[0]]
        # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
        track_direction = math.atan2(next_point[1] - prev_point[1], next_point[0] - prev_point[0])
        # Convert to degree
        track_direction = math.degrees(track_direction)
    
        # Calculate the difference between the track direction and the heading direction of the car
        direction_diff = abs(track_direction - heading)
        if direction_diff > 180:
            direction_diff = 360 - direction_diff
    
        # Penalize the reward if the difference is too large
        DIRECTION_THRESHOLD = 10.0
        if direction_diff > DIRECTION_THRESHOLD:
            reward *= 0.5
    
        return float(reward)

    def award_if_faster_than_expected(steps,progress):
        #https://medium.com/analytics-vidhya/training-deepracer-for-speed-52007aa03af5
        # Total num of steps we want the car to finish the lap, it will vary depends on the track length
        TOTAL_NUM_STEPS = 300
    
        # Initialize the reward with typical value
        reward = 1.0
    
        # Give additional reward if the car pass every 100 steps faster than expected
        if (steps % 100) == 0 and progress > (steps / TOTAL_NUM_STEPS) * 100 :
            reward += 10.0
    
        return float(reward)

    # Calculate 3 marks that are farther and father away from the center line
    marker_1 = 0.1 * track_width
    marker_2 = 0.25 * track_width
    marker_3 = 0.5 * track_width

    # Give higher reward if the car is closer to center line and vice versa
    def punish(all_wheels_on_track,is_reversed):
        reward = 1
        if not all_wheels_on_track:
            reward = 1e-3
            return float(reward)
        if distance_from_center <= marker_1:
            reward *= 1.0
        elif distance_from_center <= marker_2:
            reward *= 0.5
        elif distance_from_center <= marker_3:
            reward *= 0.1
        else:
            reward = 1e-3  # likely crashed/ close to off track
        if is_reversed:
            reward-=20
        return float(reward)  

    def pure_pursuit(closest_waypoints, waypoints, heading, x,y):    
        reward = 1e-3

        rabbit = [0,0]
        pointing = [0,0]
        # Reward when yaw (car_orientation) is pointed to the next waypoint IN FRONT.        
        # Find nearest waypoint coordinates
        rabbit = waypoints[closest_waypoints[1]]
        
        radius = math.hypot(x - rabbit[0], y - rabbit[1])
    
        pointing[0] = x + (radius * math.cos(heading))
        pointing[1] = y + (radius * math.sin(heading))
        
        vector_delta = math.hypot(pointing[0] - rabbit[0], pointing[1] - rabbit[1])
        
        # Max distance for pointing away will be the radius * 2
        # Min distance means we are pointing directly at the next waypoint
        # We can setup a reward that is a ratio to this max.
            
        if vector_delta == 0:
            reward += 1
        else:
            reward += ( 1 - ( vector_delta / (radius * 2)))

        return reward
    reward = direction_checker(waypoints,closest_waypoints,heading)+award_if_faster_than_expected(steps,progress)+punish(all_wheels_on_track,is_reversed)+pure_pursuit(closest_waypoints,waypoints,heading,x,y)
    return float(reward)
