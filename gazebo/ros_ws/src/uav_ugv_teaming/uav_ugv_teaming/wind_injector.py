#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from roscopter_msgs.msg import State
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np
import math

class PerlinNoise:
    """Simple 2D Perlin noise implementation"""
    def __init__(self, seed=None):
        if seed is not None:
            np.random.seed(seed)
        
        # Create permutation table
        self.perm = np.arange(256, dtype=int)
        np.random.shuffle(self.perm)
        self.perm = np.concatenate([self.perm, self.perm])  # Duplicate for wrapping
    
    def fade(self, t):
        """Fade function: 6t^5 - 15t^4 + 10t^3"""
        return t * t * t * (t * (t * 6 - 15) + 10)
    
    def lerp(self, a, b, t):
        """Linear interpolation"""
        return a + t * (b - a)
    
    def grad(self, hash_val, x, y):
        """Gradient function"""
        # Take the hashed value and use it to determine gradient direction
        h = hash_val & 3
        u = x if h < 2 else y
        v = y if h < 2 else x
        return (u if (h & 1) == 0 else -u) + (v if (h & 2) == 0 else -v)
    
    def noise(self, x, y):
        """Get Perlin noise value at (x, y)"""
        # Find unit grid cell
        X = int(math.floor(x)) & 255
        Y = int(math.floor(y)) & 255
        
        # Get fractional part
        x -= math.floor(x)
        y -= math.floor(y)
        
        # Fade curves
        u = self.fade(x)
        v = self.fade(y)
        
        # Hash coordinates of 4 corners
        a = self.perm[X] + Y
        aa = self.perm[a]
        ab = self.perm[a + 1]
        b = self.perm[X + 1] + Y
        ba = self.perm[b]
        bb = self.perm[b + 1]
        
        # Blend results from 4 corners
        return self.lerp(
            self.lerp(self.grad(self.perm[aa], x, y),
                     self.grad(self.perm[ba], x - 1, y), u),
            self.lerp(self.grad(self.perm[ab], x, y - 1),
                     self.grad(self.perm[bb], x - 1, y - 1), u),
            v
        )

class WindInjector(Node):
    def __init__(self):
        super().__init__('wind_injector')

        # Declare and evaluate parameters
        paramList = [
            ('seed', 42),
            ('wind_scale', 5.0),  # Max wind speed (m/s)
            ('spatial_scale', 0.05),  # Noise frequency (1/m)
            ('base_wind_north', 0.0),  # Base wind N component
            ('base_wind_east', 0.0),  # Base wind E component
            ('base_wind_down', 0.0),  # Base wind D component
            ('min_height', 2.0),        # Minimum height to apply wind
            ('viz_rate', 20.0),  # Hz for visualization
            ('viz_grid_size', 50),  # Grid points for visualization
            ('viz_grid_spacing', 2.0),  # Meters between grid points
            ('viz_arrow_scale', 1.0), # Arrow scaling
            ('wind_raw_topic', '/sim/truth_wind_raw'), # input
            ('wind_topic', '/sim/truth_wind'), # output
            ('uav_state_topic', '/sim/roscopter/state'),
        ]
        for name, defaultValue in paramList:
            self.declare_parameter(name, defaultValue)
            setattr(self, name, self.get_parameter(name).value)
        
        self.base_wind = np.array([self.base_wind_north, self.base_wind_east, self.base_wind_down])

        # Create Perlin noise generators (one for each axis)
        self.noise_north = PerlinNoise(self.seed)
        self.noise_east = PerlinNoise(self.seed + 1)
        self.noise_down = PerlinNoise(self.seed + 2)
        
        # Current UAV position
        self.uav_position = {'n': 0.0, 'e': 0.0, 'd': 0.0}
        
        # Subscribers
        self.wind_raw_sub = self.create_subscription(
            Vector3Stamped,
            self.wind_raw_topic,
            self.wind_raw_callback,
            10
        )
        
        self.uav_state_sub = self.create_subscription(
            State,
            self.uav_state_topic,
            self.uav_state_callback,
            10
        )
        
        # Publishers
        self.wind_pub = self.create_publisher(
            Vector3Stamped,
            self.wind_topic,
            10
        )        
        self.viz_pub = self.create_publisher(
            MarkerArray,
            '/wind_visualization',
            10
        )
        
        # Visualization timer
        self.viz_timer = self.create_timer(1.0 / self.viz_rate, self.publish_visualization)
        
        self.get_logger().info('Wind Injector started')
        self.get_logger().info(f'  Seed: {self.seed}')
        self.get_logger().info(f'  Wind scale: {self.wind_scale} m/s')
        self.get_logger().info(f'  Spatial scale: {self.spatial_scale}')
        self.get_logger().info(f'  Base wind: N={self.base_wind[0]}, E={self.base_wind[1]}, D={self.base_wind[2]}')
    
    def uav_state_callback(self, msg):
        """Update UAV position for wind lookup"""
        self.uav_position['n'] = msg.p_n
        self.uav_position['e'] = msg.p_e
        self.uav_position['d'] = msg.p_d
    
    def get_wind_at_position(self, north, east, down):
        """
        Get wind vector at a specific position using Perlin noise
        
        Returns: (wind_north, wind_east, wind_down) in m/s
        """
        if down > -self.min_height:
            return 0.0, 0.0, 0.0

        # Scale position for noise lookup
        x = north * self.spatial_scale
        y = east * self.spatial_scale
        z = down * self.spatial_scale
        
        # Get noise values (-1 to 1)
        noise_n = self.noise_north.noise(x, y)
        noise_e = self.noise_east.noise(x + 100, y + 100)  # Offset for independence
        noise_d = self.noise_down.noise(x + 200, z + 200)  # Use altitude for down component
        
        # Scale to wind velocity and add base wind
        wind_n = self.base_wind[0] + noise_n * self.wind_scale
        wind_e = self.base_wind[1] + noise_e * self.wind_scale
        wind_d = self.base_wind[2] + noise_d * self.wind_scale * 0.3  # Less vertical wind
        
        return wind_n, wind_e, wind_d
    
        # return 5.0 * (1 if east > 0 else -1), 5.0 * (1 if north < 0 else -1), 0.0 # debug vortex

    def wind_raw_callback(self, msg):
        """Add Perlin noise wind to raw wind"""
        
        # Get wind at current UAV position
        wind_n, wind_e, wind_d = self.get_wind_at_position(
            self.uav_position['n'],
            self.uav_position['e'],
            self.uav_position['d']
        )
        
        # Add to raw wind (if any)
        wind_msg = Vector3Stamped()
        wind_msg.header.stamp = self.get_clock().now().to_msg()
        wind_msg.header.frame_id = 'NED'
        
        wind_msg.vector.x = msg.vector.x + wind_n
        wind_msg.vector.y = msg.vector.y + wind_e
        wind_msg.vector.z = msg.vector.z + wind_d
        
        self.wind_pub.publish(wind_msg)
    
    def publish_visualization(self):
        """Publish wind field visualization as arrows in RViz"""
        
        marker_array = MarkerArray()
        
        # Create grid of wind arrows centered on UAV, rounded to grid size
        center_n = self.viz_grid_spacing * (self.uav_position['n'] // self.viz_grid_spacing)
        center_e = self.viz_grid_spacing * (self.uav_position['e'] // self.viz_grid_spacing)
        # altitude = -self.uav_position['d']  # Visualize at UAV altitude
        altitude = 3.0 # Fixed altitude
        
        # Grid parameters
        half_size = self.viz_grid_size // 2
        
        marker_id = 0
        for i in range(-half_size, half_size):
            for j in range(-half_size, half_size):
                
                # Position in world
                n = center_n + i * self.viz_grid_spacing
                e = center_e + j * self.viz_grid_spacing
                d = -altitude  # NED frame
                
                # Get wind at this position
                wind_n, wind_e, wind_d = self.get_wind_at_position(n, e, d)
                
                # Skip if wind is very small
                wind_magnitude = math.sqrt(wind_n**2 + wind_e**2)
                if wind_magnitude < 0.1:
                    continue
                
                # Create arrow marker
                marker = Marker()
                marker.header.frame_id = 'world'
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = 'wind_field'
                marker.id = marker_id
                marker.type = Marker.ARROW
                marker.action = Marker.ADD
                
                # Arrow start point
                marker.pose.position.x = n
                marker.pose.position.y = -e  # NED East to NWU West
                marker.pose.position.z = altitude
                
                # Arrow direction (pointing in wind direction)
                yaw = math.atan2(wind_e, wind_n)
                marker.pose.orientation.z = -math.sin(yaw / 2.0)
                marker.pose.orientation.w = math.cos(yaw / 2.0)
                
                # Arrow size (length proportional to wind speed)
                marker.scale.x = wind_magnitude * self.viz_arrow_scale # Length
                marker.scale.y = 0.1 * self.viz_arrow_scale # Shaft diameter
                marker.scale.z = 0.1 * self.viz_arrow_scale # Head diameter
                
                # Color (blue to red based on speed)
                speed_ratio = min(1.0, wind_magnitude / self.wind_scale)
                if speed_ratio < 0.5:
                    marker.color.r = 0.0
                    marker.color.g = 2 * speed_ratio
                    marker.color.b = 1.0 - 2 * speed_ratio
                else:
                    marker.color.r = -1.0 + 2 * speed_ratio
                    marker.color.g = 2.0 - 2 * speed_ratio
                    marker.color.b = 0.0
                marker.color.a = 0.7  # Semi-transparent
                
                marker.lifetime = rclpy.duration.Duration(seconds=1.0).to_msg()
                
                marker_array.markers.append(marker)
                marker_id += 1
        
        # Publish marker array
        self.viz_pub.publish(marker_array)
        
        # Log current wind at UAV
        wind_n, wind_e, wind_d = self.get_wind_at_position(
            self.uav_position['n'],
            self.uav_position['e'],
            self.uav_position['d']
        )
        wind_speed = math.sqrt(wind_n**2 + wind_e**2)
        wind_dir = math.degrees(math.atan2(wind_e, wind_n))
        
        self.get_logger().info(
            f'Wind at UAV: {wind_speed:.2f} m/s from {wind_dir:.0f}°, '
            f'vertical: {wind_d:.2f} m/s',
            throttle_duration_sec=5.0
        )

def main():
    rclpy.init()
    node = WindInjector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()