import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from tf2_ros import Buffer, TransformListener
import numpy as np
import torch
from collections import deque
import random

from robot_control.ddpg import DDPG
from robot_control.gym_gazebo_env import GazeboEnv

def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class RLPathFollower(Node):

    def __init__(self):
        super().__init__('rl_path_follower')

        # Subscribers and publishers
        self.path_sub = self.create_subscription(
            Path,
            '/planned_path',
            self.path_callback,
            10
        )
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Path storage
        self.path = []

        # Robot pose (assumed known / simplified)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0  # radians

        self.get_logger().info("RL Path Follower Node Started")


    def path_callback(self, msg: Path):
        self.path = msg.poses
        self.get_logger().info(f"Received new path with {len(self.path)} waypoints")       
        
        num_episodes=1000
        max_steps=1000
        warmup_steps=1000
        update_every=50
        log_interval=10

        env = GazeboEnv(world_name='depot', sim_steps_per_env_step=10)
        agent = DDPG(env, 
                    lr_start=1e-3,
                    lr_end=1e-5,
                    gamma=0.99,
                    buffer_size=100000,
                    batch_size=64,
                    polyak_tau=0.001)
        
        episode_rewards = []
        episode_lengths = []
        average_rewards = deque(maxlen=100)  # Moving average of last 100 episodes
                
        # Initialize environment
        state, _ = env.reset()
        
        # Warmup phase: collect random experiences
        self.get_logger().info("Starting warmup phase...")
        for step in range(warmup_steps):
            # Random action during warmup
            action = env.action_space.sample()
            # self.get_logger().info(str("warmup action: ") + str(action))
            
            # Take action
            next_state, reward, terminated, truncated, info = env.step(action)
            reward = self.reward_function(state)
            
            done = terminated or truncated
            
            # Store transition
            # Convert action to tensor if needed for buffer
            if isinstance(action, np.ndarray):
                action_to_store = action
            elif isinstance(action, torch.Tensor):
                action_to_store = action.cpu().numpy()
            else:
                action_to_store = np.array([action])
                
            agent.buffer.push(state, action_to_store, reward, next_state, done)
            
            if done:
                state, info = env.reset()
            else:
                state = next_state

        self.get_logger().info("Starting training phase...")
        for episode in range(num_episodes):
            state, info = env.reset()
            episode_reward = 0
            episode_length = 0
            
            # Gradually reduce noise for exploration
            agent.noise_std = max(0.01, 1.0 * (0.995 ** episode))
            
            for step in range(max_steps):
                # Select action
                action = agent.select_action(state)
                
                # Convert action to appropriate format for environment
                if isinstance(action, torch.Tensor):
                    action = action.cpu().numpy().squeeze()
                
                # Clip action to environment bounds if needed
                if hasattr(env.action_space, 'low') and hasattr(env.action_space, 'high'):
                    action = np.clip(action, env.action_space.low, env.action_space.high)
                
                # Take action
                next_state, reward, terminated, truncated, info = env.step(action)
                reward = self.reward_function(state)
                
                done = terminated or truncated
                
                # Store transition
                agent.buffer.push(state, action, reward, next_state, done)
                
                # Update state
                state = next_state
                episode_reward += reward
                episode_length += 1
                
                # Optimize agent
                agent.optimize()
                
                # Update target networks periodically
                if agent.steps % update_every == 0:
                    agent.update_target_network()
                    agent.actor_update_target_network()
                
                if done:
                    break
            
            # Record statistics
            episode_rewards.append(episode_reward)
            episode_lengths.append(episode_length)
            average_rewards.append(episode_reward)
            
            agent.steps += 1
            
            # Logging
            if episode % log_interval == 0:
                avg_reward = np.mean(average_rewards)
                self.get_logger().info(f"Episode {episode+1}/{num_episodes}")
                self.get_logger().info(f"  Reward: {episode_reward:.2f}")
                self.get_logger().info(f"  Length: {episode_length}")
                self.get_logger().info(f"  Avg Reward (last 100): {avg_reward:.2f}")
                self.get_logger().info(f"  Noise std: {agent.noise_std:.4f}")
                self.get_logger().info(f"  Buffer size: {len(agent.buffer)}")
                self.get_logger().info(f"  Steps total: {agent.steps}")
                            
        
        self.get_logger().info("Training completed!")
        return episode_rewards, episode_lengths
    
    def reward_function(self, state):
        path = self.path
        
        current_x, current_y = state[0], state[1]
        min_dist = float('inf')
        nearest_idx = 0
        
        for i, point in enumerate(path):
            dist = math.hypot(current_x - point.pose.position.x, 
                            current_y - point.pose.position.y)
            if dist < min_dist:
                min_dist = dist
                nearest_idx = i
        
        # 2. Reward based on progress
        progress = nearest_idx / max(1, len(path) - 1)  # 0 to 1
        
        # 3. Penalty based on distance to nearest waypoint
        distance_penalty = math.exp(min_dist) - 1  # Exponential penalty
        
        # 4. Combine: reward progress, penalize distance
        reward = progress - 0.5 * distance_penalty
        
        # 5. Bonus for reaching near the end
        end_dist = math.hypot(current_x - path[-1].pose.position.x,
                            current_y - path[-1].pose.position.y)
        if end_dist < 0.5:
            reward += 10.0
        
        return reward

    def publish_stop(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = RLPathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
