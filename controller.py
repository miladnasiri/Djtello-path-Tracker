import numpy as np
import torch
import torch.nn as nn

class AdvancedController:
    def __init__(self):
        # PID gains
        self.Kp = np.array([0.5, 0.5, 0.5])  # Position gains
        self.Ki = np.array([0.1, 0.1, 0.1])  # Integral gains
        self.Kd = np.array([0.2, 0.2, 0.2])  # Derivative gains
        
        # Initialize error integrals
        self.error_integral = np.zeros(3)
        self.prev_error = np.zeros(3)
        
        # Initialize neural network controller
        self.nn_controller = self._create_neural_controller()
        
        # MPC parameters
        self.horizon = 20
        self.dt = 0.1
        
    def compute_control(self, current_state, target_state):
        """Compute control commands using hybrid control"""
        # Compute PID control
        pid_command = self._compute_pid(current_state, target_state)
        
        # Compute neural network compensation
        nn_command = self._compute_nn_control(current_state, target_state)
        
        # Combine commands
        total_command = pid_command + 0.3 * nn_command  # Weighted combination
        
        # Apply safety limits
        return self._apply_limits(total_command)
        
    def _compute_pid(self, current_state, target_state):
        """Compute PID control commands"""
        # Calculate position error
        error = target_state['position'] - current_state['position']
        
        # Update integral
        self.error_integral += error * self.dt
        
        # Calculate derivative
        error_derivative = (error - self.prev_error) / self.dt
        self.prev_error = error
        
        # PID control law
        command = (self.Kp * error + 
                  self.Ki * self.error_integral + 
                  self.Kd * error_derivative)
                  
        return command
        
    def _create_neural_controller(self):
        """Create neural network for adaptive control"""
        model = nn.Sequential(
            nn.Linear(12, 64),    # Input: state vector
            nn.ReLU(),
            nn.Linear(64, 32),
            nn.ReLU(),
            nn.Linear(32, 3)      # Output: control commands
        )
        return model
        
    def _compute_nn_control(self, current_state, target_state):
        """Compute neural network control contribution"""
        # Prepare input state vector
        state_vector = np.concatenate([
            current_state['position'],
            current_state['velocity'],
            current_state['attitude'],
            current_state['angular_velocity']
        ])
        
        # Convert to tensor
        state_tensor = torch.FloatTensor(state_vector)
        
        # Forward pass through network
        with torch.no_grad():
            control = self.nn_controller(state_tensor)
            
        return control.numpy()
        
    def _apply_limits(self, command):
        """Apply safety limits to control commands"""
        # Maximum allowed control values
        max_control = np.array([0.5, 0.5, 0.5])
        
        # Clip commands
        return np.clip(command, -max_control, max_control)
        
    def train_nn_controller(self, state_data, control_data):
        """Train neural network controller using collected data"""
        # Convert data to tensors
        states = torch.FloatTensor(state_data)
        controls = torch.FloatTensor(control_data)
        
        # Define optimizer
        optimizer = torch.optim.Adam(self.nn_controller.parameters(), lr=0.001)
        criterion = nn.MSELoss()
        
        # Training loop
        for epoch in range(100):
            optimizer.zero_grad()
            
            # Forward pass
            predictions = self.nn_controller(states)
            loss = criterion(predictions, controls)
            
            # Backward pass
            loss.backward()
            optimizer.step()
            
    def update_gains(self, new_kp=None, new_ki=None, new_kd=None):
        """Update PID gains"""
        if new_kp is not None:
            self.Kp = np.array(new_kp)
        if new_ki is not None:
            self.Ki = np.array(new_ki)
        if new_kd is not None:
            self.Kd = np.array(new_kd)
            
    def reset(self):
        """Reset controller state"""
        self.error_integral = np.zeros(3)
        self.prev_error = np.zeros(3)
