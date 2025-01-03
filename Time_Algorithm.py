def calculate_motor_speed(turn_num, straight_num, target_time):
    turn_time = 3  # Time for a 90-degree turn (in seconds)
    total_turn_time = turn_time * turn_num
    remaining_time = target_time - total_turn_time
    straight_time = 0.53  # Modify this based on your specific requirements
    time_per_straight = remaining_time / straight_num
    motor_speed = 50 * (straight_time / time_per_straight)

    # Ensure motor speed does not exceed 100%
    if motor_speed > 100:
        motor_speed = 100
        
    return motor_speed

# Example usage
target_time = 55
turn_num = 8
straight_num = 70
motor_speed = calculate_motor_speed(turn_num, straight_num, target_time)

# Print the result
print(f"Calculated Motor Speed for straight segments: {motor_speed}%")
