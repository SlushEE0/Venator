target_time = 70
turn_num = 6
straight_num = 34.5
turn_time = 2.6 # time for one turn (in seconds)
straight_time = 0.93  # time for one straight at 50% speed (in seconds)
total_turn_time = turn_time * turn_num
remaining_time = target_time - total_turn_time
time_per_straight = remaining_time / straight_num
global average_speed
average_speed=0.5*(straight_time / time_per_straight)
print(average_speed)