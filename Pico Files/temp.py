turn_time = 2.6 # time for one turn (in seconds)
straight_time = 0.93 
target_time = 64
turn_num = 6
straight_num = 35.5
left=30
right=30.6
dist=25
total_turn_time = turn_time * turn_num
remaining_time = target_time - total_turn_time
time_per_straight = remaining_time / straight_num
average_speed=0.5*(straight_time / time_per_straight)
print (average_speed)