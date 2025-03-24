total_distance = 0
turns = 0

def f(distance):
    global total_distance
    total_distance += distance

def b(distance):
    global total_distance
    total_distance += distance

def l():
    global turns
    turns += 1

def r():
    global turns
    turns += 1

f(8.5)
r()
f(4)
r()
f(8)
r()
f(2)
r()
f(6)
r()
f(2)
r()
f(4)
print(f"Total distance of straight movements: {total_distance}")
print(f"Number of turns: {turns}")
