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

# Example usage
f(1.3)
r()
f(6)
b(2)
l()
f(4)
r()
f(2)
b(2)
l()
f(4)
r()
f(2)
r()
f(2)
b(2)
r()
f(4)
l()
f(6)
r()
f(2)
b(2)
r()
f(6)
l()
f(2)
l()
f(2)

print(f"Total distance of straight movements: {total_distance}")
print(f"Number of turns: {turns}")
