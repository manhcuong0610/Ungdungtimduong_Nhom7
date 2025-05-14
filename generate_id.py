import random
with open('random_id.txt', 'w') as f:
    for _ in range(1000000):  # Tạo 1 triệu ID ngẫu nhiên
        f.write(str(random.randint(10000000000, 99999999999)) + '\n')
