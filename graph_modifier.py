import osmnx as ox  
from shapely.geometry import LineString
import random

# Đọc random_id từ file
try:
    with open('random_id.txt', 'r') as f:
        random_id = [int(x.strip()) for x in f.readlines()]
except FileNotFoundError:
    print("File random_id.txt không tồn tại. Tạo file mới...")
    random_id = [random.randint(10000000000, 99999999999) for _ in range(1000000)]
    with open('random_id.txt', 'w') as f:
        for id in random_id:
            f.write(str(id) + '\n')

# Tạo đồ thị từ OpenStreetMap
G = ox.graph_from_place("Điện Biên, Ba Đình, Hà Nội, Vietnam")
max_length = 6
nodes = dict()

cnt = 0
for node in G.nodes(data=True):
    nodes[node[0]] = node[1]

def get_coordinates(node_id):
    node = G.nodes[node_id]
    return node['y'], node['x']

def calculate_distance(pa, pb):
    from math import radians, sin, cos, sqrt, atan2, acos
    lat1, lon1 = pa
    lat2, lon2 = pb
    lat1, lon1 = radians(lat1), radians(lon1)
    lat2, lon2 = radians(lat2), radians(lon2)
    return acos(sin(lat1)*sin(lat2)+cos(lat1)*cos(lat2)*cos(lon2-lon1))*6371000

def add_edge(node1, node2):
    global G
    # Kiểm tra xem cả hai node có tồn tại không
    if node1 not in G.nodes or node2 not in G.nodes:
        print(f"Không thể thêm cạnh: Node {node1} hoặc {node2} không tồn tại trong đồ thị")
        return
    
    # Tính khoảng cách giữa hai node
    dist = calculate_distance(get_coordinates(node1), get_coordinates(node2))
    
    # Thêm cạnh hai chiều
    G.add_edge(node2, node1, length=dist, oneway=False)
    G.add_edge(node1, node2, length=dist, oneway=False)

def process_edge_linestring(edge):
    global G, cnt
    try:
        pa = nodes[edge[0]]['y'], nodes[edge[0]]['x']
        pb = nodes[edge[1]]['y'], nodes[edge[1]]['x']
        data = edge[2]
        one_way = data.get('oneway', False)
        
        if 'geometry' in data:
            G.remove_edge(edge[0], edge[1])
            l = [chunk for chunk in data['geometry'].coords]
            idx = [edge[0]]
            
            for i in range(1, len(l)-1):
                if cnt >= len(random_id):
                    print("Hết ID ngẫu nhiên!")
                    return
                    
                G.add_node(random_id[cnt], y=l[i][1], x=l[i][0])
                nodes[random_id[cnt]] = {'y':l[i][1], 'x':l[i][0]}
                idx.append(random_id[cnt])
                
                dist = calculate_distance(l[i], l[i-1])
                G.add_edge(random_id[cnt], idx[-2], length=dist, oneway=one_way)
                if not one_way:
                    G.add_edge(idx[-2], random_id[cnt], length=dist, oneway=False)
                cnt += 1
                
            dist = calculate_distance(l[-1], l[-2])
            G.add_edge(edge[1], idx[-1], length=dist, oneway=one_way)
            if not one_way:
                G.add_edge(idx[-1], edge[1], length=dist, oneway=False)
    except Exception as e:
        print(f"Lỗi khi xử lý cạnh: {e}")

def process_long_edge(edge):
    global G, cnt
    try:
        pa = nodes[edge[0]]['y'], nodes[edge[0]]['x']
        pb = nodes[edge[1]]['y'], nodes[edge[1]]['x']
        data = edge[2]
        one_way = data.get('oneway', False)
        length = calculate_distance(pa, pb)
        
        if length <= 2*max_length:
            return
            
        try:
            G.remove_edge(edge[0], edge[1])
            G.remove_edge(edge[1], edge[0])
        except:
            return
            
        increment = max_length/length
        dy = (pb[0]-pa[0])*increment
        dx = (pb[1]-pa[1])*increment
        newnodes_id = [edge[0]]
        newnodes_coor = [pa]
        
        for i in range(1, int(length/max_length)):
            if cnt >= len(random_id):
                print("Hết ID ngẫu nhiên!")
                return
                
            coor = (pa[0]+dy*i, pa[1]+dx*i)
            G.add_node(random_id[cnt], y=coor[0], x=coor[1])
            nodes[random_id[cnt]] = {'y':coor[0], 'x':coor[1]}
            
            dist = calculate_distance(newnodes_coor[-1], coor)
            G.add_edge(random_id[cnt], newnodes_id[-1], length=dist, oneway=one_way)
            if not one_way:
                G.add_edge(newnodes_id[-1], random_id[cnt], length=dist, oneway=False)
                
            newnodes_coor.append(coor)
            newnodes_id.append(random_id[cnt])
            cnt += 1
            
        dist = calculate_distance(newnodes_coor[-1], pb)
        G.add_edge(edge[1], newnodes_id[-1], length=dist, oneway=one_way)
        if not one_way:
            G.add_edge(newnodes_id[-1], edge[1], length=dist, oneway=False)
            
    except Exception as e:
        print(f"Lỗi khi xử lý cạnh dài: {e}")

# Xử lý tất cả các cạnh
from copy import deepcopy
for edge in deepcopy(G.edges(data=True)):
    process_edge_linestring(edge)

for edge in deepcopy(G.edges(data=True)):
    process_long_edge(edge)

# Lưu đồ thị
ox.save_graphml(G, "dienbien.graphml")
print("Đã lưu đồ thị thành công!") 