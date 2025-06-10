import streamlit as st
import folium
from streamlit_folium import st_folium
import osmnx as ox
import networkx as nx
import os
from shapely.geometry import LineString
def calculate_distance(pa,pb): # Calculate distance between two points
    from math import radians, sin, cos, sqrt, atan2, acos
    lat1, lon1 = pa
    lat2, lon2 = pb
    lat1, lon1 = radians(lat1), radians(lon1)
    lat2, lon2 = radians(lat2), radians(lon2)
    
    # Tính toán giá trị cho acos
    arg = sin(lat1)*sin(lat2) + cos(lat1)*cos(lat2)*cos(lon2-lon1)
    
    # Giới hạn giá trị trong khoảng [-1, 1]
    arg = max(min(arg, 1.0), -1.0)
    
    return acos(arg)*6371000

# Configure OSMnx
ox.settings.log_console = True
ox.settings.use_cache = True
ox.settings.timeout = 300
CENTER_LAT = 21.03333  # Tọa độ trung tâm phường Điện Biên
CENTER_LON = 105.83333
ZOOM_START = 16
DEFAULT_LOCATION = (CENTER_LAT, CENTER_LON) #location of Phuong Mai, Dong Da, Hanoi, Vietnam
DEFAULT_ZOOM = ZOOM_START
# Define the path to the saved graph file
GRAPHML_FILE = "dienbien.graphml"
from collections import defaultdict
# Function to load or create the graph
@st.cache_resource
def load_graph():
    # Tải lại đồ thị với network_type='drive' để chỉ bao gồm đường cho xe chạy
    st.info("Đang tải và xử lý lại dữ liệu bản đồ (chỉ bao gồm đường cho xe chạy)...")
    place_name = "Phường Điện Biên, Quận Ba Đình, Hà Nội, Việt Nam"
    
    # Lấy mạng lưới đường cho xe chạy (dạng có hướng để xử lý đường 1 chiều)
    G_drive = ox.graph_from_place(place_name, network_type='drive', simplify=True)
    
    # Chuyển thành đồ thị vô hướng để phù hợp với thuật toán A* hiện tại
    G = G_drive.to_undirected()
    
    # Lưu lại đồ thị mới đã được lọc
    ox.save_graphml(G, filepath=GRAPHML_FILE)
    
    # Xây dựng adjacency list cho A*
    graph = defaultdict(list)
    for u, v, data in G.edges(data=True):
        length = data.get('length', 0)
        graph[u].append((v, length))
        graph[v].append((u, length))

    nodes = dict()
    for node, data in G.nodes(data=True):
        nodes[node] = (data['y'], data['x'])
        
    st.success("Đã tải xong bản đồ mới!")
    return G, graph, nodes
G, graph, nodes = load_graph()



if 'points' not in st.session_state:
    st.session_state['points'] = []
if 'forbidden_points' not in st.session_state:
    st.session_state['forbidden_points'] = []
if 'forbidden_edges' not in st.session_state:
    st.session_state['forbidden_edges'] = set()
if 'zoom' not in st.session_state:
    st.session_state['zoom'] = DEFAULT_ZOOM
if 'center' not in st.session_state:
    st.session_state['center'] = DEFAULT_LOCATION
m = folium.Map(
    location=st.session_state['center'],
    zoom_start=st.session_state['zoom'],
    tiles='OpenStreetMap',
)


from heapq import *

def Astar_algorithm(pointA, pointB):
    global graph, nodes
    queue = []
    heappush(queue, (0, pointA))
    father = defaultdict(int)
    father[pointA] = -432
    res = defaultdict(int)
    res[pointA] = 0
    visited = set()
    path_info = defaultdict(list)
    edge_visited = set()  # Theo dõi các cạnh đã đi
    forbidden_edges = st.session_state.get('forbidden_edges', set())
    
    while queue:
        current_cost, current_node = heappop(queue)
        
        if current_node in visited:
            continue
            
        visited.add(current_node)
        
        if current_node == pointB:
            break
            
        for neighbor, cost in graph[current_node]:
            # Kiểm tra cạnh đã đi
            edge_tuple_ids = tuple(sorted([current_node, neighbor]))
            if edge_tuple_ids in edge_visited:
                continue

            # Kiểm tra cạnh có giao với đường cấm không
            is_forbidden = False
            current_edge_line = LineString([nodes[current_node], nodes[neighbor]])
            for forbidden_coord_pair in forbidden_edges:
                forbidden_line = LineString(forbidden_coord_pair)
                if current_edge_line.crosses(forbidden_line) or current_edge_line.touches(forbidden_line) or current_edge_line.equals(forbidden_line):
                    is_forbidden = True
                    break
            if is_forbidden:
                continue
                
            if neighbor in visited:
                continue
                
            g = current_cost + cost
            h = calculate_distance(nodes[neighbor], nodes[pointB])
            f = g + h
            
            if neighbor not in father or f < res[neighbor]:
                heappush(queue, (f, neighbor))
                father[neighbor] = current_node
                res[neighbor] = f
                path_info[neighbor] = path_info[current_node] + [(current_node, neighbor, cost)]
                edge_visited.add(edge_tuple_ids)
    distance = res[pointB]
    path = []
    total_distance = 0
    while father[pointB] != -432:
        path.append(pointB)
        total_distance += res[pointB] - res[father[pointB]]
        pointB = father[pointB]
    path.append(pointA)
    path.reverse()
    
    return {
        'distance': total_distance,
        'path': path,
        'visited_nodes': len(visited),
        'path_info': path_info[pointB] if pointB in path_info else []
    }

for idx, node_id in enumerate(st.session_state['points']):
    lat, lon = nodes[node_id]
    folium.Marker(location=(lat, lon), tooltip=f"Point {idx+1}", icon=folium.Icon("blue")).add_to(m)

# Hiển thị các điểm đang chọn để tạo đường cấm
for idx, point_coord in enumerate(st.session_state.get('forbidden_points', [])):
    folium.Marker(location=point_coord, tooltip=f"Điểm cấm {idx+1}", icon=folium.Icon(color='red')).add_to(m)


if len(st.session_state['points']) == 2:
    orig = st.session_state['points'][0]
    dest = st.session_state['points'][1]

    route = Astar_algorithm(orig, dest)['path']
    

    # Extract the edge geometries for the route
    edge_geometries = []
    for u, v in zip(route[:-1], route[1:]):
        data = G.get_edge_data(u, v)
        if data:
            # If multiple edges exist between two nodes, choose the first one
            edge_data = data[list(data.keys())[0]]
            if 'geometry' in edge_data:
                # If geometry is available, use it
                edge_geometries.append(edge_data['geometry'])
            else:
                # If no geometry, create a straight line between nodes
                point_u = (G.nodes[u]['x'], G.nodes[u]['y'])
                point_v = (G.nodes[v]['x'], G.nodes[v]['y'])
                edge_geometries.append(LineString([point_u, point_v]))

    # Plot the route on the map using the edge geometries
    for geom in edge_geometries:
        coords = [(lat, lon) for lon, lat in geom.coords]
        folium.PolyLine(coords, color='blue',tooltip="Too much smoothing?", weight=3).add_to(m)

# Thêm chế độ chọn đường cấm
mode = st.radio("Chọn chế độ thao tác:", ["Tìm đường", "Tạo đường cấm"], horizontal=True)

# Hiển thị các đường cấm bằng màu đỏ
for edge_coords in st.session_state.get('forbidden_edges', set()):
    folium.PolyLine(locations=list(edge_coords), color='red', weight=5, tooltip="Đường cấm").add_to(m)

# Nếu đang chọn chế độ tạo đường cấm và đã chọn 2 điểm, vẽ preview
if mode == "Tạo đường cấm" and len(st.session_state['forbidden_points']) == 2:
    pt1, pt2 = st.session_state['forbidden_points']
    folium.PolyLine([pt1, pt2], color='red', dash_array='5, 5', weight=5, tooltip="Đường cấm (preview)").add_to(m)

st.title("Điện Biên District Map")

output = st_folium(m, width=1000, height=500,returned_objects=['last_clicked', 'zoom', 'center'])
if output and output['last_clicked']:
    st.session_state['zoom'] = output['zoom']
    st.session_state['center'] = output['center']['lat'], output['center']['lng'] 
    clicked_point = (output['last_clicked']['lat'], output['last_clicked']['lng'])

    if mode == "Tạo đường cấm":
        st.session_state['forbidden_points'].append(clicked_point)
        if len(st.session_state['forbidden_points']) >= 2:
            pt1, pt2 = st.session_state['forbidden_points']
            edge_tuple = (pt1, pt2)
            if edge_tuple in st.session_state['forbidden_edges'] or (pt2, pt1) in st.session_state['forbidden_edges']:
                st.warning("Đường cấm này đã tồn tại!")
            else:
                st.session_state['forbidden_edges'].add(edge_tuple)
                st.success("Đã tạo đường cấm mới!")
            # Reset lại danh sách điểm để tạo đường cấm mới
            st.session_state['forbidden_points'] = []
    else:
        # Chế độ tìm đường: vẫn tạo node mới như cũ
        import time
        new_node_id = int(time.time() * 1e6)
        lat, lon = clicked_point
        G.add_node(new_node_id, y=lat, x=lon)
        nodes[new_node_id] = (lat, lon)
        min_dist = float('inf')
        nearest_node = None
        for node_id, (y, x) in nodes.items():
            if node_id == new_node_id:
                continue
            dist = calculate_distance((lat, lon), (y, x))
            if dist < min_dist:
                min_dist = dist
                nearest_node = node_id
        if nearest_node is not None:
            from shapely.geometry import LineString
            new_line = LineString([nodes[nearest_node], (lat, lon)])
            intersects_forbidden = False
            for edge in st.session_state.get('forbidden_edges', set()):
                forbidden_line = LineString(edge)
                if new_line.crosses(forbidden_line) or new_line.touches(forbidden_line):
                    intersects_forbidden = True
                    break
            if intersects_forbidden:
                st.warning("Không thể nối tới node gần nhất vì cạnh này giao với đường cấm!")
            else:
                G.add_edge(new_node_id, nearest_node, length=min_dist, oneway=False)
                G.add_edge(nearest_node, new_node_id, length=min_dist, oneway=False)
                graph[new_node_id].append((nearest_node, min_dist))
                graph[nearest_node].append((new_node_id, min_dist))
        st.session_state['points'].append(new_node_id)

    # Rerun để cập nhật giao diện ngay lập tức sau mỗi lần click
    st.rerun()

if st.button("Reset Points"):
    st.session_state['points'] = []
    st.session_state['zoom'] = DEFAULT_ZOOM
    st.session_state['center'] = DEFAULT_LOCATION
    st.session_state['forbidden_points'] = []
    st.rerun()

if st.button("Xóa tất cả đường cấm"):
    st.session_state['forbidden_edges'] = set()
    # Cần load lại graph từ file để khôi phục các cạnh đã xóa
    G, graph, nodes = load_graph()
    st.success("Đã xóa tất cả đường cấm và khôi phục đồ thị!")
    st.rerun()

