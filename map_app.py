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
    G = ox.load_graphml(GRAPHML_FILE)
    
    graph = defaultdict(list)
    for edge in G.edges(data=True):
        graph[edge[0]].append((edge[1], edge[2]['length']))
        graph[edge[1]].append((edge[0], edge[2]['length']))
    nodes = dict()
    for node in G.nodes(data=True):
        nodes[node[0]] = (node[1]['y'],node[1]['x'])
    return G, graph, nodes
G, graph, nodes = load_graph()



if 'points' not in st.session_state:
    st.session_state['points'] = []
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
    
    while queue:
        current_cost, current_node = heappop(queue)
        
        if current_node in visited:
            continue
            
        visited.add(current_node)
        
        if current_node == pointB:
            break
            
        for neighbor, cost in graph[current_node]:
            # Kiểm tra cạnh đã đi chưa
            edge = tuple(sorted([current_node, neighbor]))
            if edge in edge_visited:
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
                edge_visited.add(edge)
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

for idx, point in enumerate(st.session_state['points']):
    folium.Marker(location=point, tooltip=f"Point {idx+1}",icon=folium.Icon("blue")).add_to(m)


if len(st.session_state['points']) == 2:
    orig = st.session_state['points'][0]
    dest = st.session_state['points'][1]

    # Find the nearest nodes in the graph
    orig_node = ox.nearest_nodes(G,orig[1], orig[0])
    dest_node = ox.nearest_nodes(G,dest[1], dest[0])
    
    route = Astar_algorithm(orig_node, dest_node)['path']
    

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

st.title("Điện Biên District Map")

output = st_folium(m, width=1000, height=500,returned_objects=['last_clicked', 'zoom', 'center'])
if output and output['last_clicked']:
    st.session_state['zoom'] = output['zoom']
    st.session_state['center'] = output['center']['lat'], output['center']['lng'] 
    clicked_point = (output['last_clicked']['lat'], output['last_clicked']['lng'])
    st.session_state['points'].append(clicked_point)
    st.rerun()
if st.button("Reset Points"):
    st.session_state['points'] = []
    st.session_state['zoom'] = DEFAULT_ZOOM
    st.session_state['center'] = DEFAULT_LOCATION
    st.rerun()

