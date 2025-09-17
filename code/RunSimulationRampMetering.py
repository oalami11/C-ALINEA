# ####### COORDINATED ALINEA - RAMP METERING COORDINATION FOR HIGHWAY NETWORKS
# #######
# #######     AUTHORS:      Kevin Riehl <kriehl@ethz.ch>, Omar Alami Badissi <oalami@ethz.ch>
# #######     YEAR :        2025
# #######     ORGANIZATION: Traffic Engineering Group (SVT), 
# #######                   Institute for Transportation Planning and Systems,
# #######                   ETH Zürich
# #############################################################################
"""
This code implements multiple ramp metering algorithms for highway networks:
- NO_CONTROL: Baseline without ramp metering
- ALINEA: Standard ALINEA algorithm
- C_ALINEA: Coordinated ALINEA with neighbor consideration
- METALINE: Network-wide density control algorithm

Features include comprehensive data analysis, heatmap generation, and OD analysis.

Usage Examples:
python codecoor.py --sumo-path ./sumo-1.19.0/bin/sumo-gui.exe --controller NO_CONTROL
python codecoor.py --sumo-path ./sumo-1.19.0/bin/sumo-gui.exe --controller ALINEA
python codecoor.py --sumo-path ./sumo-1.19.0/bin/sumo-gui.exe --controller C_ALINEA --neighbors 2 --method 1
python codecoor.py --sumo-path ./sumo-1.19.0/bin/sumo-gui.exe --controller METALINE
"""

# #############################################################################
# ## IMPORTS
# #############################################################################
from collections import deque
import traci
import traci.constants as tc
import math
import os
import random
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import xml.etree.ElementTree as ET
import smtplib
from datetime import datetime
import sys

# #############################################################################
# ## SIMULATION PARAMETERS
# #############################################################################

# Core simulation parameters
K_VALUES = [80]                    # Controller gain values for testing
OCC_TARGET_VALUES = [5]            # Target occupancy percentages
SEEDS = [56618]                    # Random seeds for reproducibility

# Simulation timing (seconds)
SIM_DURATION = 82800               # Total simulation duration (23 hours)
WARM_UP = 3000                     # Warm-up period (50 minutes)
ANALYSIS_START = 37000             # Analysis period start
ANALYSIS_END = 67000               # Analysis period end

# Controller parameters
Q_MIN, Q_MAX = 0, 1800            # Flow rate limits (veh/h)
avg_acc = 2                        # Average acceleration factor
C = 60                             # Traffic light cycle duration (seconds)
K_c = 0.5                          # Coordination gain for C_ALINEA

# #############################################################################
# ## NETWORK CONFIGURATION - REPLACE WITH YOUR NETWORK DATA
# #############################################################################

# Define ramp sensor configuration
# Each ramp has: [pre_sensor, main_sensor, downstream_sensors...]
RAMP_SENSOR = {
    "TLS_ID_1": ["e2_RAMP_ID_1_pre", "e2_RAMP_ID_1", "e2_downstream_1_0", "e2_downstream_1_1", "e2_downstream_1_2"],
    "TLS_ID_2": ["e2_RAMP_ID_2_pre", "e2_RAMP_ID_2", "e2_downstream_2_0", "e2_downstream_2_1", "e2_downstream_2_2"],
    "TLS_ID_3": ["e2_RAMP_ID_3_pre", "e2_RAMP_ID_3", "e2_downstream_3_0", "e2_downstream_3_1", "e2_downstream_3_2"],
}

# Define ramp processing order (upstream to downstream)
RAMP_ORDER = [
    "TLS_ID_1", "TLS_ID_2", "TLS_ID_3"
]

# Define off-ramps for Origin-Destination analysis
OFF_RAMPS = [
    "OFF_RAMP_1", "OFF_RAMP_2", "OFF_RAMP_3"
]

# Highway edges for network analysis (ordered from start to end)
HIGHWAY_EDGES_ORDERED = [
    'edge_1', 'edge_2', 'edge_3', 'edge_4', 'edge_5'
]

# Edge boundaries in kilometers for spatial analysis
EDGE_BOUNDS_KM = {
    'edge_1': (0.00, 1.00),
    'edge_2': (1.00, 2.00),
    'edge_3': (2.00, 3.00),
    'edge_4': (3.00, 4.00),
    'edge_5': (4.00, 5.00),
}

# Ramp positions along highway (km from start)
RAMP_POSITIONS_KM = {
    "TLS_ID_1": 0.5,
    "TLS_ID_2": 1.5,
    "TLS_ID_3": 2.5,
}

# Inter-ramp distances for coordination (meters)
DISTANCES = {
    "TLS_ID_1": {"after1": 1000, "after2": 2000, "after3": 3000, "before1": 3000, "before2": 4000, "before3": 5000},
    "TLS_ID_2": {"after1": 1000, "after2": 2000, "after3": 3000, "before1": 1000, "before2": 4000, "before3": 5000},
    "TLS_ID_3": {"after1": 2000, "after2": 3000, "after3": 4000, "before1": 1000, "before2": 2000, "before3": 3000},
}

# #############################################################################
# ## METALINE ALGORITHM PARAMETERS
# #############################################################################

# METALINE target densities for each highway edge (vehicles/km)
RHO_DESIRED = np.array([25.0, 30.0, 35.0, 30.0, 25.0])

# METALINE target flow rates for each ramp (vehicles/hour)
R_DESIRED = np.array([1200, 1300, 1400])

def build_metaline_gain_matrix():
    """Build spatial gain matrix for METALINE algorithm"""
    n_ramps = len(RAMP_ORDER)
    n_edges = len(HIGHWAY_EDGES_ORDERED)
    K = np.zeros((n_ramps, n_edges))
    
    for i, ramp in enumerate(RAMP_ORDER):
        ramp_km = RAMP_POSITIONS_KM[ramp]
        
        for j, edge in enumerate(HIGHWAY_EDGES_ORDERED):
            edge_start, edge_end = EDGE_BOUNDS_KM[edge]
            edge_center = (edge_start + edge_end) / 2
            distance = abs(ramp_km - edge_center)
            
            # Distance-based gain calculation
            if distance < 0.1:
                gain = 16.5
            elif distance < 0.5:
                gain = 13.5 - 6 * distance
            elif distance < 1.0:
                gain = 6 * np.exp(-1.5 * distance)
            elif distance < 2.0:
                gain = 4 * np.exp(-0.7 * distance)
            elif distance < 3.0:
                gain = 2.5 * np.exp(-distance / 2)
            elif distance < 5.0:
                gain = 1.0 * np.exp(-distance / 3)
            else:
                gain = 0.2
            
            K[i, j] = max(0.1, gain)
        
        # Normalize if row sum is too large
        row_sum = K[i].sum()
        if row_sum > 100:
            K[i] *= 100 / row_sum
    
    return K

# #############################################################################
# ## CONTROL ALGORITHMS
# #############################################################################

def alinea(q_prev, occ, K_a, occ_targ):
    """Standard ALINEA ramp metering algorithm"""
    q = q_prev + K_a * (occ_targ - occ)
    q = max(Q_MIN, min(Q_MAX, q))
    r = min(1, (q / 3600) * avg_acc)
    r = max(0, r)
    return q, r

def C_alinea(q_prev, occ, K_a, occ_targ, flow_buffers, weights):
    """Coordinated ALINEA algorithm with neighbor flow consideration"""
    # Basic ALINEA calculation
    q = q_prev + K_a * (occ_targ - occ)
    q = max(Q_MIN, min(Q_MAX, q))
    
    # Calculate average flows
    q_avg_self = (sum(flow_buffers['self']) / len(flow_buffers['self'])) * 3600 if flow_buffers['self'] else 0
    
    # Calculate weighted neighbor average
    total_weight = 0
    weighted_flow = 0
    
    for direction in ['after1', 'before1', 'after2', 'before2', 'after3', 'before3']:
        if direction in flow_buffers and direction in weights:
            flow = (sum(flow_buffers[direction]) / len(flow_buffers[direction])) * 3600 if flow_buffers[direction] else 0
            if flow > 0:
                weighted_flow += weights[direction] * flow
                total_weight += weights[direction]
    
    q_avg_neigh = weighted_flow / max(total_weight, 1e-6)
    
    # Coordination correction
    correction = K_c * (q_avg_neigh - q_avg_self)
    q_coor = q + correction
    q_coor = max(Q_MIN, min(Q_MAX, q_coor))
    
    r = min(1, (q_coor / 3600) * avg_acc)
    r = max(0, r)
    
    return q_coor, r

def calculate_weights(distances, n_neighbors, method):
    """Calculate distance-based weights for coordinated ALINEA"""
    weights = {}
    directions = ['after1', 'before1']
    
    if n_neighbors >= 2:
        directions.extend(['after2', 'before2'])
    if n_neighbors >= 3:
        directions.extend(['after3', 'before3'])
    
    if method == "1":
        # Global normalization
        L_MAX = max(max(d[f"before{i}"], d[f"after{i}"]) 
                   for d in DISTANCES.values() 
                   for i in range(1, n_neighbors+1))
        
        for direction in directions:
            if direction in distances:
                weights[direction] = max(0.0, 1 - (distances[direction] / L_MAX))
            else:
                weights[direction] = 0.0
    else:  # method == "2"
        # Local normalization
        local_distances = []
        for direction in directions:
            if direction in distances:
                local_distances.append(distances[direction])
        
        L_MAX_local = max(local_distances) if local_distances else 1.0
        
        for direction in directions:
            if direction in distances:
                weights[direction] = max(0.0, 1 - (distances[direction] / L_MAX_local))
            else:
                weights[direction] = 0.0
    
    return weights

def light(r, tls_id):
    """Control traffic light based on metering rate"""
    green_time = min(C, math.floor(r * C + 0.5))
    
    if green_time >= C:
        traci.trafficlight.setPhase(tls_id, 0)
        traci.trafficlight.setPhaseDuration(tls_id, C)
    elif green_time <= 0:
        traci.trafficlight.setPhase(tls_id, 1)
        traci.trafficlight.setPhaseDuration(tls_id, C)
    else:
        traci.trafficlight.setPhase(tls_id, 0)
        traci.trafficlight.setPhaseDuration(tls_id, green_time)

class MetalineController:
    """METALINE network-wide density control algorithm"""
    
    def __init__(self, update_interval=60):
        self.K_matrix = build_metaline_gain_matrix()
        self.q_flows = np.copy(R_DESIRED)
        self.update_interval = update_interval
        self.last_update_time = 0
        self.density_buffer = []
        
        print("METALINE Controller initialized:")
        print(f"  - {len(RAMP_ORDER)} ramps")
        print(f"  - {len(HIGHWAY_EDGES_ORDERED)} highway edges")
        print(f"  - Update interval: {update_interval}s")
    
    def collect_edge_densities(self):
        """Collect current densities for all highway edges"""
        densities = np.zeros(len(HIGHWAY_EDGES_ORDERED))
        
        for i, edge_id in enumerate(HIGHWAY_EDGES_ORDERED):
            try:
                vehicle_count = traci.edge.getLastStepVehicleNumber(edge_id)
                edge_start, edge_end = EDGE_BOUNDS_KM[edge_id]
                length_km = edge_end - edge_start
                
                if length_km > 0:
                    densities[i] = vehicle_count / length_km
                else:
                    densities[i] = 0
            except:
                densities[i] = 0
        
        return densities
    
    def step(self, current_time):
        """Execute one control step"""
        if current_time < WARM_UP:
            # Warm-up period: no control
            for ramp_id in RAMP_ORDER:
                light(1.0, ramp_id)
            return
        
        # Collect current densities
        current_densities = self.collect_edge_densities()
        self.density_buffer.append(current_densities)
        
        # Limit buffer size
        if len(self.density_buffer) > self.update_interval:
            self.density_buffer.pop(0)
        
        # Update control at specified interval
        time_since_update = current_time - self.last_update_time
        
        if time_since_update >= self.update_interval:
            self._update_control(current_time)
            self.last_update_time = current_time
            self.density_buffer = []
        
        # Apply current control
        self._apply_control()
    
    def _update_control(self, current_time):
        """Update control flows based on network density measurements"""
        if len(self.density_buffer) > 0:
            avg_densities = np.mean(self.density_buffer, axis=0)
        else:
            avg_densities = self.collect_edge_densities()
        
        # METALINE control law
        density_error = avg_densities - RHO_DESIRED
        correction = self.K_matrix.dot(density_error)
        
        # Update flow rates
        self.q_flows = R_DESIRED - correction
        self.q_flows = np.clip(self.q_flows, Q_MIN, Q_MAX)
        
        # Debug output every 5 minutes
        if current_time % 300 == 0:
            print(f"[{current_time//60:4} min] METALINE: avg_density={avg_densities.mean():.1f}, target={RHO_DESIRED.mean():.1f}")
    
    def _apply_control(self):
        """Apply current control flows to all ramps"""
        for i, ramp_id in enumerate(RAMP_ORDER):
            flow_rate = self.q_flows[i]
            metering_rate = min(1.0, (flow_rate / 3600) * avg_acc)
            metering_rate = max(0.0, metering_rate)
            light(metering_rate, ramp_id)

# #############################################################################
# ## DATA ANALYSIS FUNCTIONS
# #############################################################################

def identify_origin_destination(depart_lane, arrival_lane):
    """Identify origin and destination ramps from lane names"""
    origin_ramp = None
    destination_ramp = None
    
    # Example logic - adapt to your network
    if depart_lane and 'ramp' in depart_lane.lower():
        for ramp in RAMP_ORDER:
            if ramp.lower() in depart_lane.lower():
                origin_ramp = ramp
                break
    
    if arrival_lane and 'off' in arrival_lane.lower():
        for off_ramp in OFF_RAMPS:
            if off_ramp.lower() in arrival_lane.lower():
                destination_ramp = off_ramp
                break
    
    return origin_ramp, destination_ramp

def calculate_od_distances():
    """Calculate distances between origin-destination pairs"""
    od_distances = {}
    
    for origin in RAMP_ORDER:
        od_distances[origin] = {}
        origin_km = RAMP_POSITIONS_KM[origin]
        
        for destination in OFF_RAMPS:
            # Simple distance calculation - adapt as needed
            dest_km = origin_km + 1.0  # Example assumption
            distance = abs(dest_km - origin_km)
            od_distances[origin][destination] = max(0.5, distance)
    
    return od_distances

def calculate_od_delay_summary(all_seeds_data):
    """Calculate OD delay statistics across all seeds"""
    if not all_seeds_data['general_metrics']:
        return {}
    
    od_distances = calculate_od_distances()
    od_summary = {}
    
    for origin in RAMP_ORDER:
        od_summary[origin] = {}
        for destination in OFF_RAMPS:
            od_summary[origin][destination] = {
                'avg_delays': [], 'max_delays': [], 'vehicle_counts': [], 'avg_travel_times': []
            }
    
    # Collect data from all seeds
    for seed_data in all_seeds_data['general_metrics']:
        if 'od_delay_metrics' not in seed_data:
            continue
        
        od_metrics = seed_data['od_delay_metrics']
        for origin in RAMP_ORDER:
            for destination in OFF_RAMPS:
                if origin in od_metrics and destination in od_metrics[origin]:
                    metrics = od_metrics[origin][destination]
                    od_summary[origin][destination]['avg_delays'].append(metrics['avg_delay'])
                    od_summary[origin][destination]['max_delays'].append(metrics['max_delay'])
                    od_summary[origin][destination]['vehicle_counts'].append(metrics['vehicle_count'])
                    od_summary[origin][destination]['avg_travel_times'].append(metrics['avg_travel_time'])
    
    # Calculate final statistics
    od_final_summary = {}
    for origin in RAMP_ORDER:
        od_final_summary[origin] = {}
        for destination in OFF_RAMPS:
            data = od_summary[origin][destination]
            distance_km = od_distances[origin][destination]
            
            total_vehicles = sum(data['vehicle_counts'])
            if total_vehicles > 0:
                mean_avg_delay = np.mean([d for d in data['avg_delays'] if d > 0])
                mean_max_delay = np.mean([d for d in data['max_delays'] if d > 0])
                
                od_final_summary[origin][destination] = {
                    'mean_avg_delay': mean_avg_delay,
                    'mean_max_delay': mean_max_delay,
                    'total_vehicles': total_vehicles,
                    'mean_travel_time': np.mean([t for t in data['avg_travel_times'] if t > 0]),
                    'distance_km': distance_km,
                    'avg_delay_per_km': mean_avg_delay / distance_km if distance_km > 0 else 0,
                    'max_delay_per_km': mean_max_delay / distance_km if distance_km > 0 else 0,
                    'has_traffic': True
                }
            else:
                od_final_summary[origin][destination] = {
                    'mean_avg_delay': 0, 'mean_max_delay': 0, 'total_vehicles': 0,
                    'mean_travel_time': 0, 'distance_km': distance_km,
                    'avg_delay_per_km': 0, 'max_delay_per_km': 0, 'has_traffic': False
                }
    
    return od_final_summary

def collect_timeloss_data():
    """Collect time loss data for each highway edge"""
    timeloss_data = {}
    for edge_id in HIGHWAY_EDGES_ORDERED:
        try:
            vehicles = traci.edge.getLastStepVehicleIDs(edge_id)
            if vehicles:
                total_loss = sum(traci.vehicle.getTimeLoss(veh_id) for veh_id in vehicles)
                timeloss_data[edge_id] = total_loss / len(vehicles)
            else:
                timeloss_data[edge_id] = 0
        except:
            timeloss_data[edge_id] = 0
    return timeloss_data

def collect_highway_data():
    """Collect speed and occupancy data for highway edges"""
    highway_data = {'speed': {}, 'occupancy': {}}
    for edge_id in HIGHWAY_EDGES_ORDERED:
        try:
            speed = traci.edge.getLastStepMeanSpeed(edge_id) * 3.6  # m/s to km/h
            occupancy = traci.edge.getLastStepOccupancy(edge_id)
            highway_data['speed'][edge_id] = speed
            highway_data['occupancy'][edge_id] = occupancy
        except:
            highway_data['speed'][edge_id] = 0
            highway_data['occupancy'][edge_id] = 0
    return highway_data

def parse_tripinfo_xml(filename):
    """Parse tripinfo XML file and extract comprehensive metrics"""
    try:
        tree = ET.parse(filename)
        root = tree.getroot()

        # Initialize metrics
        analysis_departed = analysis_arrived = 0
        total_travel_time = total_travel_distance = total_delay = 0
        travel_vehicles = 0
        
        # Ramp-specific data
        ramp_waits = {r: [] for r in RAMP_ORDER}
        departed_by_ramp = {r: 0 for r in RAMP_ORDER}
        arrived_by_ramp = {r: 0 for r in RAMP_ORDER}
        travel_time_by_ramp = {r: [] for r in RAMP_ORDER}
        
        # OD data
        od_delay_data = {r: {o: [] for o in OFF_RAMPS} for r in RAMP_ORDER}
        od_vehicle_counts = {r: {o: 0 for o in OFF_RAMPS} for r in RAMP_ORDER}
        od_travel_times = {r: {o: [] for o in OFF_RAMPS} for r in RAMP_ORDER}

        # Process each trip
        for trip in root.findall('tripinfo'):
            depart_time = float(trip.get('depart'))
            arrival_attr = trip.get('arrival')
            arrival_time = float(arrival_attr) if arrival_attr and arrival_attr != '-1' else None
            depart_lane = trip.get('departLane', '')
            arrival_lane = trip.get('arrivalLane', '')
            
            time_loss = float(trip.get('timeLoss', 0.0))
            duration = float(trip.get('duration', 0.0))
            route_length = float(trip.get('routeLength', 0.0))

            # Identify ramp from departure lane
            ramp_id = None
            if 'ramp' in depart_lane.lower():
                for ramp in RAMP_ORDER:
                    if ramp.lower() in depart_lane.lower():
                        ramp_id = ramp
                        break

            # Process ramp-specific data
            if ramp_id in RAMP_ORDER:
                if ANALYSIS_START <= depart_time <= ANALYSIS_END:
                    ramp_waits[ramp_id].append(time_loss)
                    departed_by_ramp[ramp_id] += 1

                if arrival_time and ANALYSIS_START <= arrival_time <= ANALYSIS_END:
                    arrived_by_ramp[ramp_id] += 1
                    travel_time_by_ramp[ramp_id].append(duration)

            # OD analysis
            origin_ramp, destination_ramp = identify_origin_destination(depart_lane, arrival_lane)
            if origin_ramp and destination_ramp:
                if (ANALYSIS_START <= depart_time <= ANALYSIS_END and 
                    arrival_time and ANALYSIS_START <= arrival_time <= ANALYSIS_END):
                    od_delay_data[origin_ramp][destination_ramp].append(time_loss)
                    od_vehicle_counts[origin_ramp][destination_ramp] += 1
                    od_travel_times[origin_ramp][destination_ramp].append(duration)

            # Network-wide statistics
            if ANALYSIS_START <= depart_time <= ANALYSIS_END:
                analysis_departed += 1
                if arrival_time and ANALYSIS_START <= arrival_time <= ANALYSIS_END:
                    analysis_arrived += 1

            if arrival_time and not (arrival_time < ANALYSIS_START or depart_time > ANALYSIS_END):
                travel_vehicles += 1
                total_travel_time += duration
                total_travel_distance += route_length
                total_delay += time_loss

        # Calculate final metrics
        arrival_rate = (analysis_arrived / analysis_departed * 100) if analysis_departed else 0
        avg_speed = (total_travel_distance / total_travel_time * 3.6) if total_travel_time else 0
        avg_delay = (total_delay / travel_vehicles / 60) if travel_vehicles else 0

        # Process OD metrics
        od_delay_metrics = {}
        for origin in RAMP_ORDER:
            od_delay_metrics[origin] = {}
            for destination in OFF_RAMPS:
                delays = od_delay_data[origin][destination]
                travel_times = od_travel_times[origin][destination]
                vehicle_count = od_vehicle_counts[origin][destination]
                
                if delays:
                    od_delay_metrics[origin][destination] = {
                        'avg_delay': np.mean(delays),
                        'max_delay': max(delays),
                        'vehicle_count': vehicle_count,
                        'avg_travel_time': np.mean(travel_times) if travel_times else 0.0
                    }
                else:
                    od_delay_metrics[origin][destination] = {
                        'avg_delay': 0.0, 'max_delay': 0.0, 'vehicle_count': 0, 'avg_travel_time': 0.0
                    }

        return {
            'total_departed': analysis_departed,
            'total_arrived': analysis_arrived,
            'arrival_rate': arrival_rate,
            'total_travel_time': total_travel_time / 3600,
            'total_travel_distance': total_travel_distance / 1000,
            'average_speed': avg_speed,
            'total_delay': total_delay / 3600,
            'average_delay': avg_delay,
            'ramp_waits': ramp_waits,
            'departed_by_ramp': departed_by_ramp,
            'arrived_by_ramp': arrived_by_ramp,
            'travel_time_by_ramp': travel_time_by_ramp,
            'od_delay_metrics': od_delay_metrics,
            'od_vehicle_counts': od_vehicle_counts
        }

    except Exception as e:
        print(f"Error parsing {filename}: {e}")
        return None

def calculate_ramp_metrics(all_seeds_data):
    """Calculate comprehensive ramp metrics from simulation data"""
    ramp_metrics = {}
    
    for ramp in RAMP_ORDER:
        vehicles_joined_list = []
        
        for seed_data in all_seeds_data['general_metrics']:
            if 'departed_by_ramp' in seed_data:
                vj = seed_data['departed_by_ramp'].get(ramp, 0)
                vehicles_joined_list.append(vj)
        
        # Calculate merge rate
        merge_rate_list = []
        analysis_duration_hours = (ANALYSIS_END - ANALYSIS_START) / 3600
        for vj in vehicles_joined_list:
            mr = vj / analysis_duration_hours if analysis_duration_hours > 0 else 0
            merge_rate_list.append(mr)
        
        # Red time data
        red_time_data = all_seeds_data['ramp_metrics'][ramp]['red_time_percentage'] if 'ramp_metrics' in all_seeds_data and ramp in all_seeds_data['ramp_metrics'] else []
        
        ramp_metrics[ramp] = {
            'merging_rate_mean': np.mean(merge_rate_list) if merge_rate_list else 0,
            'merging_rate_std': np.std(merge_rate_list) if merge_rate_list else 0,
            'vehicles_joined_mean': np.mean(vehicles_joined_list) if vehicles_joined_list else 0,
            'vehicles_joined_std': np.std(vehicles_joined_list) if vehicles_joined_list else 0,
            'red_time_percentage': np.mean(red_time_data) if red_time_data else 0,
            'red_time_percentage_std': np.std(red_time_data) if red_time_data else 0,
        }
    
    return ramp_metrics

def create_heatmap_matrices_with_timeloss(all_seeds_data):
    """Create matrices for heatmap visualization"""
    if not all_seeds_data or 'heatmap_data' not in all_seeds_data or not all_seeds_data['heatmap_data']:
        return None, None, None, None, None

    speed_data = all_seeds_data['heatmap_data']['speed']
    occupancy_data = all_seeds_data['heatmap_data']['occupancy']
    timeloss_data = all_seeds_data['heatmap_data'].get('timeloss', [])
    
    if not speed_data or not speed_data[0]:
        return None, None, None, None, None

    n_seeds = len(speed_data)
    n_times = len(speed_data[0])
    n_edges = len(HIGHWAY_EDGES_ORDERED)

    # Initialize matrices
    speed_matrix = np.full((n_times, n_edges), np.nan)
    occupancy_matrix = np.full((n_times, n_edges), np.nan)
    timeloss_matrix = np.full((n_times, n_edges), np.nan)

    # Fill matrices
    for t in range(n_times):
        for e_idx, edge in enumerate(HIGHWAY_EDGES_ORDERED):
            speeds = [speed_data[s][t].get(edge, np.nan) for s in range(n_seeds)]
            occs = [occupancy_data[s][t].get(edge, np.nan) * 100 for s in range(n_seeds)]
            losses = [timeloss_data[s][t].get(edge, np.nan) for s in range(n_seeds) if s < len(timeloss_data)]

            valid_speeds = [v for v in speeds if not np.isnan(v) and v > 0]
            valid_occs = [o for o in occs if not np.isnan(o) and o >= 0]
            valid_losses = [l for l in losses if not np.isnan(l) and l >= 0]

            if valid_speeds:
                speed_matrix[t, e_idx] = np.mean(valid_speeds)
            if valid_occs:
                occupancy_matrix[t, e_idx] = np.mean(valid_occs)
            if valid_losses:
                timeloss_matrix[t, e_idx] = np.mean(valid_losses)

    time_labels = [f"{40 + 5*t}min" for t in range(n_times)]
    position_labels = [f"{EDGE_BOUNDS_KM[e][0]:.1f}km" for e in HIGHWAY_EDGES_ORDERED]
    
    return speed_matrix, occupancy_matrix, timeloss_matrix, time_labels, position_labels

def create_heatmap_visualization_with_timeloss(speed_matrix, occupancy_matrix, timeloss_matrix, time_labels, position_labels, K_a, occ_targ, controller):
    """Create heatmap visualization with three subplots"""
    if speed_matrix is None:
        return None

    # Create position boundaries
    x_bounds = [EDGE_BOUNDS_KM[HIGHWAY_EDGES_ORDERED[0]][0]]
    for e in HIGHWAY_EDGES_ORDERED:
        x_bounds.append(EDGE_BOUNDS_KM[e][1])
    x_bounds = np.array(x_bounds)

    y_bounds = np.arange(speed_matrix.shape[0] + 1)

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(16, 18))
    
    # Generate parameter tag for title
    if controller in ["no_control", "metaline"]:
        param_tag = ""
    else:
        param_tag = f" (K={K_a}, T={occ_targ}%)"

    # Speed heatmap
    pcm1 = ax1.pcolormesh(x_bounds, y_bounds, speed_matrix, cmap='viridis', shading='auto', vmin=0, vmax=130)
    ax1.set_title(f"Speed – {controller.upper()}{param_tag}")
    ax1.set_ylabel('Time (5 min steps)')
    plt.colorbar(pcm1, ax=ax1, label='km/h')

    # Occupancy heatmap
    pcm2 = ax2.pcolormesh(x_bounds, y_bounds, occupancy_matrix, cmap='viridis', shading='auto', vmin=0, vmax=30)
    ax2.set_title(f"Occupancy – {controller.upper()}{param_tag}")
    ax2.set_ylabel('Time (5 min steps)')
    plt.colorbar(pcm2, ax=ax2, label='%')

    # Time Loss heatmap
    pcm3 = ax3.pcolormesh(x_bounds, y_bounds, timeloss_matrix, cmap='Reds', shading='auto', vmin=0)
    ax3.set_title(f"Average Time Loss – {controller.upper()}{param_tag}")
    ax3.set_ylabel('Time (5 min steps)')
    ax3.set_xlabel('Kilometres along carriageway')
    plt.colorbar(pcm3, ax=ax3, label='seconds')

    # Set ticks for all axes
    for ax in (ax1, ax2, ax3):
        xticks = x_bounds[::max(1, len(x_bounds)//15)]
        ax.set_xticks(xticks)
        ax.set_xticklabels([f'{x:.1f}' for x in xticks], rotation=45)

        # Y-axis time labels
        rows_per_hour = 3600 // 300
        hour_rows = range(0, speed_matrix.shape[0], rows_per_hour)
        ax.set_yticks(hour_rows)
        ax.set_yticklabels([f"{int((ANALYSIS_START + i*300) // 3600):02d}h" for i in hour_rows])

        # Add ramp markers
        for km_pos in RAMP_POSITIONS_KM.values():
            ax.axvline(km_pos, ls='--', lw=0.8, c='red', alpha=0.7)

    plt.tight_layout()

    # Generate filename
    if controller in ["no_control", "metaline"]:
        fname = f"heatmap_with_timeloss_{controller}_{datetime.now():%Y%m%d_%H%M%S}.png"
    else:
        fname = f"heatmap_with_timeloss_{controller}_K{K_a}_T{occ_targ}_{datetime.now():%Y%m%d_%H%M%S}.png"
    
    plt.savefig(fname, dpi=300, bbox_inches='tight')
    plt.close()
    return fname

def write_od_analysis_to_file(f, od_summary):
    """Write OD analysis to report file"""
    f.write("\n\nORIGIN-DESTINATION DELAY ANALYSIS\n")
    f.write("="*120 + "\n")
    f.write("Average Time Loss (seconds) and per km for each Origin-Destination pair\n")
    f.write("-"*120 + "\n")
    
    # Create header
    f.write(f"{'Origin\\Destination':<15}")
    for dest in OFF_RAMPS:
        dest_short = dest.replace('OFF_RAMP_', '')
        f.write(f" | {dest_short:<10}")
    f.write("\n")
    f.write("-"*120 + "\n")
    
    # Write OD delay matrix
    for origin in RAMP_ORDER:
        origin_short = origin.replace('TLS_ID_', '')
        f.write(f"{origin_short:<15}")
        
        for destination in OFF_RAMPS:
            if origin in od_summary and destination in od_summary[origin]:
                metrics = od_summary[origin][destination]
                if metrics['has_traffic']:
                    delay_val = metrics['mean_avg_delay']
                    f.write(f" | {delay_val:>10.1f}")
                else:
                    f.write(f" | {'--':>10}")
            else:
                f.write(f" | {'--':>10}")
        f.write("\n")

# #############################################################################
# ## COMMAND LINE INTERFACE
# #############################################################################

def print_help():
    """Print help information about running the simulation"""
    print("\n" + "="*80)
    print("COORDINATED ALINEA SIMULATION - HELP")
    print("="*80)
    print("\nUsage:")
    print("  python codecoor.py --sumo-path [PATH] --controller [CONTROLLER]")
    print("\nRequired Arguments:")
    print("  --sumo-path [PATH]     Path to SUMO installation directory")
    print("  --controller [CTRL]    Control algorithm to use")
    print("\nAvailable Controllers:")
    print("  NO_CONTROL            No ramp metering (baseline)")
    print("  ALINEA               Standard ALINEA algorithm") 
    print("  C_ALINEA          Coordinated ALINEA algorithm")
    print("  METALINE             Network-wide density control (METALINE)")
    print("\nAdditional Options for C_ALINEA:")
    print("  --neighbors [N]       Number of neighbors (1, 2, or 3)")
    print("  --method [M]          Weight calculation method (1 or 2)")
    print("\nExample Commands:")
    print("\n1. No Control (baseline):")
    print("   python codecoor.py --sumo-path ./sumo-1.19.0/bin/sumo-gui.exe --controller NO_CONTROL")
    print("\n2. Standard ALINEA:")
    print("   python codecoor.py --sumo-path ./sumo-1.19.0/bin/sumo-gui.exe --controller ALINEA")
    print("\n3. Coordinated ALINEA with 2 neighbors, method 1:")
    print("   python codecoor.py --sumo-path ./sumo-1.19.0/bin/sumo-gui.exe --controller C_ALINEA --neighbors 2 --method 1")
    print("\n4. METALINE network-wide control:")
    print("   python codecoor.py --sumo-path ./sumo-1.19.0/bin/sumo-gui.exe --controller METALINE")
    print("\nNote: Update the configuration file path in the code before running.")
    print("="*80 + "\n")

def parse_arguments():
    """Parse command line arguments for simulation configuration"""
    if len(sys.argv) < 2 or '--help' in sys.argv or '-h' in sys.argv:
        print_help()
        sys.exit(0)
    
    # Parse arguments
    args = {}
    i = 1
    while i < len(sys.argv):
        if sys.argv[i] == '--sumo-path' and i + 1 < len(sys.argv):
            args['sumo_path'] = sys.argv[i + 1]
            i += 2
        elif sys.argv[i] == '--controller' and i + 1 < len(sys.argv):
            args['controller'] = sys.argv[i + 1].upper()
            i += 2
        elif sys.argv[i] == '--neighbors' and i + 1 < len(sys.argv):
            args['neighbors'] = int(sys.argv[i + 1])
            i += 2
        elif sys.argv[i] == '--method' and i + 1 < len(sys.argv):
            args['method'] = sys.argv[i + 1]
            i += 2
        else:
            print(f"Unknown argument: {sys.argv[i]}")
            print_help()
            sys.exit(1)
    
    # Validate required arguments
    if 'sumo_path' not in args:
        print("Error: --sumo-path is required")
        print_help()
        sys.exit(1)
    
    if 'controller' not in args:
        print("Error: --controller is required")
        print_help()
        sys.exit(1)
    
    # Validate controller
    valid_controllers = ['NO_CONTROL', 'ALINEA', 'C_ALINEA', 'METALINE']
    if args['controller'] not in valid_controllers:
        print(f"Error: Invalid controller '{args['controller']}'. Must be one of: {valid_controllers}")
        sys.exit(1)
    
    # Validate C_ALINEA specific arguments
    if args['controller'] == 'C_ALINEA':
        if 'neighbors' not in args:
            print("Error: --neighbors is required for C_ALINEA")
            sys.exit(1)
        if 'method' not in args:
            print("Error: --method is required for C_ALINEA")
            sys.exit(1)
        if args['neighbors'] not in [1, 2, 3]:
            print("Error: --neighbors must be 1, 2, or 3")
            sys.exit(1)
        if args['method'] not in ['1', '2']:
            print("Error: --method must be 1 or 2")
            sys.exit(1)
    
    return args

def run_single_simulation(args):
    """Run a single simulation with specified parameters"""
    print("============================================================")
    print("COORDINATED ALINEA - RAMP METERING COORDINATION FOR HIGHWAY NETWORKS")
    print("by Kevin Riehl and Omar Alami Badissi 2025 (IVT, ETH Zürich)")
    print("============================================================")
    
    controller = args['controller'].lower()
    sumo_path = args['sumo_path']
    
    print(f"\nRunning simulation with:")
    print(f"  Controller: {controller.upper()}")
    print(f"  SUMO Path: {sumo_path}")
    
    # Initialize controller-specific parameters
    if controller == 'C_alinea':
        n_neighbors = args['neighbors']
        method = args['method']
        print(f"  Neighbors: {n_neighbors}")
        print(f"  Method: {method}")
    elif controller == 'metaline':
        print(f"  Network-wide density control enabled")
        print(f"  Target edges: {len(HIGHWAY_EDGES_ORDERED)}")
        print(f"  Controlled ramps: {len(RAMP_ORDER)}")
    
    # Storage for all seeds data
    all_seeds_data = {
        'general_metrics': [],
        'heatmap_data': {'speed': [], 'occupancy': [], 'timeloss': []},
        'ramp_waits': {ramp: [] for ramp in RAMP_ORDER}
    }
    
    # Run simulation for each K and target combination (skip for no_control and metaline)
    k_values = K_VALUES if controller not in ["no_control", "metaline"] else [0]
    occ_values = OCC_TARGET_VALUES if controller not in ["no_control", "metaline"] else [0]
    
    for K_a in k_values:
        for occ_targ in occ_values:
            if controller in ["no_control", "metaline"]:
                print(f"\nTesting {controller.upper()} (no K/target parameters)")
            else:
                print(f"\nTesting K={K_a}, Target={occ_targ}%")
            
            for seed_idx, seed in enumerate(SEEDS):
                print(f"Running simulation {seed_idx + 1}/{len(SEEDS)} with seed {seed}")
                
                # Initialize control structures
                occ_buffers = {ramp_id: deque(maxlen=30) for ramp_id in RAMP_ORDER}
                q_prev = {ramp_id: Q_MAX for ramp_id in RAMP_SENSOR}
                flow_buffers = {}
                metaline_controller = None
                
                # Initialize controller-specific structures
                if controller == "C_alinea":
                    flow_buffers = {
                        ramp_id: {
                            "self": deque(maxlen=60),
                            "after1": deque(maxlen=60), "before1": deque(maxlen=60),
                            "after2": deque(maxlen=60), "before2": deque(maxlen=60),
                            "after3": deque(maxlen=60), "before3": deque(maxlen=60),
                        }
                        for ramp_id in RAMP_SENSOR
                    }
                elif controller == "metaline":
                    # Initialize METALINE controller
                    metaline_controller = MetalineController(update_interval=60)
                
                # Seed-specific data storage
                seed_heatmap_data = {'speed': [], 'occupancy': [], 'timeloss': []}
                seed_ramp_metrics = {
                    ramp: {
                        'red_time_total': 0,
                        'green_time_total': 0,
                        'queue_lengths': [],
                        'phase_changes': 0,
                        'prev_tl_state': None,
                    }
                    for ramp in RAMP_ORDER
                }
                
                # Configure simulation files with proper naming
                if controller == "C_alinea":
                    suffix = f"{controller}_{n_neighbors}neigh_method{method}_K{K_a}_target{occ_targ}_{seed}"
                elif controller in ["no_control", "metaline"]:
                    suffix = f"{controller}_{seed}"
                else:
                    suffix = f"{controller}_K{K_a}_target{occ_targ}_{seed}"
                
                sumoCmd = [
                    sumo_path,
                    "-c","Configuration.sumocfg",  # Update this path
                    "--seed", str(seed),
                    "--tripinfo-output", f"output/tripinfo_{suffix}.xml",
                    "--summary-output", f"output/summary_{suffix}.xml",
                ]
                
                traci.start(sumoCmd)
                
                # Main simulation loop
                for t in range(SIM_DURATION):
                    traci.simulationStep()
                    
                    # Collect heatmap data every 5 minutes during analysis period
                    if ANALYSIS_START <= t <= ANALYSIS_END and t % 300 == 0:
                        highway_data = collect_highway_data()
                        timeloss_data = collect_timeloss_data()
                        seed_heatmap_data['speed'].append(highway_data['speed'])
                        seed_heatmap_data['occupancy'].append(highway_data['occupancy'])
                        seed_heatmap_data['timeloss'].append(timeloss_data)
                    
                    # Apply control logic based on controller type
                    if controller == "metaline":
                        # METALINE handles all ramps globally
                        metaline_controller.step(t)
                    else:
                        # Individual ramp control (no_control, alinea, C_alinea)
                        if t < WARM_UP:
                            # Warm-up period: no control (all lights green)
                            for ramp_id in RAMP_ORDER:
                                light(1, ramp_id)
                        else:
                            # Control period: apply selected controller
                            for i, ramp_id in enumerate(RAMP_ORDER):
                                # Get occupancy from downstream sensors
                                downstream_sensors = RAMP_SENSOR[ramp_id][2:]
                                try:
                                    occ_now = sum([traci.lanearea.getLastStepOccupancy(d) for d in downstream_sensors]) / len(downstream_sensors)
                                except:
                                    occ_now = 0
                                
                                # Update occupancy buffer and calculate average
                                occ_buffers[ramp_id].append(occ_now)
                                occ = (sum(occ_buffers[ramp_id]) / len(occ_buffers[ramp_id])) if occ_buffers[ramp_id] else 0
                                
                                # Apply control strategy
                                if controller == "no_control":
                                    r = 1  # Always green
                                elif controller == "alinea":
                                    q, r = alinea(q_prev[ramp_id], occ, K_a, occ_targ)
                                    q_prev[ramp_id] = q
                                elif controller == "C_alinea":
                                    # Get neighbor indices for coordination
                                    neighbors = {}
                                    for j in range(1, n_neighbors + 1):
                                        neighbors[f'before{j}'] = RAMP_ORDER[(i - j) % len(RAMP_ORDER)]
                                        neighbors[f'after{j}'] = RAMP_ORDER[(i + j) % len(RAMP_ORDER)]
                                    
                                    # Update flow buffers with current measurements
                                    try:
                                        flow_buffers[ramp_id]["self"].append(
                                            traci.lanearea.getLastStepVehicleNumber(RAMP_SENSOR[ramp_id][1])
                                        )
                                        for direction, neighbor in neighbors.items():
                                            flow_buffers[ramp_id][direction].append(
                                                traci.lanearea.getLastStepVehicleNumber(RAMP_SENSOR[neighbor][1])
                                            )
                                    except:
                                        pass
                                    
                                    # Calculate distance-based weights
                                    weights = calculate_weights(DISTANCES[ramp_id], n_neighbors, method)
                                    
                                    # Apply coordinated ALINEA
                                    q, r = C_alinea(q_prev[ramp_id], occ, K_a, occ_targ, flow_buffers[ramp_id], weights)
                                    q_prev[ramp_id] = q
                                
                                # Control traffic light (except for METALINE which handles this internally)
                                if controller != "metaline":
                                    light(r, ramp_id)
                    
                    # Collect traffic light metrics during analysis period
                    if ANALYSIS_START <= t <= ANALYSIS_END:
                        for ramp_id in RAMP_ORDER:
                            try:
                                # Queue length measurement
                                try:
                                    q_len = traci.lanearea.getJamLengthMeters(RAMP_SENSOR[ramp_id][0])
                                except traci.TraCIException:
                                    q_len = 0
                                seed_ramp_metrics[ramp_id]['queue_lengths'].append(q_len)

                                # Traffic light state monitoring
                                tl_state = traci.trafficlight.getRedYellowGreenState(ramp_id)
                                if 'r' in tl_state.lower() or 'y' in tl_state.lower():
                                    seed_ramp_metrics[ramp_id]['red_time_total'] += 1
                                else:
                                    seed_ramp_metrics[ramp_id]['green_time_total'] += 1
                                
                                prev_state = seed_ramp_metrics[ramp_id]['prev_tl_state']
                                if prev_state is not None and prev_state != tl_state:
                                    seed_ramp_metrics[ramp_id]['phase_changes'] += 1
                                seed_ramp_metrics[ramp_id]['prev_tl_state'] = tl_state
                            except Exception:
                                pass
                
                traci.close()
                
                # Store queue metrics
                if 'queue_metrics' not in all_seeds_data:
                    all_seeds_data['queue_metrics'] = {
                        r: {'max_queue': [], 'avg_queue': [], 'cum_queue': [], 'phase_changes': []}
                        for r in RAMP_ORDER
                    }

                for r in RAMP_ORDER:
                    q_list = seed_ramp_metrics[r].get('queue_lengths', [])
                    max_q = max(q_list) if q_list else 0
                    avg_q = np.mean(q_list) if q_list else 0
                    cum_q = sum(q_list) if q_list else 0

                    all_seeds_data['queue_metrics'][r]['max_queue'].append(max_q)
                    all_seeds_data['queue_metrics'][r]['avg_queue'].append(avg_q)
                    all_seeds_data['queue_metrics'][r]['cum_queue'].append(cum_q)
                    all_seeds_data['queue_metrics'][r]['phase_changes'].append(
                        seed_ramp_metrics[r]['phase_changes']
                    )
                
                # Store ramp metrics for red time analysis
                all_seeds_data.setdefault('ramp_metrics', {})
                for ramp in RAMP_ORDER:
                    if ramp not in all_seeds_data['ramp_metrics']:
                        all_seeds_data['ramp_metrics'][ramp] = {'red_time_percentage': []}
                    
                    total_time = ANALYSIS_END - ANALYSIS_START
                    red_pct = (seed_ramp_metrics[ramp]['red_time_total'] / total_time) * 100 if total_time > 0 else 0
                    all_seeds_data['ramp_metrics'][ramp]['red_time_percentage'].append(red_pct)
                
                # Parse tripinfo results
                results = parse_tripinfo_xml(f"output/tripinfo_{suffix}.xml")
                if results:
                    all_seeds_data['general_metrics'].append(results)
                    for ramp in RAMP_ORDER:
                        all_seeds_data['ramp_waits'][ramp].append(results['ramp_waits'].get(ramp, []))
                
                # Store heatmap data
                all_seeds_data['heatmap_data']['speed'].append(seed_heatmap_data['speed'])
                all_seeds_data['heatmap_data']['occupancy'].append(seed_heatmap_data['occupancy'])
                all_seeds_data['heatmap_data']['timeloss'].append(seed_heatmap_data['timeloss'])
                
                print(f"Completed seed {seed}")
    
    print(f"\nAll {len(SEEDS)} simulations completed. Generating analysis...")
    
    # Generate comprehensive analysis report with proper naming
    if controller == "C_alinea":
        analysis_filename = f"analysis_{controller}_{n_neighbors}neigh_method{method}_K{k_values[0]}_target{occ_values[0]}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
    elif controller in ["no_control", "metaline"]:
        analysis_filename = f"analysis_{controller}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
    else:
        analysis_filename = f"analysis_{controller}_K{k_values[0]}_target{occ_values[0]}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
    
    # Write comprehensive analysis report
    with open(analysis_filename, 'w') as f:
        f.write("="*80 + "\n")
        f.write(f"TRAFFIC SIMULATION ANALYSIS REPORT - {controller.upper()}\n")
        
        # Add controller-specific parameters to header
        if controller == "C_alinea":
            f.write(f"K_a = {k_values[0]}, Occupancy Target = {occ_values[0]}%\n")
            f.write(f"Number of neighbors = {n_neighbors}\n")
            f.write(f"Weight calculation method = {method}\n")
        elif controller == "alinea":
            f.write(f"K_a = {k_values[0]}, Occupancy Target = {occ_values[0]}%\n")
        elif controller == "metaline":
            f.write(f"Network-wide density control algorithm\n")
            f.write(f"Target edges: {len(HIGHWAY_EDGES_ORDERED)}, Controlled ramps: {len(RAMP_ORDER)}\n")
        elif controller == "no_control":
            f.write(f"Baseline scenario without ramp metering\n")
            
        f.write(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"Seeds used: {SEEDS}\n")
        f.write("="*80 + "\n\n")
        
        # General network metrics
        if all_seeds_data['general_metrics']:
            metrics_df = pd.DataFrame(all_seeds_data['general_metrics'])
            f.write("GENERAL SIMULATION METRICS\n")
            f.write("-" * 50 + "\n")
            f.write(f"{'Metric':<35} | {'Mean':<12} | {'Std Dev':<12}\n")
            f.write("-" * 64 + "\n")
            f.write(f"{'Total Departed Vehicles':<35} | {metrics_df['total_departed'].mean():<12.1f} | {metrics_df['total_departed'].std():<12.1f}\n")
            f.write(f"{'Total Arrived Vehicles':<35} | {metrics_df['total_arrived'].mean():<12.1f} | {metrics_df['total_arrived'].std():<12.1f}\n")
            f.write(f"{'Arrival Rate (%)':<35} | {metrics_df['arrival_rate'].mean():<12.1f} | {metrics_df['arrival_rate'].std():<12.1f}\n")
            f.write(f"{'Total Travel Time (hours)':<35} | {metrics_df['total_travel_time'].mean():<12.1f} | {metrics_df['total_travel_time'].std():<12.1f}\n")
            f.write(f"{'Total Travel Distance (km)':<35} | {metrics_df['total_travel_distance'].mean():<12.1f} | {metrics_df['total_travel_distance'].std():<12.1f}\n")
            f.write(f"{'Total Delay (hours)':<35} | {metrics_df['total_delay'].mean():<12.1f} | {metrics_df['total_delay'].std():<12.1f}\n")
            f.write(f"{'Average Speed (km/h)':<35} | {metrics_df['average_speed'].mean():<12.1f} | {metrics_df['average_speed'].std():<12.1f}\n")
            f.write(f"{'Average Delay (min/car)':<35} | {metrics_df['average_delay'].mean():<12.1f} | {metrics_df['average_delay'].std():<12.1f}\n")
            
            # Queue analysis
            if 'queue_metrics' in all_seeds_data:
                f.write("\nQUEUE ANALYSIS\n")
                f.write("-" * 100 + "\n")
                f.write(f"{'Ramp':<12} | {'Max Queue (m)':<12} | {'Avg Queue (m)':<12} | {'Cum Queue (m)':<12} | {'Phase Changes':<15}\n")
                f.write("-" * 100 + "\n")
                
                for ramp in RAMP_ORDER:
                    queue_data = all_seeds_data['queue_metrics'][ramp]
                    max_q = np.mean(queue_data['max_queue']) if queue_data['max_queue'] else 0
                    avg_q = np.mean(queue_data['avg_queue']) if queue_data['avg_queue'] else 0
                    cum_q = np.mean(queue_data['cum_queue']) if queue_data['cum_queue'] else 0
                    phase_ch = np.mean(queue_data['phase_changes']) if queue_data['phase_changes'] else 0
                    
                    f.write(f"{ramp:<12} | {max_q:<12.1f} | {avg_q:<12.1f} | {cum_q:<12.1f} | {phase_ch:<15.0f}\n")

            # Comprehensive ramp analysis
            ramp_metrics = calculate_ramp_metrics(all_seeds_data)
            f.write("\nCOMPREHENSIVE RAMP ANALYSIS\n")
            f.write("-" * 100 + "\n")
            f.write(f"{'Ramp':<12} | {'Avg Delay':<10} | {'Std Delay':<10} | {'Merge Rate':<12} | {'Vehicles':<10} | {'Red Time%':<10}\n")
            f.write(f"{'':12} | {'(s)':<10} | {'(s)':<10} | {'(veh/h)':<12} | {'Joined':<10} | {'':10}\n")
            f.write("-" * 100 + "\n")

            for ramp in RAMP_ORDER:
                # Calculate delay statistics
                all_time_loss = []
                for seed_waits in all_seeds_data['ramp_waits'][ramp]:
                    if seed_waits:
                        all_time_loss.extend(seed_waits)
                
                avg_delay = np.mean(all_time_loss) if all_time_loss else 0
                std_delay = np.std(all_time_loss) if all_time_loss else 0
                
                metrics = ramp_metrics.get(ramp, {})
                merge_rate = metrics.get('merging_rate_mean', 0)
                vehicles = metrics.get('vehicles_joined_mean', 0)
                red_time_pct = metrics.get('red_time_percentage', 0)

                f.write(f"{ramp:<12} | {avg_delay:<10.1f} | {std_delay:<10.1f} | "
                       f"{merge_rate:<12.2f} | {vehicles:<10.0f} | {red_time_pct:<10.1f}\n")

            # Origin-Destination analysis
            print("Generating OD delay analysis...")
            od_summary = calculate_od_delay_summary(all_seeds_data)
            if od_summary:
                write_od_analysis_to_file(f, od_summary)

            # Summary statistics
            f.write("\n" + "="*80 + "\n")
            f.write("SUMMARY STATISTICS\n")
            f.write("-" * 50 + "\n")
            f.write(f"Best Average Speed: {metrics_df['average_speed'].max():.2f} km/h\n")
            f.write(f"Lowest Average Delay: {metrics_df['average_delay'].min():.2f} minutes/car\n")
            f.write(f"Highest Arrival Rate: {metrics_df['arrival_rate'].max():.2f}%\n")
        
        f.write("\n" + "="*80 + "\n")
    
    print(f"Analysis report saved to: {analysis_filename}")
    
    # Generate heatmap visualization with proper parameters
    print("Generating heatmap visualization...")
    speed_matrix, occupancy_matrix, timeloss_matrix, time_labels, position_labels = create_heatmap_matrices_with_timeloss(all_seeds_data)
    
    # Pass appropriate parameters for heatmap naming
    if controller in ["no_control", "metaline"]:
        heatmap_filename = create_heatmap_visualization_with_timeloss(speed_matrix, occupancy_matrix, timeloss_matrix, time_labels, position_labels, 0, 0, controller)
    else:
        heatmap_filename = create_heatmap_visualization_with_timeloss(speed_matrix, occupancy_matrix, timeloss_matrix, time_labels, position_labels, k_values[0], occ_values[0], controller)
    
    # Export segment averages
    segments_csv = None
    if speed_matrix is not None:
        mean_speed = np.nanmean(speed_matrix, axis=0)
        mean_occupancy = np.nanmean(occupancy_matrix, axis=0)

        seg_rows = []
        for edge_id, km_bounds, spd, occ in zip(
                HIGHWAY_EDGES_ORDERED,
                EDGE_BOUNDS_KM.values(),
                mean_speed, mean_occupancy):
            seg_rows.append({
                'edge': edge_id,
                'km_start': km_bounds[0],
                'km_end': km_bounds[1],
                'mean_speed': spd,
                'mean_occupancy': occ
            })

        # Generate CSV filename based on controller type
        if controller == "C_alinea":
            segments_csv = f"segments_{controller}_{n_neighbors}neigh_method{method}_K{k_values[0]}_T{occ_values[0]}_{datetime.now():%Y%m%d_%H%M%S}.csv"
        elif controller in ["no_control", "metaline"]:
            segments_csv = f"segments_{controller}_{datetime.now():%Y%m%d_%H%M%S}.csv"
        else:
            segments_csv = f"segments_{controller}_K{k_values[0]}_T{occ_values[0]}_{datetime.now():%Y%m%d_%H%M%S}.csv"
        
        pd.DataFrame(seg_rows).to_csv(segments_csv, index=False)
        print(f"Segment averages saved to: {segments_csv}")
    
    print("\nSimulation completed successfully!")
    print(f"Files generated:")
    print(f"  - Analysis report: {analysis_filename}")
    if heatmap_filename:
        print(f"  - Heatmap visualization: {heatmap_filename}")
    if segments_csv:
        print(f"  - Segment averages: {segments_csv}")

def print_help():
    """Print help information about running the simulation"""
    print("\n" + "="*80)
    print("COORDINATED ALINEA SIMULATION - HELP")
    print("="*80)
    print("\nUsage:")
    print("  python codecoor.py --sumo-path [PATH] --controller [CONTROLLER]")
    print("\nRequired Arguments:")
    print("  --sumo-path [PATH]     Path to SUMO installation directory")
    print("  --controller [CTRL]    Control algorithm to use")
    print("\nAvailable Controllers:")
    print("  NO_CONTROL            No ramp metering (baseline)")
    print("  ALINEA               Standard ALINEA algorithm") 
    print("  C_ALINEA          Coordinated ALINEA algorithm")
    print("  METALINE             Network-wide density control (METALINE)")
    print("\nAdditional Options for C_ALINEA:")
    print("  --neighbors [N]       Number of neighbors (1, 2, or 3)")
    print("  --method [M]          Weight calculation method (1 or 2)")
    print("\nExample Commands:")
    print("\n1. No Control (baseline):")
    print("   python codecoor.py --sumo-path ./sumo-1.19.0/bin/sumo-gui.exe --controller NO_CONTROL")
    print("\n2. Standard ALINEA:")
    print("   python codecoor.py --sumo-path ./sumo-1.19.0/bin/sumo-gui.exe --controller ALINEA")
    print("\n3. Coordinated ALINEA with 2 neighbors, method 1:")
    print("   python codecoor.py --sumo-path ./sumo-1.19.0/bin/sumo-gui.exe --controller C_ALINEA --neighbors 2 --method 1")
    print("\n4. METALINE network-wide control:")
    print("   python codecoor.py --sumo-path ./sumo-1.19.0/bin/sumo-gui.exe --controller METALINE")
    print("\nNote: Update the configuration file path in the code before running.")
    print("="*80 + "\n")

def parse_arguments():
    """Parse command line arguments for simulation configuration"""
    import sys
    
    if len(sys.argv) < 2 or '--help' in sys.argv or '-h' in sys.argv:
        print_help()
        sys.exit(0)
    
    # Parse arguments
    args = {}
    i = 1
    while i < len(sys.argv):
        if sys.argv[i] == '--sumo-path' and i + 1 < len(sys.argv):
            args['sumo_path'] = sys.argv[i + 1]
            i += 2
        elif sys.argv[i] == '--controller' and i + 1 < len(sys.argv):
            args['controller'] = sys.argv[i + 1].upper()
            i += 2
        elif sys.argv[i] == '--neighbors' and i + 1 < len(sys.argv):
            args['neighbors'] = int(sys.argv[i + 1])
            i += 2
        elif sys.argv[i] == '--method' and i + 1 < len(sys.argv):
            args['method'] = sys.argv[i + 1]
            i += 2
        else:
            print(f"Unknown argument: {sys.argv[i]}")
            print_help()
            sys.exit(1)
    
    # Validate required arguments
    if 'sumo_path' not in args:
        print("Error: --sumo-path is required")
        print_help()
        sys.exit(1)
    
    if 'controller' not in args:
        print("Error: --controller is required")
        print_help()
        sys.exit(1)
    
    # Validate controller
    valid_controllers = ['NO_CONTROL', 'ALINEA', 'C_ALINEA', 'METALINE']
    if args['controller'] not in valid_controllers:
        print(f"Error: Invalid controller '{args['controller']}'. Must be one of: {valid_controllers}")
        sys.exit(1)
    
    # Validate C_ALINEA specific arguments
    if args['controller'] == 'C_ALINEA':
        if 'neighbors' not in args:
            print("Error: --neighbors is required for C_ALINEA")
            sys.exit(1)
        if 'method' not in args:
            print("Error: --method is required for C_ALINEA")
            sys.exit(1)
        if args['neighbors'] not in [1, 2, 3]:
            print("Error: --neighbors must be 1, 2, or 3")
            sys.exit(1)
        if args['method'] not in ['1', '2']:
            print("Error: --method must be 1 or 2")
            sys.exit(1)
    
    return args

if __name__ == "__main__":
    """
    Main execution point for the coordinated ALINEA simulation
    
    Command line usage examples:
    python codecoor.py --sumo-path ./sumo-1.19.0/bin/sumo-gui.exe --controller NO_CONTROL
    python codecoor.py --sumo-path ./sumo-1.19.0/bin/sumo-gui.exe --controller ALINEA
    python codecoor.py --sumo-path ./sumo-1.19.0/bin/sumo-gui.exe --controller C_ALINEA --neighbors 2 --method 1
    python codecoor.py --sumo-path ./sumo-1.19.0/bin/sumo-gui.exe --controller METALINE
    """
    
    # Parse command line arguments
    args = parse_arguments()
    
    # Run single simulation with specified parameters
    run_single_simulation(args)