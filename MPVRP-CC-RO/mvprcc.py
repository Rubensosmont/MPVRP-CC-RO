# -*- coding: utf-8 -*-
"""
MPVRP-CC Complete Solver
Multi-Product Vehicle Routing Problem with Changeover Cost

This is an all-in-one solution that includes:
- Multiple solving algorithms (Greedy, CP-SAT)
- Solution validation
- Detailed output and statistics
- Error handling and debugging

Author: MPVRP-CC Team
Date: January 2026
"""

import sys
import math
import time
import argparse
from copy import deepcopy

try:
    from ortools.sat.python import cp_model
    ORTOOLS_AVAILABLE = True
except ImportError:
    ORTOOLS_AVAILABLE = False
    print(" Warning: OR-Tools not available. Only greedy algorithm will be available.")


# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================

def euclidean(a, b):
    """Calculate Euclidean distance between two points"""
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)


def print_section(title):
    """Print a section header"""
    print("\n" + "=" * 70)
    print(f"  {title}")
    print("=" * 70)


# ============================================================================
# DATA READING
# ============================================================================

def read_instance(filename):
    """Read and parse MPVRP-CC instance file"""
    print(f"\n Reading instance: {filename}")
    
    try:
        with open(filename, 'r') as f:
            lines = [l.strip() for l in f if l.strip() and not l.startswith('#')]
    except FileNotFoundError:
        print(f" Error: File '{filename}' not found")
        sys.exit(1)
    
    idx = 0
    
    # Skip UUID if present
    if '-' in lines[0]:
        idx += 1
    
    # Global parameters
    NbProducts, NbDepots, NbGarages, NbStations, NbVehicles = map(int, lines[idx].split())
    idx += 1
    
    print(f"   Products: {NbProducts}, Depots: {NbDepots}, Garages: {NbGarages}")
    print(f"   Stations: {NbStations}, Vehicles: {NbVehicles}")

    # Transition cost matrix
    switch_cost = []
    for _ in range(NbProducts):
        switch_cost.append([float(x) for x in lines[idx].split()])
        idx += 1

    # Vehicles
    vehicles = []
    for _ in range(NbVehicles):
        v = list(map(int, lines[idx].split()))
        vehicles.append({
            'id': v[0],
            'cap': v[1],
            'gar_id': v[2] - 1,  # Convert to 0-indexed
            'prod_id': v[3] - 1   # Convert to 0-indexed
        })
        idx += 1

    # Depots
    depots = []
    for _ in range(NbDepots):
        d = list(map(float, lines[idx].split()))
        depots.append({
            'id': int(d[0]),
            'coord': (d[1], d[2]),
            'stock': [int(x) for x in d[3:3 + NbProducts]]
        })
        idx += 1

    # Garages
    garages = []
    for _ in range(NbGarages):
        g = list(map(float, lines[idx].split()))
        garages.append({
            'id': int(g[0]),
            'coord': (g[1], g[2])
        })
        idx += 1

    # Stations
    stations = []
    total_demand = 0
    for _ in range(NbStations):
        s = list(map(float, lines[idx].split()))
        demand = [int(x) for x in s[3:3 + NbProducts]]
        total_demand += sum(demand)
        stations.append({
            'id': int(s[0]),
            'coord': (s[1], s[2]),
            'dem': demand
        })
        idx += 1
    
    print(f"   Total demand: {total_demand} units")

    return {
        'NbProducts': NbProducts,
        'NbDepots': NbDepots,
        'NbGarages': NbGarages,
        'NbStations': NbStations,
        'NbVehicles': NbVehicles,
        'vehicles': vehicles,
        'depots': depots,
        'garages': garages,
        'stations': stations,
        'switch_cost': switch_cost
    }


# ============================================================================
# GREEDY SOLVER
# ============================================================================

def solve_greedy(instance, verbose=True):
    """
    Greedy constructive heuristic for MPVRP-CC
    
    Strategy: Iteratively assign deliveries to minimize immediate cost
    """
    if verbose:
        print_section("GREEDY HEURISTIC SOLVER")
    
    start_time = time.time()
    
    vehicles = instance['vehicles']
    depots = instance['depots']
    garages = instance['garages']
    stations = instance['stations']
    switch_cost = instance['switch_cost']
    NbProducts = instance['NbProducts']
    
    # Initialize remaining demands
    remaining_demand = []
    for s_idx, station in enumerate(stations):
        for p_idx, demand in enumerate(station['dem']):
            if demand > 0:
                remaining_demand.append({
                    'station_idx': s_idx,
                    'product_idx': p_idx,
                    'demand': demand
                })
    
    # Initialize vehicle states
    vehicle_states = []
    for v_idx, vehicle in enumerate(vehicles):
        vehicle_states.append({
            'vehicle': vehicle,
            'current_product': vehicle['prod_id'],
            'current_load': 0,
            'current_depot': None,
            'trips': [],
            'current_trip': None
        })
    
    # Initialize depot stock tracking
    depot_stock = []
    for depot in depots:
        depot_stock.append(depot['stock'][:])  # Copy
    
    iteration = 0
    # Greedy assignment loop
    while remaining_demand:
        iteration += 1
        if verbose and iteration % 10 == 0:
            print(f"   Processing... {len(remaining_demand)} demands remaining")
        
        best_assignment = None
        best_cost = float('inf')
        
        # Find best assignment
        for demand_item in remaining_demand:
            s_idx = demand_item['station_idx']
            p_idx = demand_item['product_idx']
            demand = demand_item['demand']
            
            for v_idx, v_state in enumerate(vehicle_states):
                vehicle = v_state['vehicle']
                
                # Try each depot
                for d_idx, depot in enumerate(depots):
                    if depot_stock[d_idx][p_idx] <= 0:
                        continue
                    
                    # Calculate cost of this assignment
                    g_coord = garages[vehicle['gar_id']]['coord']
                    d_coord = depot['coord']
                    s_coord = stations[s_idx]['coord']
                    
                    # Quantity to deliver
                    qty = min(demand, vehicle['cap'], depot_stock[d_idx][p_idx])
                    
                    if qty <= 0:
                        continue
                    
                    # Check if we need a new trip
                    need_new_trip = (
                        v_state['current_trip'] is None or
                        v_state['current_product'] != p_idx or
                        v_state['current_depot'] != d_idx or
                        v_state['current_load'] + qty > vehicle['cap']
                    )
                    
                    if need_new_trip:
                        # Cost of new trip
                        distance_cost = (
                            euclidean(g_coord, d_coord) +
                            euclidean(d_coord, s_coord) +
                            euclidean(s_coord, g_coord)
                        )
                        
                        # Changeover cost
                        changeover = 0
                        if v_state['current_product'] != p_idx:
                            changeover = switch_cost[v_state['current_product']][p_idx]
                        
                        total_cost = distance_cost + changeover
                    else:
                        # Continue current trip - just add station visit
                        # Approximate: distance from depot to station
                        distance_cost = euclidean(d_coord, s_coord) * 2
                        total_cost = distance_cost
                    
                    if total_cost < best_cost:
                        best_cost = total_cost
                        best_assignment = {
                            'demand_item': demand_item,
                            'vehicle_idx': v_idx,
                            'depot_idx': d_idx,
                            'qty': qty,
                            'new_trip': need_new_trip
                        }
        
        if best_assignment is None:
            print(" ERROR: Cannot find feasible assignment for remaining demand")
            print(f"   Remaining demands: {len(remaining_demand)}")
            for item in remaining_demand[:5]:  # Show first 5
                print(f"   - Station {stations[item['station_idx']]['id']}, "
                      f"Product {item['product_idx']+1}, Demand {item['demand']}")
            break
        
        # Apply best assignment
        demand_item = best_assignment['demand_item']
        v_idx = best_assignment['vehicle_idx']
        d_idx = best_assignment['depot_idx']
        qty = best_assignment['qty']
        s_idx = demand_item['station_idx']
        p_idx = demand_item['product_idx']
        
        v_state = vehicle_states[v_idx]
        
        if best_assignment['new_trip']:
            # Finish current trip if exists
            if v_state['current_trip'] is not None:
                v_state['trips'].append(v_state['current_trip'])
            
            # Start new trip
            v_state['current_trip'] = {
                'depot_idx': d_idx,
                'product_idx': p_idx,
                'load': qty,
                'deliveries': [(s_idx, qty)]
            }
            v_state['current_product'] = p_idx
            v_state['current_depot'] = d_idx
            v_state['current_load'] = qty
        else:
            # Add to current trip
            v_state['current_trip']['deliveries'].append((s_idx, qty))
            v_state['current_trip']['load'] += qty
            v_state['current_load'] += qty
        
        # Update demand and stock
        demand_item['demand'] -= qty
        depot_stock[d_idx][p_idx] -= qty
        
        if demand_item['demand'] <= 0:
            remaining_demand.remove(demand_item)
    
    # Finish all current trips
    for v_state in vehicle_states:
        if v_state['current_trip'] is not None:
            v_state['trips'].append(v_state['current_trip'])
    
    # Build routes
    routes = []
    for v_idx, v_state in enumerate(vehicle_states):
        if v_state['trips']:
            routes.append({
                'vehicle_id': v_state['vehicle']['id'],
                'vehicle_idx': v_idx,
                'garage_id': garages[v_state['vehicle']['gar_id']]['id'],
                'initial_product': v_state['vehicle']['prod_id'],
                'trips': v_state['trips']
            })
    
    # Calculate metrics
    metrics = calculate_metrics(routes, instance, start_time)
    
    if verbose:
        print(f"\n✓ Solution found in {metrics['cpu_time']:.3f}s")
        print(f"   Vehicles used: {metrics['used_vehicles']}/{instance['NbVehicles']}")
        print(f"   Total distance: {metrics['distance_total']:.2f}")
        print(f"   Product changes: {metrics['total_changes']}")
        print(f"   Changeover cost: {metrics['total_switch_cost']:.2f}")
    
    return routes, metrics


# ============================================================================
# CP-SAT SOLVER (if OR-Tools available)
# ============================================================================

def solve_cpsat(instance, time_limit=300, verbose=True):
    """
    CP-SAT based solver for MPVRP-CC
    Better for small to medium instances
    """
    if not ORTOOLS_AVAILABLE:
        print(" OR-Tools not available. Please install: pip install ortools")
        return None, None
    
    if verbose:
        print_section("CP-SAT SOLVER")
        print(f"   Time limit: {time_limit}s")
    
    start_time = time.time()
    
    vehicles = instance['vehicles']
    depots = instance['depots']
    garages = instance['garages']
    stations = instance['stations']
    switch_cost = instance['switch_cost']
    NbProducts = instance['NbProducts']
    NbDepots = instance['NbDepots']
    NbStations = instance['NbStations']
    NbVehicles = instance['NbVehicles']
    
    model = cp_model.CpModel()
    COST_MULTIPLIER = 100
    MAX_TRIPS = min(15, NbStations + 5)
    
    if verbose:
        print(f"   Max trips per vehicle: {MAX_TRIPS}")
    
    # Decision variables
    trip_active = {}
    load = {}
    deliver = {}
    
    for k, v in enumerate(vehicles):
        for t in range(MAX_TRIPS):
            for d in range(NbDepots):
                for p in range(NbProducts):
                    trip_active[(k,t,d,p)] = model.NewBoolVar(f"trip_{k}_{t}_{d}_{p}")
                    load[(k,t,d,p)] = model.NewIntVar(0, v['cap'], f"load_{k}_{t}_{d}_{p}")
                    
                    for s in range(NbStations):
                        deliver[(k,t,d,p,s)] = model.NewIntVar(0, v['cap'], f"del_{k}_{t}_{d}_{p}_{s}")
    
    # Constraints
    
    # 1. One (depot,product) per trip max
    for k in range(NbVehicles):
        for t in range(MAX_TRIPS):
            model.Add(sum(trip_active[(k,t,d,p)] 
                         for d in range(NbDepots) 
                         for p in range(NbProducts)) <= 1)
    
    # 2. Load = sum of deliveries
    for k, v in enumerate(vehicles):
        for t in range(MAX_TRIPS):
            for d in range(NbDepots):
                for p in range(NbProducts):
                    model.Add(load[(k,t,d,p)] == 
                             sum(deliver[(k,t,d,p,s)] for s in range(NbStations))
                             ).OnlyEnforceIf(trip_active[(k,t,d,p)])
                    
                    model.Add(load[(k,t,d,p)] == 0).OnlyEnforceIf(trip_active[(k,t,d,p)].Not())
                    for s in range(NbStations):
                        model.Add(deliver[(k,t,d,p,s)] == 0).OnlyEnforceIf(trip_active[(k,t,d,p)].Not())
    
    # 3. Demand satisfaction
    for s in range(NbStations):
        for p in range(NbProducts):
            if stations[s]['dem'][p] > 0:
                model.Add(sum(deliver[(k,t,d,p,s)] 
                             for k in range(NbVehicles) 
                             for t in range(MAX_TRIPS) 
                             for d in range(NbDepots)) == stations[s]['dem'][p])
    
    # 4. Depot stock
    for d in range(NbDepots):
        for p in range(NbProducts):
            model.Add(sum(load[(k,t,d,p)] 
                         for k in range(NbVehicles) 
                         for t in range(MAX_TRIPS)) <= depots[d]['stock'][p])
    
    # 5. Vehicle capacity
    for k, v in enumerate(vehicles):
        for t in range(MAX_TRIPS):
            model.Add(sum(load[(k,t,d,p)] 
                         for d in range(NbDepots) 
                         for p in range(NbProducts)) <= v['cap'])
    
    # 6. Trip ordering
    for k in range(NbVehicles):
        for t in range(MAX_TRIPS - 1):
            trip_t_used = model.NewBoolVar(f"trip_used_{k}_{t}")
            trip_t1_used = model.NewBoolVar(f"trip_used_{k}_{t+1}")
            
            model.Add(sum(trip_active[(k,t,d,p)] 
                         for d in range(NbDepots) 
                         for p in range(NbProducts)) >= 1).OnlyEnforceIf(trip_t_used)
            model.Add(sum(trip_active[(k,t,d,p)] 
                         for d in range(NbDepots) 
                         for p in range(NbProducts)) == 0).OnlyEnforceIf(trip_t_used.Not())
            
            model.Add(sum(trip_active[(k,t+1,d,p)] 
                         for d in range(NbDepots) 
                         for p in range(NbProducts)) >= 1).OnlyEnforceIf(trip_t1_used)
            
            model.AddImplication(trip_t1_used, trip_t_used)
    
    # Objective
    obj_terms = []
    switch_cost_int = [[int(c * COST_MULTIPLIER) for c in row] for row in switch_cost]
    
    for k, v in enumerate(vehicles):
        g_coord = garages[v['gar_id']]['coord']
        
        for t in range(MAX_TRIPS):
            for d in range(NbDepots):
                dep_coord = depots[d]['coord']
                base_dist = int(euclidean(g_coord, dep_coord) * 2 * COST_MULTIPLIER)
                
                for p in range(NbProducts):
                    # Distance cost
                    trip_dist = base_dist
                    for s in range(NbStations):
                        st_coord = stations[s]['coord']
                        st_dist = int(euclidean(dep_coord, st_coord) * COST_MULTIPLIER)
                        
                        del_happens = model.NewBoolVar(f"del_h_{k}_{t}_{d}_{p}_{s}")
                        model.Add(deliver[(k,t,d,p,s)] >= 1).OnlyEnforceIf(del_happens)
                        model.Add(deliver[(k,t,d,p,s)] == 0).OnlyEnforceIf(del_happens.Not())
                        
                        obj_terms.append(del_happens * st_dist)
                    
                    obj_terms.append(trip_active[(k,t,d,p)] * trip_dist)
                    
                    # Changeover cost (simplified)
                    if t == 0 and p != v['prod_id']:
                        obj_terms.append(trip_active[(k,t,d,p)] * switch_cost_int[v['prod_id']][p])
    
    model.Minimize(sum(obj_terms))
    
    # Solve
    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = float(time_limit)
    if not verbose:
        solver.parameters.log_search_progress = False
    
    if verbose:
        print("\n   Solving...")
    status = solver.Solve(model)
    
    if status == cp_model.INFEASIBLE:
        print(" Problem is INFEASIBLE")
        return None, None
    elif status == cp_model.MODEL_INVALID:
        print(" Model is INVALID")
        return None, None
    
    if verbose:
        print(f"   Status: {solver.StatusName(status)}")
        print(f"   Objective: {solver.ObjectiveValue() / COST_MULTIPLIER:.2f}")
    
    # Extract solution
    routes = extract_cpsat_solution(solver, instance, trip_active, load, deliver, 
                                    MAX_TRIPS)
    
    metrics = calculate_metrics(routes, instance, start_time)
    
    if verbose:
        print(f"\n✓ Solution found in {metrics['cpu_time']:.3f}s")
        print(f"   Vehicles used: {metrics['used_vehicles']}/{NbVehicles}")
    
    return routes, metrics


def extract_cpsat_solution(solver, instance, trip_active, load, deliver, MAX_TRIPS):
    """Extract solution from CP-SAT solver"""
    vehicles = instance['vehicles']
    garages = instance['garages']
    NbVehicles = instance['NbVehicles']
    NbDepots = instance['NbDepots']
    NbProducts = instance['NbProducts']
    NbStations = instance['NbStations']
    
    routes = []
    
    for k, v in enumerate(vehicles):
        vehicle_trips = []
        
        for t in range(MAX_TRIPS):
            for d in range(NbDepots):
                for p in range(NbProducts):
                    if solver.Value(trip_active[(k,t,d,p)]):
                        loaded_qty = solver.Value(load[(k,t,d,p)])
                        
                        deliveries = []
                        for s in range(NbStations):
                            qty = solver.Value(deliver[(k,t,d,p,s)])
                            if qty > 0:
                                deliveries.append((s, qty))
                        
                        vehicle_trips.append({
                            'depot_idx': d,
                            'product_idx': p,
                            'load': loaded_qty,
                            'deliveries': deliveries
                        })
        
        if vehicle_trips:
            routes.append({
                'vehicle_id': v['id'],
                'vehicle_idx': k,
                'garage_id': garages[v['gar_id']]['id'],
                'initial_product': v['prod_id'],
                'trips': vehicle_trips
            })
    
    return routes


# ============================================================================
# METRICS CALCULATION
# ============================================================================

def calculate_metrics(routes, instance, start_time):
    """Calculate solution metrics"""
    vehicles = instance['vehicles']
    garages = instance['garages']
    depots = instance['depots']
    stations = instance['stations']
    switch_cost = instance['switch_cost']
    
    total_distance = 0.0
    total_changes = 0
    total_switch_cost = 0.0
    
    for route in routes:
        v = vehicles[route['vehicle_idx']]
        g_coord = garages[v['gar_id']]['coord']
        last_product = route['initial_product']
        
        for trip in route['trips']:
            # Check for product change
            if trip['product_idx'] != last_product:
                total_changes += 1
                total_switch_cost += switch_cost[last_product][trip['product_idx']]
            last_product = trip['product_idx']
            
            # Calculate distance for this trip
            dep_coord = depots[trip['depot_idx']]['coord']
            
            # Garage to depot
            total_distance += euclidean(g_coord, dep_coord)
            
            # Depot to stations
            current_pos = dep_coord
            for s_idx, qty in trip['deliveries']:
                st_coord = stations[s_idx]['coord']
                total_distance += euclidean(current_pos, st_coord)
                current_pos = st_coord
            
            # Last position back to garage
            total_distance += euclidean(current_pos, g_coord)
    
    metrics = {
        'used_vehicles': len(routes),
        'total_changes': total_changes,
        'total_switch_cost': total_switch_cost,
        'distance_total': total_distance,
        'processor': 'Intel Core i7-10700K',
        'cpu_time': time.time() - start_time
    }
    
    return metrics


# ============================================================================
# SOLUTION VALIDATION
# ============================================================================

def validate_solution(routes, instance, verbose=True):
    """Validate solution against all constraints"""
    if verbose:
        print_section("VALIDATING SOLUTION")
    
    errors = []
    warnings = []
    
    vehicles = instance['vehicles']
    depots = instance['depots']
    garages = instance['garages']
    stations = instance['stations']
    NbProducts = instance['NbProducts']
    
    # Track deliveries
    deliveries = {}
    for s in stations:
        deliveries[s['id']] = [0] * NbProducts
    
    # Track depot usage
    depot_usage = {}
    for d in depots:
        depot_usage[d['id']] = [0] * NbProducts
    
    for route in routes:
        vehicle = vehicles[route['vehicle_idx']]
        
        for trip in route['trips']:
            depot_id = depots[trip['depot_idx']]['id']
            product_idx = trip['product_idx']
            
            # Check capacity
            if trip['load'] > vehicle['cap']:
                errors.append(f"Vehicle {route['vehicle_id']}: Load {trip['load']} exceeds capacity {vehicle['cap']}")
            
            # Track depot usage
            depot_usage[depot_id][product_idx] += trip['load']
            
            # Track deliveries
            total_delivered = 0
            for s_idx, qty in trip['deliveries']:
                deliveries[stations[s_idx]['id']][product_idx] += qty
                total_delivered += qty
            
            # Check load = deliveries
            if total_delivered != trip['load']:
                errors.append(f"Vehicle {route['vehicle_id']}: Load {trip['load']} != Deliveries {total_delivered}")
    
    # Check demand satisfaction
    for station in stations:
        for p_idx in range(NbProducts):
            demand = station['dem'][p_idx]
            delivered = deliveries[station['id']][p_idx]
            
            if delivered != demand:
                errors.append(f"Station {station['id']}, Product {p_idx+1}: Demand {demand}, Delivered {delivered}")
    
    # Check depot stock
    for depot in depots:
        for p_idx in range(NbProducts):
            stock = depot['stock'][p_idx]
            used = depot_usage[depot['id']][p_idx]
            
            if used > stock:
                errors.append(f"Depot {depot['id']}, Product {p_idx+1}: Stock {stock}, Used {used}")
    
    if verbose:
        if errors:
            print(f"\n Found {len(errors)} constraint violations:")
            for i, error in enumerate(errors[:10], 1):
                print(f"   {i}. {error}")
            if len(errors) > 10:
                print(f"   ... and {len(errors)-10} more")
        else:
            print("\n✓ All constraints satisfied")
        
        if warnings:
            print(f"\n⚠ Found {len(warnings)} warnings:")
            for warning in warnings:
                print(f"   - {warning}")
    
    return len(errors) == 0, errors, warnings


# ============================================================================
# SOLUTION EXPORT
# ============================================================================

def export_solution(routes, metrics, instance, filename):
    """Export solution in the required format"""
    depots = instance['depots']
    stations = instance['stations']
    switch_cost = instance['switch_cost']
    
    if not routes:
        print(" No routes to export")
        return
    
    with open(filename, 'w') as f:
        for route in routes:
            # Build visit sequence line
            visit_line = f"{route['vehicle_id']}: {route['garage_id']}"
            product_line = f"{route['vehicle_id']}:"
            
            cumulative_cost = 0.0
            last_product = route['initial_product']
            
            for trip_idx, trip in enumerate(route['trips']):
                depot_id = depots[trip['depot_idx']]['id']
                product_id = trip['product_idx']
                
                # Add changeover cost if product changed
                if product_id != last_product:
                    cumulative_cost += switch_cost[last_product][product_id]
                last_product = product_id
                
                # Add depot with load
                visit_line += f" - {depot_id} [{trip['load']}]"
                # First element doesn't have leading " - "
                if trip_idx == 0:
                    product_line += f" {product_id + 1}({cumulative_cost:.1f})"
                else:
                    product_line += f" - {product_id + 1}({cumulative_cost:.1f})"
                
                # Add each delivery
                for s_idx, qty in trip['deliveries']:
                    station_id = stations[s_idx]['id']
                    visit_line += f" - {station_id} ({qty})"
                    product_line += f" - {product_id + 1}({cumulative_cost:.1f})"
            
            # Return to garage  
            visit_line += f" - {route['garage_id']}"
            product_line += f" - {last_product + 1}({cumulative_cost:.1f})"
            
            f.write(f"{visit_line}\n")
            f.write(f"{product_line}\n\n")
        
        # Write metrics
        f.write(f"{metrics['used_vehicles']}\n")
        f.write(f"{metrics['total_changes']}\n")
        f.write(f"{metrics['total_switch_cost']:.2f}\n")
        f.write(f"{metrics['distance_total']:.2f}\n")
        f.write(f"{metrics['processor']}\n")
        f.write(f"{metrics['cpu_time']:.3f}\n")
    
    print(f"\n Solution saved to: {filename}")


def print_solution_summary(routes, metrics, instance):
    """Print a nice summary of the solution"""
    print_section("SOLUTION SUMMARY")
    
    print(f"\n Performance Metrics:")
    print(f"   Vehicles used: {metrics['used_vehicles']}/{instance['NbVehicles']}")
    print(f"   Total distance: {metrics['distance_total']:.2f}")
    print(f"   Product changes: {metrics['total_changes']}")
    print(f"   Changeover cost: {metrics['total_switch_cost']:.2f}")
    print(f"   Total cost: {metrics['distance_total'] + metrics['total_switch_cost']:.2f}")
    print(f"   CPU time: {metrics['cpu_time']:.3f}s")
    
    print(f"\n Route Details:")
    for route in routes:
        print(f"\n   Vehicle {route['vehicle_id']} (Garage {route['garage_id']}):")
        print(f"      Trips: {len(route['trips'])}")
        total_load = sum(trip['load'] for trip in route['trips'])
        total_deliveries = sum(len(trip['deliveries']) for trip in route['trips'])
        print(f"      Total loaded: {total_load} units")
        print(f"      Total deliveries: {total_deliveries} stops")


# ============================================================================
# MAIN
# ============================================================================

def main():
    parser = argparse.ArgumentParser(
        description='MPVRP-CC Solver - Multi-Product Vehicle Routing with Changeover Cost',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python mpvrp_complete.py instance.dat                    # Use greedy (fast)
  python mpvrp_complete.py instance.dat -a cpsat           # Use CP-SAT
  python mpvrp_complete.py instance.dat -o solution.dat    # Specify output
  python mpvrp_complete.py instance.dat -a cpsat -t 600    # CP-SAT with 10min limit
  python mpvrp_complete.py instance.dat --no-validate      # Skip validation
        """
    )
    
    parser.add_argument('instance', help='Instance file (.dat)')
    parser.add_argument('-o', '--output', default='solution.dat', 
                       help='Output solution file (default: solution.dat)')
    parser.add_argument('-a', '--algorithm', choices=['greedy', 'cpsat', 'both'], 
                       default='greedy',
                       help='Algorithm to use (default: greedy)')
    parser.add_argument('-t', '--time-limit', type=int, default=300,
                       help='Time limit for CP-SAT in seconds (default: 300)')
    parser.add_argument('--no-validate', action='store_true',
                       help='Skip solution validation')
    parser.add_argument('-q', '--quiet', action='store_true',
                       help='Minimal output')
    
    args = parser.parse_args()
    
    verbose = not args.quiet
    
    if verbose:
        print("=" * 70)
        print("  MPVRP-CC COMPLETE SOLVER")
        print("  Multi-Product Vehicle Routing Problem with Changeover Cost")
        print("=" * 70)
    
    # Read instance
    instance = read_instance(args.instance)
    
    best_routes = None
    best_metrics = None
    best_algorithm = None
    
    # Solve with selected algorithm(s)
    if args.algorithm in ['greedy', 'both']:
        routes, metrics = solve_greedy(instance, verbose=verbose)
        if routes:
            best_routes = routes
            best_metrics = metrics
            best_algorithm = 'Greedy'
    
    if args.algorithm in ['cpsat', 'both']:
        if not ORTOOLS_AVAILABLE:
            print("\n⚠ CP-SAT solver not available (OR-Tools not installed)")
            if args.algorithm == 'cpsat':
                print("   Please install: pip install ortools")
                sys.exit(1)
        else:
            routes, metrics = solve_cpsat(instance, time_limit=args.time_limit, 
                                         verbose=verbose)
            if routes:
                if best_metrics is None or (metrics['distance_total'] + metrics['total_switch_cost'] < 
                                            best_metrics['distance_total'] + best_metrics['total_switch_cost']):
                    best_routes = routes
                    best_metrics = metrics
                    best_algorithm = 'CP-SAT'
    
    if best_routes is None:
        print("\n No solution found")
        sys.exit(1)
    
    if args.algorithm == 'both' and verbose:
        print(f"\n Best solution from: {best_algorithm}")
    
    # Validate
    if not args.no_validate:
        is_valid, errors, warnings = validate_solution(best_routes, instance, verbose=verbose)
        if not is_valid:
            print("\n⚠ WARNING: Solution has constraint violations!")
    
    # Print summary
    if verbose:
        print_solution_summary(best_routes, best_metrics, instance)
    
    # Export
    export_solution(best_routes, best_metrics, instance, args.output)
    
    if verbose:
        print("\n" + "=" * 70)
        print("✓ Done!")
        print("=" * 70)
    
    return 0


if __name__ == "__main__":
    sys.exit(main())