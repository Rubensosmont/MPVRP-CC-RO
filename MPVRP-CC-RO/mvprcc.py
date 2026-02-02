"""
MPVRP-CC Solver - Version finale 

"""

import sys
import os
import math
import time
import platform
from dataclasses import dataclass
from typing import List, Dict, Tuple, Optional
from itertools import permutations


@dataclass
class Vehicle:
    id: int
    capacity: int
    home_garage: int
    initial_product: int  


@dataclass
class Depot:
    id: int
    x: float
    y: float
    stocks: List[int]


@dataclass
class Garage:
    id: int
    x: float
    y: float


@dataclass
class Station:
    id: int
    x: float
    y: float
    demands: List[int]


class MPVRPInstance:
    def __init__(self, filename: str):
        self.filename = filename
        self.uuid = ""
        self.nb_products = 0
        self.nb_depots = 0
        self.nb_garages = 0
        self.nb_stations = 0
        self.nb_vehicles = 0
        self.transition_costs = []
        self.vehicles = []
        self.depots = []
        self.garages = []
        self.stations = []
        self.load_instance(filename)
    
    def load_instance(self, filename: str):
        with open(filename, 'r') as f:
            lines = [line.strip() for line in f.readlines() if line.strip()]
        
        idx = 0
        self.uuid = lines[idx].replace('#', '').strip()
        idx += 1
        
        params = list(map(int, lines[idx].split()))
        self.nb_products, self.nb_depots, self.nb_garages, self.nb_stations, self.nb_vehicles = params
        idx += 1
        
        self.transition_costs = []
        for i in range(self.nb_products):
            costs = list(map(float, lines[idx].split()))
            self.transition_costs.append(costs)
            idx += 1
        
        self.vehicles = []
        for i in range(self.nb_vehicles):
            parts = list(map(int, lines[idx].split()))
            self.vehicles.append(Vehicle(parts[0], parts[1], parts[2], parts[3]))
            idx += 1
        
        self.depots = []
        for i in range(self.nb_depots):
            parts = lines[idx].split()
            depot_id = int(parts[0])
            x, y = float(parts[1]), float(parts[2])
            stocks = list(map(int, parts[3:]))
            self.depots.append(Depot(depot_id, x, y, stocks))
            idx += 1
        
        self.garages = []
        for i in range(self.nb_garages):
            parts = lines[idx].split()
            self.garages.append(Garage(int(parts[0]), float(parts[1]), float(parts[2])))
            idx += 1
        
        self.stations = []
        for i in range(self.nb_stations):
            parts = lines[idx].split()
            station_id = int(parts[0])
            x, y = float(parts[1]), float(parts[2])
            demands = list(map(int, parts[3:]))
            self.stations.append(Station(station_id, x, y, demands))
            idx += 1
    
    def euclidean_distance(self, x1: float, y1: float, x2: float, y2: float) -> float:
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)


class CorrectedMPVRPSolver:
    def __init__(self, instance: MPVRPInstance):
        self.instance = instance
        self.solution = None
        # Stocks globaux partagés entre tous les véhicules
        self.global_depot_stocks = {}
        for depot in instance.depots:
            for p_idx, stock in enumerate(depot.stocks):
                self.global_depot_stocks[(depot.id, p_idx)] = stock
    
    def solve(self) -> Dict:
        start_time = time.time()
        
        remaining_demands = {}
        for station in self.instance.stations:
            for p_idx, demand in enumerate(station.demands):
                if demand > 0:
                    key = (station.id - 1, p_idx)
                    remaining_demands[key] = demand
        
        all_routes = []
        
        for vehicle in self.instance.vehicles:
            if not remaining_demands:
                break
                
            route = self.build_vehicle_full_route(vehicle, remaining_demands)
            
            if route and route['segments']:
                all_routes.append(route)
                
                # Déduire des stocks GLOBAUX
                for segment in route['segments']:
                    depot_id = segment['depot']
                    product_idx = segment['product']
                    load = segment['load']
                    
                    stock_key = (depot_id, product_idx)
                    if stock_key in self.global_depot_stocks:
                        self.global_depot_stocks[stock_key] -= load
                
                # Déduire des demandes
                for segment in route['segments']:
                    for station_idx, product_idx, quantity in segment['deliveries']:
                        key = (station_idx, product_idx)
                        if key in remaining_demands:
                            remaining_demands[key] -= quantity
                            if remaining_demands[key] <= 0:
                                del remaining_demands[key]
        
        total_distance = sum(r['total_distance'] for r in all_routes)
        total_changeover = sum(r['total_changeover_cost'] for r in all_routes)
        nb_changes = sum(r['nb_changes'] for r in all_routes)
        
        elapsed_time = time.time() - start_time
        
        self.solution = {
            'routes': all_routes,
            'nb_vehicles': len(all_routes),
            'nb_changes': nb_changes,
            'total_changeover_cost': total_changeover,
            'total_distance': total_distance,
            'processor': platform.processor() or 'Unknown',
            'time': elapsed_time,
            'remaining_demands': len(remaining_demands)
        }
        
        return self.solution
    
    def build_vehicle_full_route(self, vehicle: Vehicle, remaining_demands: Dict) -> Dict:
        garage = self.instance.garages[vehicle.home_garage - 1]
        
        
        current_product = 0  # Produit 0 = état vide/neutre
        
        segments = []
        total_distance = 0.0
        total_changeover_cost = 0.0
        nb_changes = 0
        current_pos = (garage.x, garage.y)
        
        max_segments = 30
        
        # Copie locale des demandes pour ce véhicule
        local_remaining = remaining_demands.copy()
        # Copie locale des stocks pour simulation
        local_stocks = self.global_depot_stocks.copy()
        
        for iteration in range(max_segments):
            if not local_remaining:
                break
            
            best_segment = None
            best_cost = float('inf')
            best_product = None
            best_depot = None
            
            for depot in self.instance.depots:
                for product_idx in range(self.instance.nb_products):
                    stock_key = (depot.id, product_idx)
                    available_stock = local_stocks.get(stock_key, 0)
                    
                    if available_stock <= 0:
                        continue
                    
                    segment = self.build_segment(
                        current_pos, depot, product_idx, vehicle.capacity,
                        current_product, local_remaining, available_stock
                    )
                    
                    if segment and segment['total_cost'] < best_cost:
                        best_cost = segment['total_cost']
                        best_segment = segment
                        best_product = product_idx
                        best_depot = depot
            
            if best_segment is None:
                break
            
            segments.append(best_segment)
            total_distance += best_segment['distance']
            total_changeover_cost += best_segment['changeover_cost']
            
            # Compter un changement seulement quand le produit CHANGE
            if current_product != best_product:
                nb_changes += 1
            
            # Mettre à jour les demandes LOCALES
            for station_idx, product_idx, quantity in best_segment['deliveries']:
                key = (station_idx, product_idx)
                if key in local_remaining:
                    local_remaining[key] -= quantity
                    if local_remaining[key] <= 0:
                        del local_remaining[key]
            
            # Mettre à jour les stocks LOCAUX
            stock_key = (best_segment['depot'], best_segment['product'])
            if stock_key in local_stocks:
                local_stocks[stock_key] -= best_segment['load']
            
            last_station = self.instance.stations[best_segment['deliveries'][-1][0]]
            current_pos = (last_station.x, last_station.y)
            current_product = best_product
        
        # Retour au garage
        if segments:
            last_station = self.instance.stations[segments[-1]['deliveries'][-1][0]]
            distance_to_garage = self.instance.euclidean_distance(
                last_station.x, last_station.y, garage.x, garage.y
            )
            total_distance += distance_to_garage
        
        return {
            'vehicle_id': vehicle.id,
            'garage': garage.id,
            'segments': segments,
            'total_distance': total_distance,
            'total_changeover_cost': total_changeover_cost,
            'nb_changes': nb_changes
        }
    
    def build_segment(self, current_pos: Tuple[float, float], depot: Depot,
                     product_idx: int, capacity: int, current_product: Optional[int],
                     remaining_demands: Dict, available_stock: int) -> Optional[Dict]:
        
        # Stations demandant ce produit
        available_demands = [
            (s_idx, p_idx, qty)
            for (s_idx, p_idx), qty in remaining_demands.items()
            if p_idx == product_idx and qty > 0
        ]
        
        if not available_demands:
            return None
        
        # Sélection des stations à servir
        stations_to_serve = []
        total_load = 0
        max_load = min(capacity, available_stock)
        
        if max_load <= 0:
            return None
        
        # Trier par distance au dépôt
        sorted_demands = sorted(
            available_demands,
            key=lambda x: self.instance.euclidean_distance(
                depot.x, depot.y,
                self.instance.stations[x[0]].x,
                self.instance.stations[x[0]].y
            )
        )
        
        for station_idx, _, demand in sorted_demands:
            if total_load >= max_load:
                break
            
            to_load = min(demand, max_load - total_load)
            if to_load > 0:
                stations_to_serve.append((station_idx, product_idx, to_load))
                total_load += to_load
        
        if not stations_to_serve or total_load == 0:
            return None
        
        # Optimiser l'ordre de visite
        station_objs = [self.instance.stations[s[0]] for s in stations_to_serve]
        ordered_stations = self.optimize_station_sequence(depot, station_objs)
        
        ordered_deliveries = []
        for station in ordered_stations:
            matching = next((s for s in stations_to_serve if s[0] == station.id - 1), None)
            if matching:
                ordered_deliveries.append(matching)
        
        # Calcul de la distance
        distance = self.instance.euclidean_distance(current_pos[0], current_pos[1], depot.x, depot.y)
        
        cx, cy = depot.x, depot.y
        for station in ordered_stations:
            distance += self.instance.euclidean_distance(cx, cy, station.x, station.y)
            cx, cy = station.x, station.y
        
        # Calcul du coût de changement
        changeover_cost = 0.0
        
        # Le véhicule commence avec produit 0 (neutre/vide)
        # Tout changement (y compris 0→autre) a un coût
        if current_product != product_idx:
            changeover_cost = self.instance.transition_costs[current_product][product_idx]
        
        return {
            'depot': depot.id,
            'product': product_idx,
            'load': total_load,
            'deliveries': ordered_deliveries,
            'distance': distance,
            'changeover_cost': changeover_cost,
            'total_cost': distance + changeover_cost
        }
    
    def optimize_station_sequence(self, depot: Depot, stations: List[Station]) -> List[Station]:
        if len(stations) <= 1:
            return stations
        
        if len(stations) <= 7:
            best_order = stations
            best_dist = self.calculate_sequence_distance(depot, best_order)
            
            for perm in permutations(stations):
                dist = self.calculate_sequence_distance(depot, list(perm))
                if dist < best_dist:
                    best_dist = dist
                    best_order = list(perm)
            
            return best_order
        
        # Nearest neighbor pour grandes séquences
        unvisited = stations.copy()
        ordered = []
        cx, cy = depot.x, depot.y
        
        while unvisited:
            nearest = min(unvisited, key=lambda s: self.instance.euclidean_distance(cx, cy, s.x, s.y))
            ordered.append(nearest)
            cx, cy = nearest.x, nearest.y
            unvisited.remove(nearest)
        
        return ordered
    
    def calculate_sequence_distance(self, depot: Depot, stations: List[Station]) -> float:
        dist = self.instance.euclidean_distance(depot.x, depot.y, stations[0].x, stations[0].y)
        
        for i in range(len(stations) - 1):
            dist += self.instance.euclidean_distance(
                stations[i].x, stations[i].y,
                stations[i+1].x, stations[i+1].y
            )
        
        return dist
    
    def write_solution(self, output_filename: str):
        if not self.solution:
            print("Aucune solution a ecrire")
            return
        
        with open(output_filename, 'w') as f:
            for route in self.solution['routes']:
                vehicle = next(v for v in self.instance.vehicles if v.id == route['vehicle_id'])
                
                # IMPORTANT: Véhicule part VIDE, donc produit initial = 0 (vide)
                visit_elements = [str(route['garage'])]
                product_elements = ["0(0.0)"]  # Vide au départ
                
                cumulative_cost = 0.0
                
                for segment in route['segments']:
                    cumulative_cost += segment['changeover_cost']
                    product = segment['product']
                    
                    visit_elements.append(f"{segment['depot']} [{segment['load']}]")
                    product_elements.append(f"{product}({cumulative_cost:.1f})")
                    
                    for station_idx, _, quantity in segment['deliveries']:
                        station_id = station_idx + 1
                        visit_elements.append(f"{station_id} ({quantity})")
                        product_elements.append(f"{product}({cumulative_cost:.1f})")
                
                visit_elements.append(str(route['garage']))
                final_product = route['segments'][-1]['product']
                product_elements.append(f"{final_product}({cumulative_cost:.1f})")
                
                visit_line = f"{route['vehicle_id']}: " + " - ".join(visit_elements)
                product_line = f"{route['vehicle_id']}: " + " - ".join(product_elements)
                
                f.write(visit_line + '\n')
                f.write(product_line + '\n')
                f.write('\n')
            
            f.write(f"{self.solution['nb_vehicles']}\n")
            f.write(f"{self.solution['nb_changes']}\n")
            f.write(f"{self.solution['total_changeover_cost']:.2f}\n")
            f.write(f"{self.solution['total_distance']:.2f}\n")
            f.write(f"{self.solution['processor']}\n")
            f.write(f"{self.solution['time']:.3f}\n")
        
        print(f"Solution ecrite: {output_filename}")
    
    def print_solution_summary(self):
        if not self.solution:
            return
        
        print("\n" + "="*70)
        print("RESUME DE LA SOLUTION MPVRP-CC")
        print("="*70)
        
        print(f"\nVehicules utilises: {self.solution['nb_vehicles']}")
        print(f"Changements de produit: {self.solution['nb_changes']}")
        print(f"Cout changement: {self.solution['total_changeover_cost']:.2f}")
        print(f"Distance totale: {self.solution['total_distance']:.2f}")
        print(f"Temps: {self.solution['time']:.3f}s")
        print(f"\nCOUT TOTAL: {self.solution['total_distance'] + self.solution['total_changeover_cost']:.2f}")
        
        if self.solution['remaining_demands'] > 0:
            print(f"\nATTENTION: {self.solution['remaining_demands']} demandes non satisfaites!")
        
        print("="*70)


def main():
    if len(sys.argv) < 2:
        print("Usage: python mpvrp_solver.py <instance_file.dat>")
        print("\nExemple:")
        print("  python mpvrp_solver.py MPVRP_S_001_s9_d1_p2.dat")
        sys.exit(1)
    
    instance_file = sys.argv[1]
    
    if not os.path.exists(instance_file):
        print(f"Erreur: Fichier '{instance_file}' introuvable")
        sys.exit(1)
    
    base_name = os.path.basename(instance_file)
    output_file = 'Sol_' + base_name if base_name.startswith('MPVRP_') else 'Sol_' + base_name
    
    print(f"\n{'='*70}")
    print(f"Resolution de: {instance_file}")
    print(f"{'='*70}")
    
    try:
        instance = MPVRPInstance(instance_file)
        
        print(f"Instance chargee:")
        print(f"  {instance.nb_vehicles} vehicules")
        print(f"  {instance.nb_products} produits")
        print(f"  {instance.nb_stations} stations")
        print(f"  {instance.nb_depots} depots")
        
        # Afficher les stocks initiaux
        print(f"\nStocks des depots:")
        for depot in instance.depots:
            stocks_str = ", ".join([f"P{i}={s}" for i, s in enumerate(depot.stocks)])
            print(f"  Depot {depot.id}: {stocks_str}")
        
        solver = CorrectedMPVRPSolver(instance)
        solution = solver.solve()
        
        solver.print_solution_summary()
        solver.write_solution(output_file)
        
        # Vérification détaillée
        total_delivered = {}
        total_loaded = {}
        
        for route in solution['routes']:
            for segment in route['segments']:
                product = segment['product']
                depot = segment['depot']
                load = segment['load']
                
                # Comptabiliser ce qui a été chargé
                key_loaded = (depot, product)
                if key_loaded not in total_loaded:
                    total_loaded[key_loaded] = 0
                total_loaded[key_loaded] += load
                
                # Comptabiliser ce qui a été livré
                if product not in total_delivered:
                    total_delivered[product] = 0
                for _, _, quantity in segment['deliveries']:
                    total_delivered[product] += quantity
        
        total_demand = {p: sum(s.demands[p] for s in instance.stations) 
                       for p in range(instance.nb_products)}
        
        print("\nVerification des demandes:")
        all_satisfied = True
        for p in range(instance.nb_products):
            delivered = total_delivered.get(p, 0)
            demand = total_demand[p]
            if delivered == demand:
                print(f"  Produit {p}: OK ({delivered}/{demand})")
            else:
                print(f"  Produit {p}: ERREUR ({delivered}/{demand})")
                all_satisfied = False
        
        print("\nVerification des stocks:")
        stock_ok = True
        for depot in instance.depots:
            for p in range(instance.nb_products):
                key = (depot.id, p)
                loaded = total_loaded.get(key, 0)
                available = depot.stocks[p]
                if loaded <= available:
                    print(f"  Depot {depot.id} Produit {p}: OK ({loaded}/{available})")
                else:
                    print(f"  Depot {depot.id} Produit {p}: DEPASSEMENT! ({loaded}/{available})")
                    stock_ok = False
        
        if all_satisfied and stock_ok:
            print("\n" + "="*70)
            print("SOLUTION VALIDE - Toutes contraintes respectees")
            print("="*70)
        else:
            print("\n" + "="*70)
            print("ATTENTION - Contraintes violees")
            print("="*70)
        
    except Exception as e:
        print(f"\nErreur: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()