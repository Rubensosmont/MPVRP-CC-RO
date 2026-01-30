import sys
import math
import time
import platform
import os
from ortools.linear_solver import pywraplp

# =========================================================
# 1. Lecture des instances MPVRP-CC (inchangé)
# =========================================================

def read_instance(filepath):
    # Note: Cette fonction est utilisée pour lire l'instance,
    # nous devons nous assurer qu'elle est appelée correctement dans le main.
    with open(filepath, "r", encoding='utf-8') as f:
        lines = [l.strip() for l in f if l.strip() and not l.startswith("#")]

    idx = 0
    Pn, Dn, Gn, Sn, Kn = map(int, lines[idx].split())
    idx += 1
    CC = []
    for _ in range(Pn):
        CC.append(list(map(float, lines[idx].split())))
        idx += 1
    vehicles = []
    Q = {}
    vehicle_garage = {}
    for _ in range(Kn):
        v_id, cap, g, initial_p = map(int, lines[idx].split())
        vehicles.append(v_id)
        Q[v_id] = cap
        vehicle_garage[v_id] = g
        idx += 1

    depots = {}
    for d in range(1, Dn + 1):
        parts = list(map(float, lines[idx].split()))
        depot_id, x, y = int(parts), parts, parts
        depots[depot_id] = (x, y)
        idx += 1

    garages = {}
    for g in range(1, Gn + 1):
        parts = list(map(float, lines[idx].split()))
        garage_id, x, y = int(parts), parts, parts
        garages[garage_id] = (x, y)
        idx += 1

    stations = {}
    demand_per_product = {}
    for s in range(1, Sn + 1):
        parts = list(map(float, lines[idx].split()))
        sid, x, y = int(parts), parts, parts
        stations[sid] = (x, y)
        for p_idx in range(Pn):
            demand_per_product[(sid, p_idx)] = parts[3 + p_idx]
        idx += 1

    return {
        "P_count": Pn, "D": list(depots.keys()), "G": list(garages.keys()), "S": list(stations.keys()),
        "K": vehicles, "coords": {**depots, **garages, **stations}, "Q": Q,
        "garage_of_vehicle": vehicle_garage, "demand_per_product": demand_per_product, "CC": CC
    }

# =========================================================
# 2. Distance euclidienne (inchangé)
# =========================================================

def dist(i, j, coords):
    xi, yi = coords[i]
    xj, yj = coords[j]
    return math.hypot(xi - xj, yi - j)

# =========================================================
# 3. Modèle PLNE/MILP et formatage de sortie
# =========================================================

def build_and_solve_milp(instance_filepath_str, data):
    solver = pywraplp.Solver.CreateSolver("SCIP")
    if solver is None:
        print("❌ Solveur SCIP non disponible")
        return

    start_time = time.time()
    
    P, D, G, S, K = range(data["P_count"]), data["D"], data["G"], data["S"], data["K"]
    N = D + S + G # Ensemble de tous les noeuds
    coords = data["coords"]
    Q = data["Q"]
    demand = data["demand_per_product"]
    CC = data["CC"]

    # -----------------------
    # Variables de Décision
    # -----------------------

    x = {} # x[i, j, k]: 1 si véhicule k va de i à j
    f = {} # f[i, j, k, p]: Quantité de produit p sur l'arc (i, j) par véhicule k
    # NOTE: Pour simplifier l'implémentation MILP, nous n'ajouterons pas directement
    # les variables de changement de produit binaires, mais nous nous concentrerons
    # sur le flux pour dériver les coûts.

    for k in K:
        for i in N:
            for j in N:
                if i != j:
                    x[i, j, k] = solver.BoolVar(f"x_{i}_{j}_{k}")
                    for p in P:
                         # Utilisation d'un flow variable bound
                        f[i, j, k, p] = solver.NumVar(0, Q[k], f"f_{i}_{j}_{k}_{p}")


    # -----------------------
    # Fonction Objectif
    # -----------------------
    # Minimiser les coûts de transport (nous ignorons les coûts de changement
    # pour le moment car ils nécessitent une modélisation d'état complexe)

    objective = solver.Objective()
    for k in K:
         for i in N:
             for j in N:
                 if i != j:
                    objective.SetCoefficient(x[i, j, k], dist(i, j, coords))

    objective.SetMinimization()

    # -----------------------
    # Contraintes
    # -----------------------

    # 1. Conservation du flux véhicule pour les dépôts et stations
    for k in K:
        for j in D + S:
            solver.Add(
                sum(x[i, j, k] for i in N if i != j) ==
                sum(x[j, l, k] for l in N if l != j)
            )

    # 2. Départ / retour garage (chaque véhicule doit partir et revenir à son garage)
    for k in K:
        g = data["garage_of_vehicle"][k]
        solver.Add(sum(x[g, j, k] for j in N if j != g) == 1)
        solver.Add(sum(x[i, g, k] for i in N if i != g) == 1)

    # 3. Satisfaction de la demande (le flux net au niveau de la station est la demande)
    for k in K:
        for s in S:
            for p in P:
                if (s, p) in demand and demand[s, p] > 0:
                    solver.Add(
                        sum(f[i, s, k, p] for i in N if i != s) - 
                        sum(f[s, j, k, p] for j in N if j != s) == demand[s, p]
                    )

    # 4. Capacité (le flux total sur un arc ne dépasse pas la capacité du véhicule si l'arc est utilisé)
    for k in K:
        for i in N:
            for j in N:
                if i != j:
                    solver.Add(sum(f[i, j, k, p] for p in P) <= Q[k] * x[i, j, k])

    # 5. Contrainte de produit unique par "segment de route" (Complexe en MILP standard, omis pour cette première tentative)
    
    # 6. Élimination des sous-tours (Très difficile à implémenter pour le MPVRP-CC sans variables d'état supplémentaires)

    # -----------------------
    # Résolution
    # -----------------------

    status = solver.Solve()
    end_time = time.time()

    if status == pywraplp.Solver.OPTIMAL or status == pywraplp.Solver.FEASIBLE:
        print(f"✅ Solution trouvée pour l'instance {os.path.basename(instance_filepath_str)}.")
        
        # NOTE: La fonction de formatage doit être réécrite pour extraire les routes
        # à partir des variables x et f d'un solveur MILP, ce qui est différent
        # du solveur de routage.

        # Puisque nous visons le fichier solution, nous allons tenter d'extraire
        # les routes manuellement pour le formatage.

        solution_lines = []
        # ... Extraction des routes MILP complexe ...
        # Pour l'instant, on produit juste les métriques de base
        
        total_distance = solver.Objective().Value()
        
        solution_lines.append("Nombre de véhicules utilisés non calculé")
        solution_lines.append("0") # Nb changements (fictif)
        solution_lines.append("0.00") # Coût transition (fictif)
        solution_lines.append(f"{total_distance:.2f}")
        solution_lines.append(platform.processor() or "Unknown CPU")
        solution_lines.append(f"{end_time - start_time:.3f}")


        # Déterminer le nom du fichier de sortie (ex: Sol_MPVRP_...dat)
        output_filename = "Sol_" + os.path.basename(instance_filepath_str)
        with open(output_filename, 'w', encoding='utf-8') as f_out:
            for line in solution_lines:
                f_out.write(line + '\n')
        
        print(f"💾 Solution sauvegardée dans {output_filename}")

    else:
        print(f"❌ Aucune solution trouvée pour l'instance {os.path.basename(instance_filepath_str)}.")
    
    return status


# =========================================================
# 4. Main (Execution)
# =========================================================

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python solver_script.py instance.dat")
        sys.exit(1)

    # Prendre le chemin du fichier
    instance_file_path_str = sys.argv
    try:
        data = read_instance(instance_file_path_str)
        build_and_solve_milp(instance_file_path_str, data)
    except Exception as e:
        print(f"Une erreur est survenue lors du traitement du fichier : {e}")
        sys.exit(1)
