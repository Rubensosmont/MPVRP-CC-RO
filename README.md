# MPVRP-CC Solver
## Multi-Product Vehicle Routing Problem with Changeover Cost

[![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Status](https://img.shields.io/badge/Status-Operational-success.svg)](.)

Solveur heuristique performant pour le problème MPVRP-CC développé dans le cadre du cours de Recherche Opérationnelle.

---

##  Table des matières

- [Description du problème](#-description-du-problème)
- [Installation](#-installation)
- [Utilisation](#-utilisation)
- [Résultats](#-résultats)
- [Architecture](#-architecture)
- [Documentation](#-documentation)


---

##  Description du problème

Le **MPVRP-CC** est un problème d'optimisation logistique inspiré de la distribution de carburants. Il combine plusieurs défis:

### Caractéristiques principales

- **Multi-produits**: Plusieurs types de carburants à livrer
- **Multi-dépôts**: Sources d'approvisionnement multiples avec stocks limités
- **Flotte hétérogène**: Véhicules avec capacités différentes
- **Contrainte de pureté**: Un véhicule ne transporte qu'un seul produit à la fois
- **Coût de changement**: Nettoyage obligatoire lors du changement de produit
- **Stocks limités**: Chaque dépôt a un stock fini pour chaque produit

### Objectif

Minimiser le coût total = Distance parcourue + Coûts de changement de produit

---

##  Installation

### Prérequis

- Python 3.8 ou supérieur
- Aucune dépendance externe requise 

### Installation rapide

```bash
# Cloner le dépôt
git clone https://github.com/Rubensosmont/MPVRP-CC-RO.git
cd MPVRP-CC-RO

# Le solveur est prêt à l'emploi !
python mpvrp_solver.py <instance.dat>
```

---

##  Utilisation

### Syntaxe de base

```bash
python mpvrp_solver.py <fichier_instance.dat>
```

### Exemples

```bash
# Instance petite (9 stations)
python mpvrp_solver.py MPVRP_S_001_s9_d1_p2.dat

# Instance moyenne (55 stations)
python mpvrp_solver.py MPVRP_M_001_s55_d4_p7.dat

# Instance large (182 stations)
python mpvrp_solver.py MPVRP_L_001_s182_d9_p10.dat
```

### Sortie du programme

```
======================================================================
Resolution de: MPVRP_S_001_s9_d1_p2.dat
======================================================================
Instance chargee:
  3 vehicules
  2 produits
  9 stations
  1 depots

Stocks des depots:
  Depot 1: P0=17411, P1=22128

======================================================================
RESUME DE LA SOLUTION MPVRP-CC
======================================================================

Vehicules utilises: 2
Changements de produit: 4
Cout changement: 75.20
Distance totale: 1066.71
Temps: 0.002s

COUT TOTAL: 1141.91
======================================================================

Verification des demandes:
  Produit 0: OK (14890/14890)
  Produit 1: OK (19553/19553)

Verification des stocks:
  Depot 1 Produit 0: OK (14890/17411)
  Depot 1 Produit 1: OK (19553/22128)

======================================================================
SOLUTION VALIDE - Toutes contraintes respectees
======================================================================

Solution ecrite: Sol_MPVRP_S_001_s9_d1_p2.dat
```

---

##  Résultats

###  Instances Small (S) - 5/5 résolues

| Instance | Stations | Produits | Véhicules | Changements | Distance | Coût total | Temps |
|----------|----------|----------|-----------|-------------|----------|------------|-------|
| S_001    | 9        | 2        | 2/3       | 4           | 1066.71  | 1141.91    | <1ms  |
| S_002    | 10       | 3        | 4/5       | 5           | 2671.71  | 2834.11    | 4ms   |
| S_003    | 12       | 2        | 1/2       | 2           | 2045.98  | 2106.38    | <1ms  |
| S_004    | 9        | 3        | 4/5       | 4           | 1614.35  | 1746.85    | 2ms   |
| S_005    | 7        | 3        | 2/2       | 2           | 788.34   | 879.64     | 2ms   |

###  Instances Medium (M) - 3/3 résolues

| Instance | Stations | Produits | Véhicules | Changements | Distance  | Coût total | Temps  |
|----------|----------|----------|-----------|-------------|-----------|------------|--------|
| M_001    | 55       | 7        | 4/15      | 55          | 55630.73  | 57895.43   | 590ms  |
| M_002    | 52       | 6        | 4/12      | 45          | 49803.27  | 51593.37   | 152ms  |
| M_003    | 59       | 7        | 5/20      | 76          | 66730.57  | 70948.87   | 1820ms |

###  Instances Large (L) - Opérationnelles

Le solveur gère egalement les instances Large (150-200 stations) avec succès.

###  Contraintes respectées à 100%

-  **Demandes satisfaites**: 100% des stations livrées
-  **Capacités véhicules**: Aucun dépassement
-  **Stocks dépôts**: Gestion stricte, zéro excès
-  **Routes valides**: Départ et retour au garage assigné
-  **Unicité produit**: Un seul type à la fois
-  **Changements au dépôt**: Nettoyage uniquement aux dépôts

---

##  Architecture

### Structure du projet

```
MPVRP-CC-RO/
│
├── mpvrp_solver.py              # Solveur principal (530 lignes)
├── README.md                    # Ce fichier
│
├── instances/                   # Fichiers d'instances
│   ├── MPVRP_S_*.dat           # Small (7-15 stations)
│   ├── MPVRP_M_*.dat           # Medium (50-60 stations)
│   └── MPVRP_L_*.dat           # Large (150-200 stations)
│
└── solutions/                   # Solutions générées
    └── Sol_*.dat               # Format conforme
```

### Classes principales

```python
class MPVRPInstance:
    """Charge et valide les données d'instance"""
    - load_instance()        # Parse le fichier .dat
    - euclidean_distance()   # Calcule les distances
    
class CorrectedMPVRPSolver:
    """Résout le problème MPVRP-CC"""
    - solve()                         # Algorithme principal
    - build_vehicle_full_route()      # Construction route véhicule
    - build_segment()                 # Construction segment
    - optimize_station_sequence()     # Optimisation locale
    - write_solution()                # Génération fichier solution
```

### Algorithme

**Approche**: Heuristique constructive gloutonne avec optimisation locale

**Étapes**:
1. **Construction par véhicule**: Routes construites véhicule par véhicule
2. **Segments multiples**: Chaque véhicule peut retourner au dépôt plusieurs fois
3. **Sélection gloutonne**: Choix du segment minimisant le coût à chaque étape
4. **Gestion des stocks**: Vérification et mise à jour en temps réel
5. **Optimisation locale**: 
   - ≤7 stations: permutations complètes (optimal)
   - >7 stations: plus proche voisin (rapide)

**Complexité**: `O(V × D × P × S × log(S))`
- V: véhicules | D: dépôts | P: produits | S: stations

---

##  Documentation

### Format fichier instance (.dat)

```
# UUID unique
nb_products nb_depots nb_garages nb_stations nb_vehicles
[Matrice coûts transition NxN]
[Véhicules: id capacité garage produit_initial]
[Dépôts: id x y stock_p0 stock_p1 ...]
[Garages: id x y]
[Stations: id x y demande_p0 demande_p1 ...]
```

### Format fichier solution (.dat)

```
vehicule: garage - depot[charge] - station(livraison) - ... - garage
vehicule: produit(coût_cumulé) - produit(coût_cumulé) - ...

nb_vehicules_utilises
nb_changements_total
cout_changement_total
distance_totale
processeur
temps_execution
```

### Points clés de l'implémentation

1. **Véhicule part avec produit 0** (état neutre/vide)
   ```python
   current_product = 0  # Départ neutre
   ```

2. **Tout changement a un coût** (y compris 0→produit)
   ```python
   changeover_cost = transition_costs[current_product][new_product]
   ```

3. **Gestion stricte des stocks**
   ```python
   # Vérification avant chargement
   available = depot_stocks[(depot, product)]
   load = min(vehicle_capacity, available)
   # Mise à jour immédiate
   depot_stocks[(depot, product)] -= load
   ```

4. **Comptage correct des changements**
   ```python
   if current_product != new_product:
       nb_changes += 1
   ```

---

### Auteurs
**GROUPE 14 


---

**Dernière mise à jour**: Février 2026  
**Version**: 2.0 - Solveur opérationnel sur toutes tailles d'instances