# MPVRP-CC-RO
#  Guide d'Utilisation Rapide - MPVRP-CC Solver

## Installation

```bash
# Installation de OR-Tools (optionnel, mais recommandé)
pip install ortools
```

## Utilisation de Base

### 1. Solution Rapide (Recommandé)
```bash
python mpvrp_complete.py instance.dat
```
✓ Utilise l'algorithme glouton (rapide et efficace)
✓ Valide automatiquement la solution
✓ Sauvegarde dans `solution.dat`

### 2. Spécifier le fichier de sortie
```bash
python mpvrp_complete.py instance.dat -o ma_solution.dat
```

### 3. Utiliser CP-SAT (pour petites instances)
```bash
python mpvrp_complete.py instance.dat -a cpsat
```
✓ Meilleure qualité de solution
✗ Plus lent (recommandé pour < 20 stations)

### 4. Comparer les deux algorithmes
```bash
python mpvrp_complete.py instance.dat -a both
```
✓ Exécute greedy ET cpsat
✓ Garde la meilleure solution

### 5. Augmenter le temps pour CP-SAT
```bash
python mpvrp_complete.py instance.dat -a cpsat -t 600
```
⏱ Limite de temps de 10 minutes (600 secondes)

### 6. Mode silencieux
```bash
python mpvrp_complete.py instance.dat -q
```

### 7. Sans validation
```bash
python mpvrp_complete.py instance.dat --no-validate
```

## Exemples d'Utilisation

### Petite instance (< 20 stations)
```bash
python mpvrp_complete.py MPVRP_S_001_s15_d2_p3.dat -a cpsat -t 300
```

### Instance moyenne (20-50 stations)
```bash
python mpvrp_complete.py MPVRP_M_001_s35_d3_p4.dat -a greedy
```

### Grande instance (> 50 stations)
```bash
python mpvrp_complete.py MPVRP_L_001_s100_d5_p5.dat -a greedy -o solution_large.dat
```

### Batch processing (plusieurs instances)
```bash
for file in instances/*.dat; do
    output="solutions/$(basename $file .dat)_sol.dat"
    python mpvrp_complete.py "$file" -o "$output" -q
done
```

## Aide Complète

```bash
python mpvrp_complete.py --help
```

## Structure de Sortie

Le programme affiche:

```
======================================================================
  MPVRP-CC COMPLETE SOLVER
======================================================================

   Reading instance: instance.dat
   Products: 3, Depots: 2, Garages: 1
   Stations: 20, Vehicles: 5
   Total demand: 15420 units

======================================================================
  GREEDY HEURISTIC SOLVER
======================================================================
   Processing... 40 demands remaining
   Processing... 20 demands remaining

✓ Solution found in 0.045s
   Vehicles used: 3/5
   Total distance: 1385.07
   Product changes: 7
   Changeover cost: 55.66

======================================================================
  VALIDATING SOLUTION
======================================================================

✓ All constraints satisfied

======================================================================
  SOLUTION SUMMARY
======================================================================

   Performance Metrics:
   Vehicles used: 3/5
   Total distance: 1385.07
   Product changes: 7
   Changeover cost: 55.66
   Total cost: 1440.73
   CPU time: 0.045s

   Route Details:
   Vehicle 1 (Garage 1):
      Trips: 2
      Total loaded: 5622 units
      Total deliveries: 8 stops

   Solution saved to: solution.dat

======================================================================
✓ Done!
======================================================================
```

## Options Avancées

### Arguments Disponibles

| Argument | Description | Valeur par défaut |
|----------|-------------|-------------------|
| `instance` | Fichier d'instance (requis) | - |
| `-o, --output` | Fichier de sortie | `solution.dat` |
| `-a, --algorithm` | `greedy`, `cpsat`, ou `both` | `greedy` |
| `-t, --time-limit` | Limite temps CP-SAT (secondes) | `300` |
| `--no-validate` | Sauter la validation | False |
| `-q, --quiet` | Mode silencieux | False |

## Recommandations par Taille d'Instance

| Taille | Stations | Algorithme | Temps estimé |
|--------|----------|------------|--------------|
| Petite | < 20 | `cpsat` | 1-5 min |
| Moyenne | 20-50 | `greedy` | < 10s |
| Grande | > 50 | `greedy` | < 30s |

## Dépannage

### Problème: "OR-Tools not available"
**Solution**: Installez OR-Tools
```bash
pip install ortools
```

### Problème: "Problem is INFEASIBLE"
**Causes possibles**:
- Capacité des véhicules insuffisante
- Stock des dépôts insuffisant
- Demandes incohérentes

**Solution**: Vérifiez votre fichier d'instance

### Problème: Solution invalide
**Solution**: Regardez les erreurs de validation
```bash
python mpvrp_complete.py instance.dat
# Regardez la section VALIDATING SOLUTION
```

### Problème: Temps d'exécution trop long
**Solution**: 
1. Utilisez `greedy` au lieu de `cpsat`
2. Réduisez le temps limite: `-t 60`
3. Utilisez le mode silencieux: `-q`

## Tout-en-Un

Ce fichier unique contient:
✓ Algorithme glouton (rapide)
✓ Solveur CP-SAT (optimal)
✓ Validation automatique
✓ Export au bon format
✓ Statistiques détaillées
✓ Gestion d'erreurs

Pas besoin de fichiers séparés!
