from ortools.linear_solver import pywraplp

solver = pywraplp.Solver.CreateSolver('GLOP')

x = solver.NumVar(0, solver.infinity(), 'x')
y = solver.NumVar(0, solver.infinity(), 'y')

solver.Add(x + 2 * y <= 14)
solver.Add(3 * x - y >= 0)
solver.Add(x - y <= 2)

solver.Maximize(3 * x + 4 * y)

status = solver.Solve()

if status == pywraplp.Solver.OPTIMAL:
    print("Solution optimale trouvée")
    print(f"x = {x.solution_value()}")
    print(f"y = {y.solution_value()}")
    print(f"Valeur maximale Z = {solver.Objective().Value()}")
else:
    print("Pas de solution optimale trouvée")
    
    
    
    
    
    
    
    
    
    
    
    
    

"""# Problème d'affectation avec coûts 2
from ortools.linear_solver import pywraplp

costs = [
    [90, 76, 75],
    [35, 85, 55],
    [125, 95, 90]
]

solver = pywraplp.Solver.CreateSolver('SCIP')

x = {}
for i in range(3):
    for j in range(3):
        x[i, j] = solver.IntVar(0, 1, f'x_{i}_{j}')

# Chaque personne a une seule tâche
for i in range(3):
    solver.Add(sum(x[i, j] for j in range(3)) == 1)

# Chaque tâche est attribuée à une seule personne
for j in range(3):
    solver.Add(sum(x[i, j] for i in range(3)) == 1)

# Fonction objectif
solver.Minimize(
    sum(costs[i][j] * x[i, j] for i in range(3) for j in range(3))
)

solver.Solve()

print("Affectations optimales :")
total_cost = 0
for i in range(3):
    for j in range(3):
        if x[i, j].solution_value() == 1:
            print(f"Personne {i+1} → Tâche {j+1}")
            total_cost += costs[i][j]

print("Coût total =", total_cost)
"""




"""
#PLNE
from ortools.linear_solver import pywraplp

solver = pywraplp.Solver.CreateSolver('SCIP')

x = solver.IntVar(0, 1, 'x')
y = solver.IntVar(0, 1, 'y')

solver.Add(x + y <= 1)
solver.Maximize(3*x + 2*y)

solver.Solve()

print("x =", x.solution_value())
print("y =", y.solution_value())
print("Z =", solver.Objective().Value())
"""

"""
from ortools.linear_solver import pywraplp

solver = pywraplp.Solver.CreateSolver("SCIP")

x = solver.IntVar(0, 1, "x")
y = solver.IntVar(0, 1, "y")

solver.Add(x + y <= 1)
solver.Maximize(3*x + 2*y)

solver.Solve()

print("x =", x.solution_value())
print("y =", y.solution_value())
print("Z =", solver.Objective().Value())
"""



"""
from ortools.linear_solver import pywraplp

# Créer le solveur
solver = pywraplp.Solver.CreateSolver('GLOP')

# Variables
x = solver.NumVar(0, solver.infinity(), 'x')
y = solver.NumVar(0, solver.infinity(), 'y')

# Contraintes
solver.Add(x + 2*y <= 14)
solver.Add(3*x - y >= 0)
solver.Add(x - y <= 2)

# Objectif
solver.Maximize(3*x + 4*y)

# Résolution
status = solver.Solve()

if status == pywraplp.Solver.OPTIMAL:
    print("Solution optimale")
    print("x =", x.solution_value())
    print("y =", y.solution_value())
    print("Z =", solver.Objective().Value())
else:
    print("Pas de solution")
"""