import sympy as sp
import numpy as np
import control

########## System Definition ##########
# Define symbolic state and input variables
state_var1, state_var2, input_signal = sp.symbols('state_var1 state_var2 input_signal')

# Define the system's differential equations
state_var1_deriv = -state_var1 + state_var2 + 4 * input_signal
state_var2_deriv = -state_var1 - state_var2 + 4 * state_var1 * (state_var2**2) + 2 * input_signal
#######################################

def find_equilibrium_points():
    """
    1. Set the input signal to 0 in both state derivative equations.
    2. Set both state derivative equations to 0 to find steady-state conditions.
    3. Solve the resulting system of equations for the state variables.
    4. Return the calculated steady-state points.
    """
    ###### REFACTORED CODE ###############
    equation1 = sp.Eq(state_var1_deriv.subs(input_signal, 0), 0)
    equation2 = sp.Eq(state_var2_deriv.subs(input_signal, 0), 0)

    steady_state_points = sp.solve((equation1, equation2), (state_var1, state_var2))
    ####################################
    return steady_state_points

def find_A_B_matrices(steady_states):
    """
    1. For each steady-state point, substitute its values into the Jacobian matrices.
    2. Calculate the Jacobian matrices A (system matrix) and B (input matrix).
    3. Return a list of A and B matrices, one for each steady-state point.
    """
    jacobian_A_sym = sp.Matrix([
        [sp.diff(state_var1_deriv, state_var1), sp.diff(state_var1_deriv, state_var2)],
        [sp.diff(state_var2_deriv, state_var1), sp.diff(state_var2_deriv, state_var2)]
    ])
    
    jacobian_B_sym = sp.Matrix([
        [sp.diff(state_var1_deriv, input_signal)],
        [sp.diff(state_var2_deriv, input_signal)]
    ])

    system_matrices_A, input_matrices_B = [], []
    
    ###### REFACTORED CODE ################
    for point in steady_states:
        substitution_map = {state_var1: point[0], state_var2: point[1], input_signal: 0}
        
        evaluated_A = jacobian_A_sym.subs(substitution_map)
        evaluated_B = jacobian_B_sym.subs(substitution_map)
        
        system_matrices_A.append(evaluated_A)
        input_matrices_B.append(evaluated_B)
    ####################################
    
    return system_matrices_A, input_matrices_B

def find_eigen_values(list_of_A_matrices):
    """
    1. Calculate the eigenvalues for each system matrix A.
    2. Determine the stability of the system at each point based on its eigenvalues.
       (Stable if all real parts of eigenvalues are negative).
    3. Return lists of eigenvalues and their corresponding stability statuses.
    """
    eigenvalue_results = []
    stability_status = []

    ###### REFACTORED CODE ################
    for matrix_A in list_of_A_matrices:
        eigenvalue_dictionary = matrix_A.eigenvals()
        eigenvalue_results.append(eigenvalue_dictionary)
        
        is_stable = all(sp.re(eigenval) < 0 for eigenval in eigenvalue_dictionary.keys())
        
        if is_stable:
            stability_status.append("Stable")
        else:
            stability_status.append("Unstable")
    ####################################
    
    return eigenvalue_results, stability_status

def compute_lqr_gain(system_matrix_list_A, input_matrix_list_B):
    """
    This function computes the LQR gain matrix K for the unstable equilibrium point.
    1. Select the Jacobian A and B matrices corresponding to the unstable point.
    2. Define state (Q) and control (R) weighting matrices.
    3. Use the control library's lqr function to compute the gain K.
    """
    lqr_gain_matrix = 0

    # Define the weighting matrices for the LQR controller
    state_weight_matrix = np.eye(2)
    control_weight_matrix = np.array([1])

    # Select the matrices for the unstable equilibrium point (assuming it's the second one)
    unstable_A = np.array(system_matrix_list_A[1]).astype(float)
    unstable_B = np.array(input_matrix_list_B[1]).astype(float)

    ###### REFACTORED CODE ################
    lqr_gain_matrix, _, _ = control.lqr(unstable_A, unstable_B, state_weight_matrix, control_weight_matrix)
    ####################################
    
    return lqr_gain_matrix

def main_function(deriv1, deriv2, inp): # Main orchestrator function
    """
    Executes the full analysis pipeline.
    """
    steady_state_coords = find_equilibrium_points()
    
    if not steady_state_coords:
        print("No steady-state points were found.")
        return None, None, None, None, None
    
    final_A_matrices, final_B_matrices = find_A_B_matrices(steady_state_coords)
    
    final_eigenvalues, final_stability_list = find_eigen_values(final_A_matrices)
    
    final_K_gain = compute_lqr_gain(final_A_matrices, final_B_matrices)
    
    return steady_state_coords, final_A_matrices, final_eigenvalues, final_stability_list, final_K_gain

def task1a_output():
    """
    Prints the final computed results in a formatted way.
    """
    print("Equilibrium Points:")
    for index, coord in enumerate(steady_state_coords):
        print(f"  Point {index + 1}: state_var1 = {coord[0]}, state_var2 = {coord[1]}")
    
    print("\nJacobian Matrices at Equilibrium Points:")
    for index, sys_matrix in enumerate(final_A_matrices):
        print(f"  At Point {index + 1}:")
        print(sp.pretty(sys_matrix, use_unicode=True))
    
    print("\nEigenvalues at Equilibrium Points:")
    for index, eigen_values_map in enumerate(final_eigenvalues):
        eigen_values_string = ', '.join([f"{value}: multiplicity {multiplicity}" for value, multiplicity in eigen_values_map.items()])
        print(f"  At Point {index + 1}: {eigen_values_string}")
    
    print("\nStability of Equilibrium Points:")
    for index, stability_result in enumerate(final_stability_list):
        print(f"  At Point {index + 1}: {stability_result}")
    
    print("\nLQR Gain Matrix K at the selected Equilibrium Point:")
    print(final_K_gain)

if __name__ == "__main__":
    # Execute the main analysis
    analysis_results = main_function(state_var1_deriv, state_var2_deriv, input_signal)
    
    # Unpack the results from the analysis
    steady_state_coords, final_A_matrices, final_eigenvalues, final_stability_list, final_K_gain = analysis_results

    # Print the formatted results
    task1a_output()

