import sys

def extract_variables_from_dimacs(file_path):
    """Extracts a set of all unique variables from a DIMACS file."""
    variables = set()
    with open(file_path, 'r') as f:
        for line in f:
            line = line.strip()
            
            # Skip comments and the problem definition line
            if not line or line.startswith('c') or line.startswith('p'):
                continue
            
            # Read token by token to handle multiple variables per line
            for token in line.split():
                val = int(token)
                if val != 0:
                    # Use absolute value because variables are always positive, 
                    # even if the literal is negated
                    variables.add(abs(val))
                    
    return variables

def get_clauses_subset(file_path, valid_variables):
    """Extracts clauses where ALL variables exist in the valid_variables set."""
    matching_clauses = []
    current_clause = []
    
    with open(file_path, 'r') as f:
        for line in f:
            line = line.strip()
            
            if not line or line.startswith('c') or line.startswith('p'):
                continue
            
            for token in line.split():
                val = int(token)
                
                if val == 0:
                    # A '0' indicates the end of a clause
                    if current_clause: 
                        # Check if every variable in this clause is in our valid_variables set
                        if all(abs(lit) in valid_variables for lit in current_clause):
                            matching_clauses.append(current_clause)
                        # Reset for the next clause
                        current_clause = [] 
                else:
                    current_clause.append(val)
                    
    return matching_clauses

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python filter_dimacs.py <file_for_vars.cnf> <file_for_clauses.cnf>")
        sys.exit(1)

    vars_file = sys.argv[1]
    clauses_file = sys.argv[2]

    # Step 1: Get variables from the first file
    print(f"Extracting variables from {vars_file}...")
    target_vars = extract_variables_from_dimacs(vars_file)
    print(f"Found {len(target_vars)} unique variables.")

    # Step 2: Get matching clauses from the second file
    print(f"Filtering clauses in {clauses_file}...")
    filtered_clauses = get_clauses_subset(clauses_file, target_vars)
    print(f"Found {len(filtered_clauses)} clauses that only use the extracted variables.")

    # Step 3: Output the results (printing the first 10 as an example)
    print("\nMatching clauses:")
    for clause in filtered_clauses:
        for l in clause:
            print(f"{l}", end=" ")
        print("0")