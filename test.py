import numpy as np



def create_spiral_matrix(n):
    """
    Create a 2D numpy array with a spiral pattern of incremental numbers.

    Parameters:
    - n: Size of the square matrix (n x n). n must be odd.

    Returns:
    - A 2D numpy array with a spiral pattern.
    """
    if n % 2 == 0:
        raise ValueError("n must be odd")

    matrix = np.zeros((n, n), dtype=int)

    mid = n // 2
    current_value = 0

    for layer in range(mid + 1):
        # Top row
        for i in range(mid - layer, mid + layer + 1):
            matrix[mid - layer, i] = current_value
            current_value += 1

        # Right column
        for i in range(mid - layer + 1, mid + layer + 1):
            matrix[i, mid + layer] = current_value
            current_value += 1

        # Bottom row
        for i in range(mid + layer - 1, mid - layer, -1):
            matrix[mid + layer, i] = current_value
            current_value += 1

        # Left column
        for i in range(mid + layer , mid - layer, -1):
            matrix[i, mid - layer] = current_value
            current_value += 1

    return matrix

# Example usage
n = 3  # Example with n=5 (odd)
result_matrix = create_spiral_matrix(n)
print(result_matrix)
