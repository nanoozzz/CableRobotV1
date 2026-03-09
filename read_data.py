import csv

# Load points from CSV file
def load_points(csv_file):
    points = []

    with open(csv_file, newline='') as f:
        reader = csv.reader(f)

        for row in reader:
            try:
                x = float(row[0])
                y = float(row[1])
                intensity = float(row[2])
                points.append((x, y, intensity))
            
            except:
                continue # Skip invalid rows
    
    if len(points) < 3:
        raise ValueError("A point requires x, y, and intensity values.")
    
    return points