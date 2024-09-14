import csv
import json
import numpy as np
from perception_data import Centreline

def export_solution(points: np.ndarray, output_path: str, format: str = 'csv'):
    if format == 'csv':
        with open(output_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            for point in points:
                writer.writerow(point)
    elif format == 'json':
        json_data = [
            {
                'pose': {
                    'position': {
                        'x': point[0],
                        'y': point[1],
                        'z': 0.0
                    }
                }
            }
            for point in points
        ]
        with open(output_path, mode='w') as file:
            json.dump(json_data, file, indent=4)
    else:
        raise ValueError(f"Unsupported format: {format}")
