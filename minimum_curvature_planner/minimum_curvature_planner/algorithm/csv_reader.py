import csv
import numpy as np
from perception_data import Centreline

def centreline_from_csv(vehicle_width, file_path, delimiter=',', quotechar='"', encoding='utf-8'):
    try:
        with open(file_path, 'r', encoding=encoding) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=delimiter, quotechar=quotechar)
            data = list(csv_reader)
            for i in range(len(data)):
                if (data[i][0][0] == '#'): continue
                data[i] = [float(x) for x in data[i]]
                data = np.array(data[i:len(data)], dtype=np.float64)
                break
            centreline = Centreline(len(data), data[:, 0:2], data[:, 2], vehicle_width)
            return centreline
    except FileNotFoundError:
        print(f"Error: File '{file_path}' not found.")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    # centreline = centreline_from_csv('maps/Budapest_centerline.csv')
    centreline = centreline_from_csv('maps/Spielberg_centerline.csv')
