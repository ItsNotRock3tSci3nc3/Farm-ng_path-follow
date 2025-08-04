import csv
import pandas as pd
import os

RELATIVE_PATH = "./target_data"

YARD_PATH = F"{RELATIVE_PATH}/yard"
SANBORN_PATH = f"{RELATIVE_PATH}/sanborn"



def read_data(file):
    with open(file, 'r') as file:
        csvreader = csv.reader(file)
        df = pd.read_csv(file)
        df.columns = df.columns.str.strip()

        print("Columns in CSV:", df.columns.tolist())
        print(df)


        for lines in csvreader:
            print(lines)

        latitude = df["latitude"]
        longitude = df["longitude"]
        print(f"Lat:\n{latitude} \nLon:\n{longitude}")

    return df


def get_targets():
    file = set_targets()
    df = read_data(file)
    targets = list(zip(df['latitude'], df['longitude']))
    #print(f"return values{targets}")
    return targets

def set_targets():
    print("Available target sets:")
    all_files = {}

    index = 1
    for label, folder in [('YARD', YARD_PATH), ('SANBORN', SANBORN_PATH)]:
        print(f"\n{label} targets:")
        if not os.path.exists(folder):
            print(f"  (No directory: {folder})")
            continue

        for file in os.listdir(folder):
            if file.endswith(".csv"):
                full_path = os.path.join(folder, file)
                all_files[str(index)] = full_path
                print(f"  {index}. {file}")
                index += 1

    if not all_files:
        print("No target CSV files found.")
        return None

    choice = input("\nEnter the number of the target file you want to use: ").strip()
    if choice in all_files:
        print(f"\nSelected: {choice}")
        return all_files[choice]
    
    else:
        print("Invalid selection.")
        return None